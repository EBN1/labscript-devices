# -*- coding: utf-8 -*-
"""

@author: eric norrgard

Python software for communicating with transfer cavity lock labview VI
"""
from __future__ import print_function, division, unicode_literals, absolute_import

# LABSCRIPT_DEVICES IMPORTS
from labscript_devices import labscript_device, BLACS_tab, BLACS_worker, runviewer_parser

# LABSCRIPT IMPORTS
from labscript import  StaticAnalogQuantity, Device, IntermediateDevice, LabscriptError, Output, config, set_passed_properties
import numpy as np
#import visa
import pyvisa as visa
import os
import socket
import struct

@labscript_device
class LaserTCPIPLock(Device):
    description ='Laser Cavity Lock Frequencies'
    @set_passed_properties(property_names = {"connection_table_properties" : ["port"]})
    def __init__(self, name, port = 2577):
        Device.__init__(self, name, None, None)
        self.BLACS_connection = port
        self.frequencies = []

    def set_frequencies(self, laser_index, frequency):
        self.frequencies.append((laser_index, frequency))

    def generate_code(self, hdf5_file):
        table_dtypes = [('index', 'int32'), ('frequency', float)]
        data = np.array(self.frequencies, dtype=table_dtypes)

        group = self.init_device_group(hdf5_file)

        if self.frequencies:
            group.create_dataset('FREQUENCIES', data=data)


import os
import socket
import struct
# BLACS IMPORTS
from blacs.tab_base_classes import Worker, define_state
from blacs.tab_base_classes import MODE_MANUAL, MODE_TRANSITION_TO_BUFFERED, MODE_TRANSITION_TO_MANUAL, MODE_BUFFERED
from blacs.device_base_class import DeviceTab

from qtutils.qt.QtCore import *
from qtutils.qt.QtGui import *
from qtutils.qt.QtCore import pyqtSignal as Signal

#from qtutils import UiLoader
#import qtutils.icons



@BLACS_tab
class LaserTCPIPLockTab(DeviceTab):
    def initialise_GUI(self):
        #A method to sort the widgets in the UI.  Since everthing in named "XXX YYY number", and number is either 1 or 2 digits, sort by the last 2 characters in the string.  Works fine unless this convention is broken
        def sort(channel):
            flag = channel[-2:]
            flag = int(flag)
            return '%02d'%(flag)
        # Capabilities
        self.num_lasers=16
        self.num_AO=2*self.num_lasers
        self.num_DO=1
        do_prop = {}
        do_prop['Go Home']={}


        self.create_digital_outputs(do_prop)
        do_widgets = self.auto_create_widgets()
        self.auto_place_widgets(('Digital Outputs', do_widgets))
        DO_widgets_by_port={}
        DO_widgets_by_port['Go Home'] = self.create_digital_widgets(do_prop)

        self.ao_base_units     = 'MHz'
        self.ao_base_min       = -(2**31-1)
        self.ao_base_max       = 2**31-1
        self.ao_base_step      = 1
        self.ao_base_decimals  = 3
        AO_proplist = []
        AO_hardware_names = []
        for port_num in {'freq','home'}:
            port_props = {}
            for line in range(self.num_lasers):
                hardware_name = '{} Laser {}'.format(port_num, line)
                port_props[hardware_name] = {'base_unit':self.ao_base_units,
                                         'min':self.ao_base_min,
                                         'step':self.ao_base_step,
                                         'max':self.ao_base_max,
                                         'decimals':self.ao_base_decimals}
                AO_hardware_names.append(hardware_name)
            AO_proplist.append((port_num, port_props))
        for _, AO_prop in AO_proplist:
            self.create_analog_outputs(AO_prop)

        # Manually create the analog output widgets so they are grouped separately
        AO_widgets_by_port = {}
        for port_str, AO_prop in AO_proplist:
            AO_widgets_by_port[port_str] = self.create_analog_widgets(AO_prop)

        widget_list = []
        for port_str in {'freq','home'}:
            AO_widgets = AO_widgets_by_port[port_str]
            name = "Laser %s" % port_str
            widget_list.append((name, AO_widgets, sort))
        widget_list.append(("Go Home", DO_widgets_by_port['Go Home']))
        self.auto_place_widgets(*widget_list)
        #Create the BLACS output objects
        # Create the output objects

        #self.create_digital_outputs(do_prop)

        # Define the sort function for the digital outputs


        # Create widgets for output objects
        #dds_widgets,ao_widgets,do_widgets = self.auto_create_widgets()
        #self.auto_place_widgets(('Laser Frequencies', ao_widgets,sort_freq),('Laser Home Frequencies', ao_widgets,sort_home))


        # Store the Measurement and Automation Explorer (MAX) name
        #self.MAX_name = str(self.settings['connection_table'].find_by_name(self.device_name).BLACS_connection)
        # Create and set the primary worker

        self.create_worker("main_worker",LaserTCPIPLockWorker,{'port':int(self.BLACS_connection),'num_AO':self.num_AO})
        self.primary_worker = "main_worker"


        # Set the capabilities of this device
        self.supports_remote_value_check(False)
        self.supports_smart_programming(False)


@BLACS_worker
class LaserTCPIPLockWorker(Worker):
    def init(self):
        global socket; import socket
        global h5py; import labscript_utils.h5_lock, h5py
        global queue; import queue
        global time; import time
        global threading; import threading
        self.num_lasers=16
        self.num_AO=2*self.num_lasers

        self.current_frequencies = {}

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.settimeout(1)
        self.server.bind(('', int(self.port)))  #the first argument '' says to accept any net address, second argurment is the TCP port
        self.server.listen(1)  #Accept 1 connection on the port


    def set_remote_frequencies(self, values):
        print(values)
        # Set the currently output frequencies, where values is a dictionary of values:
        #remote_values = [self.get_freq(), self.get_power(), self.get_is_rf_high(), self.get_is_rf_on()]
        #return remote_values
        gf=np.array([values['freq Laser 0'] , values['freq Laser 1'] , values['freq Laser 2'] , values['freq Laser 3'] , values['freq Laser 4'] , values['freq Laser 5'] ,values['freq Laser 6'] ,values['freq Laser 7'] ,values['freq Laser 8'] ,values['freq Laser 9'] ,values['freq Laser 10'] ,values['freq Laser 11'] ,values['freq Laser 12'] ,values['freq Laser 13'] ,values['freq Laser 14'] ,values['freq Laser 15'] ])
        message=struct.pack('>ffffffffffffffff',gf[0],gf[1],gf[2],gf[3],gf[4],gf[5],gf[6],gf[7],gf[8],gf[9],gf[10],gf[11],gf[12],gf[13],gf[14],gf[15])
        length=struct.pack('>i',len(message))
        #print('I will attempt to send the message length: {}'.format(length))
        try:
            conn, addr = self.server.accept()
            conn.sendall(length)
            #print('Length sent')
            #print('I will attempt to send the message: {}'.format(message))
            time.sleep(.002)
            conn.sendall(message)

            #print('Message sent')
        except socket.timeout as err:
            pass
        except ConnectionAbortedError: #Not Standard Python
            print('Got that stupid ConnectionAbortedError again')
            self.server.close()
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server.settimeout(1)
            self.server.bind(('', int(self.port)))  #the first argument '' says to accept any net address, second argurment is the TCP port
            self.server.listen(1)  #Accept 1 connection on the port
            try:
                conn, addr = self.server.accept()
                conn.sendall(length)
                #print('Length sent')
                #print('I will attempt to send the message: {}'.format(message))
                conn.sendall(message)
                #print('Message sent')
            except socket.timeout as err:
                pass
            except ConnectionAbortedError: #Not Standard Python
                print('Got that stupid ConnectionAbortedError twice, I give up')
        self.current_frequencies=values
        return values



    def program_manual(self, front_panel_values):
        # TODO: Optimise this so that only items that have changed are reprogrammed by storing the last programmed values
        # For each parameter,

        #If 'Go Home' is low, then send the latest Laser freq and Laser home values to set_remote_frequencies
        #If 'Go Home' is high, check if Laser freq equals Laser home.
        #If equal, do nothing.  If not, then iteratively adjust Laser freq until it equals Laser home
        new_values = {}
        step=10 #we'll step the frequency this amount in MHz
        t_btw_steps = .4 #time between steps.  Note that the Labview VI only checks every .2 s
        try:
            new_values=front_panel_values
            if front_panel_values['Go Home']!=0:
                freq_array=np.zeros(self.num_lasers)
                home_array=np.zeros(self.num_lasers)
                while new_values['Go Home']==1:
                    for i in range(self.num_lasers):
                        freq_array[i]=new_values['freq Laser %d'%i]
                        home_array[i]=new_values['home Laser %d'%i]
                    if np.array_equal(freq_array, home_array):

                        new_values['Go Home']=0
                        break
                    for i in range(self.num_lasers):
                        new_values['home Laser %d'%i]=front_panel_values['home Laser %d'%i]
                        if abs(front_panel_values['home Laser %d'%i]-front_panel_values['freq Laser %d'%i]) < step:
                            new_values['freq Laser %d'%i]=front_panel_values['home Laser %d'%i]
                        else:
                            new_values['freq Laser %d'%i]=front_panel_values['freq Laser %d'%i] + np.sign(front_panel_values['home Laser %d'%i]-front_panel_values['freq Laser %d'%i])*step
                    new_values=self.set_remote_frequencies(new_values)
                    time.sleep(t_btw_steps)
            self.set_remote_frequencies(new_values)
        except visa.VisaIOError as e:
            if str(e).startswith('VI_ERROR_SYSTEM_ERROR'):
                print('ERROR: Try waiting a bit longer between commands to the windfreak device')
            else:
                raise e
        self.current_frequencies = new_values

        return new_values


    def transition_to_buffered(self, device_name, h5file, initial_values, fresh):
        self.h5file = h5file

        with h5py.File(h5file,'r') as hdf5_file:
            group = hdf5_file['devices/'][device_name]
            #device_properties = labscript_utils.properties.get(hdf5_file, device_name, 'device_properties')
            #connection_table_properties = labscript_utils.properties.get(hdf5_file, device_name, 'connection_table_properties')
            #frequencies = connection_table_properties['LaserTCPIPLock']
            h5_data = group.get('FREQUENCIES')
            if h5_data:
                new_values = initial_values
                print(initial_values)
                for laser in h5_data:
                    print('Updating laser %d to %f' % (laser['index'], laser['frequency']))
                    new_values['freq Laser %d' % laser['index']] = laser['frequency']
                print('Now setting remote frequencies:')
                print(new_values)
                self.set_remote_frequencies(new_values)
                self.current_frequencies = new_values

        return new_values
        """# Store the initial values in case we have to abort and restore them:
        self.initial_values = initial_values
        self.current_frequencies=initial_values
        print('Transitioning to Buffered')
        print('Initial Values {}'.format(initial_values))
        print('Current Frequencies {}'.format(self.current_frequencies))
        self.device_name = device_name

        self.comm_queue = Queue.Queue()
        self.results_queue = Queue.Queue()
        start_time = time.time()
        self.read_thread = threading.Thread(args = (self.comm_queue, self.results_queue, start_time))
        self.read_thread.daemon=True
        self.read_thread.start()

        def set_frequency(self,freq_dict):
            for key in freq_dict:
                self.current_frequencies[key]=freq_dict[key]
            set_remote_frequencies(self.current_frequencies)

        final_values={}


        return final_values"""


    def transition_to_manual(self):
        print('Transitioning to Manual')
        print('Current Frequencies {}'.format(self.current_frequencies))
        """self.comm_queue.put('exit')
        laser_data = np.array([self.initial_values])
        #print(laser_data)
        with h5py.File(self.h5file) as hdf5_file:
            group = hdf5_file.create_group('/data/'+ self.device_name)
            group.create_dataset('LaserTCPIPLock', data = laser_data)
        self.read_thread.join(1.0)
        self.comm_queue.get(timeout=4)"""
        return True


    def abort_buffered(self):
        return True

    def abort_transition_to_buffered(self):
        return True

    def shutdown(self):
        self.server.close()
        return
