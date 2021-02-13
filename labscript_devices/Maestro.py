# -*- coding: utf-8 -*-
"""

@author: eric norrgard

Python software for communicating with Pololu Maestro servo controllers
Can move to a set Open or Close position, or to a target position
"""
from __future__ import print_function, division, unicode_literals, absolute_import


# LABSCRIPT_DEVICES IMPORTS
from labscript_devices import labscript_device, BLACS_tab, BLACS_worker, runviewer_parser

# LABSCRIPT IMPORTS
from labscript import  Device, IntermediateDevice, LabscriptError, Output, config, set_passed_properties, TriggerableDevice
import numpy as np
import pyvisa as visa
import os
import serial

# BLACS IMPORTS
from blacs.tab_base_classes import Worker, define_state
from blacs.tab_base_classes import MODE_MANUAL, MODE_TRANSITION_TO_BUFFERED, MODE_TRANSITION_TO_MANUAL, MODE_BUFFERED
from blacs.device_base_class import DeviceTab

from qtutils.qt.QtCore import *
from qtutils.qt.QtGui import *
from qtutils.qt.QtCore import pyqtSignal as Signal



#MultiHarp IMPORTS
import time
import ctypes as ct
from ctypes import byref


# Variables to store information read from DLLs

dev=[]


@labscript_device
class Maestro(Device):
    description ='MultiHarp 150'
    @set_passed_properties(property_names = {"connection_table_properties": ["visa_resource","device","num_ch"]})

    def __init__(self, name, visa_resource='COM10', device=12, num_ch=6, **kwargs):
        Device.__init__(self, name, None, visa_resource, **kwargs)
        print('Made it to the labscript_device __init__ !!!')
        self.BLACS_connection = visa_resource
        self.num_ch=num_ch

        self.target = []


    def set_target(self, channel, target):
        self.target.append((self.name, channel, target))

    def generate_code(self, hdf5_file):
        group = self.init_device_group(hdf5_file)
        if self.target:
            table_dtypes = [('name','a256'), ('channel',uint), ('target',unit)]
            data = np.array(self.target,dtype=table_dtypes)
            group.create_dataset('TARGETS', data=data)



@BLACS_tab
class MaestroTab(DeviceTab):

    def initialise_GUI(self):
        print('Made it to initialise_GUI!!!')



        #A method to sort the widgets in the UI.  Since everthing in named "XXX YYY number", and number is either 1 or 2 digits, sort by the last 2 characters in the string.  Works fine unless this convention is broken
        def sort(channel):
            flag = channel[-2:]
            flag = int(flag)
            return '%02d'%(flag)
        # Capabilities
        worker_initialisation_kwargs = self.connection_table.find_by_name(self.device_name).properties
        self.BLACS_connection = self.settings['connection_table'].find_by_name(self.device_name).BLACS_connection
        self.num_ch=worker_initialisation_kwargs['num_ch']
        #worker_initialisation_kwargs['num_ch']=self.num_ch #6 #number of channels,
        self.num_AO=3*self.num_ch  #For each channel,  Target, Open, Close
        self.num_DO=2*self.num_ch  #For each channel, mode, open?

        #mode:0 reference position to target
        #mode:1 reference position to open/close

        do_proplist=[]
        do_hardware_names=[]
        for port_num in {'mode','open?'}:
            port_props={}
            for line in range(self.num_ch):
                hardware_name = '{} Ch {}'.format(port_num, line)
                do_hardware_names.append(hardware_name)
                port_props[hardware_name]={}
            do_proplist.append((port_num, port_props))
        for _, do_prop in do_proplist:
            self.create_digital_outputs(do_prop)

        do_widgets_by_port = {}
        for port_str, do_prop in do_proplist:
            do_widgets_by_port[port_str] = self.create_digital_widgets(do_prop)

        widget_list = []
        for port_str in {'mode','open?'}:
            do_widgets = do_widgets_by_port[port_str]
            name = "%s" % port_str
            widget_list.append((name, do_widgets, sort))

        #self.auto_place_widgets(*widget_list)


        self.ao_base_units     = ''
        self.ao_base_min       = 0
        self.ao_base_max       = 10000
        self.ao_base_step      = 1
        self.ao_base_decimals  = 1
        AO_proplist = []
        AO_hardware_names = []
        #make separate for loops for paramters with mV and ps times.
        for port_num in {'target', 'open', 'close'}:
            port_props = {}
            for line in range(self.num_ch):
                hardware_name = '{} Ch {}'.format(port_num, line)
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

        #widget_list = []
        for port_str in {'target', 'open', 'close'}:
            AO_widgets = AO_widgets_by_port[port_str]
            name = "%s" % port_str
            widget_list.append((name, AO_widgets, sort))
        self.auto_place_widgets(*widget_list)

        self.supports_remote_value_check(False)
        self.supports_smart_programming(False)
        self.create_worker("main_worker", MaestroWorker,worker_initialisation_kwargs)
        self.primary_worker = "main_worker"

#@BLACS_worker
class MaestroWorker(Worker):
    def init(self):
    #def init(self, name, visa_resource = 'COM5', baud=115200, timeout=1, termination='\n'):
        global h5py; import labscript_utils.h5_lock, h5py
        global time; import time
        global threading; import threading
        global serial; import serial
        print('Made it to the Blacs_worker!!!')

        m=serial.Serial(port=self.visa_resource,baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=10)
        try:
            m.close()
            m.open()
            print('Opened serial connection on %s' % self.visa_resource)
        except serial.SerialException as e:
            raise # -*- coding: utf-8 -*-



    def close(self):
        m.close()
        exit(0)

    def set_position (self,device, channel, target):
        m=serial.Serial(port=self.visa_resource,baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=10)
        command = bytes([0xAA, device, 0x84 & 0x7F, channel, target & 0x7F, (target >> 7) & 0x7F])
        m.write(command)




    def program_manual(self, values):
        #print(values)
        for channel in range(self.num_ch):
            if values['mode Ch %d' % channel]==0:
                for port_num in {'target'}:#This also handles 'Level'
                    hardware_name = '{} Ch {}'.format(port_num, channel)
                    print('Setting Ch %d position to target  %r' % (channel, int(values[hardware_name])))
                    self.set_position(device=self.device, channel=channel, target=int(values[hardware_name] ))
            if values['mode Ch %d' % channel]==1:
                if values['open? Ch %d' % channel]==0:
                    hardware_name = 'close Ch {}'.format(channel)
                    print('Setting Ch %d position to close  %r' % (channel, int(values[hardware_name])))
                    self.set_position(device=self.device, channel=channel, target=int(values[hardware_name] ))
                else:
                    hardware_name = 'open Ch {}'.format(channel)
                    print('Setting Ch %d position to open  %r' % (channel, int(values[hardware_name])))
                    self.set_position(self.device, channel, int(values[hardware_name] ))
        return values


    def transition_to_buffered(self, device_name, h5file, initial_values, fresh):
        self.h5_file = h5file
        new_values=initial_values
        with h5py.File(h5file,'r') as hdf5_file:
            group = hdf5_file['devices/'][device_name]
            h5_data = group.get('TARGETS')
            if h5_data:
                for datum in h5_data:
                    new_values['target Ch {}'.format(datum['channel'])] = datum['target']
                    self.set_position(device=self.device, channel=datum['channel'], target= datum['target'] )
        return new_values


    def transition_to_manual(self):
        return True


    def abort_buffered(self):
        return True

    def abort_transition_to_buffered(self):
        return True

    def shutdown(self):
        self.close()
        return
