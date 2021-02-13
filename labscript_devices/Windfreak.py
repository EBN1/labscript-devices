# -*- coding: utf-8 -*-
"""

@author: eric norrgard

Python software for communicating with windfreak USB2
"""
from __future__ import print_function, division, unicode_literals, absolute_import
#from labscript_utils import PY2
#if PY2:
#    str = unicode

# LABSCRIPT_DEVICES IMPORTS
from labscript_devices import labscript_device, BLACS_tab, BLACS_worker, runviewer_parser

# LABSCRIPT IMPORTS
from labscript import  StaticAnalogQuantity, Device, IntermediateDevice, LabscriptError, Output, config, set_passed_properties
import numpy as np
#import visa
import matplotlib
import pyvisa as visa
import os

@labscript_device
class Windfreak(Device):
    description ='Windfreak SynthUSB II'
    @set_passed_properties(property_names = {"connection_table_properties" : ["visa_resource"]})

    def __init__(self, name, visa_resource = 'COM5'):
        Device.__init__(self, name, None, None, visa_resource)
        self.BLACS_connection = visa_resource
        self.frequencies = []

    def set_f(self, laser_index, frequency):
        self.frequencies.append((laser_index, frequency))
        print('Appended laser index{} and frequency {}'.format(laser_index,frequency))


    def generate_code(self, hdf5_file):
        table_dtypes = [('index', int), ('frequency', float)]
        data = np.array(self.frequencies, dtype=table_dtypes)

        group = self.init_device_group(hdf5_file)

        if self.frequencies:
            group.create_dataset('FREQUENCIES', data=data)
            print('Created a data set {}'.format(data))




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
class WindfreakTab(DeviceTab):
    def initialise_GUI(self):
        # Capabilities
        self.num_AO=2
        self.num_DO=2
        ao_prop = {}
        do_prop = {}

        #Define channel hardware parameters
        ao_prop['freq'] = {'base_unit':'MHz', 'min':34, 'max':4400, 'step':0.001, 'decimals':3}
        ao_prop['power'] = {'base_unit':'Power', 'min':0, 'max':3, 'step':1, 'decimals':0}

        do_prop['high'] ={}
        do_prop['is_on']={}

        #Create the BLACS output objects
        # Create the output objects
        self.create_analog_outputs(ao_prop)
        self.create_digital_outputs(do_prop)
        # Create widgets for output objects
        dds_widgets,ao_widgets,do_widgets = self.auto_create_widgets()
        self.auto_place_widgets(('Analog Outputs', ao_widgets),'Digital Outputs', do_widgets)


        # Store the Measurement and Automation Explorer (MAX) name
        #self.MAX_name = str(self.settings['connection_table'].find_by_name(self.device_name).BLACS_connection)
        # Create and set the primary worker

        self.create_worker("main_worker",WindfreakWorker,{'visa_resource':str(self.BLACS_connection),'num_AO':self.num_AO,'num_DO':self.num_DO})
        self.primary_worker = "main_worker"


        # Set the capabilities of this device
        self.supports_remote_value_check(False)
        self.supports_smart_programming(False)



@BLACS_worker
class WindfreakWorker(Worker):
    def init(self):
    #def init(self, name, visa_resource = 'COM5', baud=115200, timeout=1, termination='\n'):
        global visa; import visa
        global h5py; import labscript_utils.h5_lock, h5py
        global time; import time
        global threading; import threading

        self.open = visa.ResourceManager().open_resource(self.visa_resource, baud_rate=115200,  timeout=4000)
        self.open.read_termination = '\n'
        #self.open.timeout = 1000 * timeout
        #initial_values=self.check_remote_values()
        #print(initial_values)



    def get_freq(self):
        return float(self.open.query('f?'))/1000

    def set_freq(self,value):
        self.open.write('f' + str(value))
        return self.get_freq()


    def get_power(self):
        return int(self.open.query('a?'))

    def set_power(self,value):
        self.open.write('a' + str(value))
        return self.get_power()


    def rf_on(self):
        self.open.write('o1')

    def rf_off(self):
        self.open.write('o0')

    def get_is_rf_on(self):
        return int(self.open.query('o?'))
    def set_rf_on(self,value):
        self.open.write('o' + str(value))
        return self.get_is_rf_on()



    def rf_power_low(self):
        self.open.write('h0')

    def rf_power_high(self):
        self.open.write('h1')

    def get_is_rf_high(self):
        return int(self.open.query('h?'))

    def set_is_rf_high(self,value):
        self.open.write('h' + str(value))
        return self.get_is_rf_high()



    def set_pulse_mode(self,value):
        self.open.write('j' + str(value))

    def get_pulse_mode(self,value):
        return self.open.query('j?')


    def check_osci(self):
        return self.open.query('p')

    def set_clock(self,value):
        return float(self.open.query('x' + str(value)))

    def get_clock(self):
        return float(self.open.query('x?'))

    def serial_number(self):
        return self.open.query('+')


    def program_manual(self,front_panel_values):
        self.set_values(front_panel_values)
        # TODO: Optimise this so that only items that have changed are reprogrammed by storing the last programmed values
        # For each parameter,
    def set_values(self,values):
        pause_freak=.5
        new_values = {}
        print(values)
        try:
            new_values['freq']=self.set_freq(values['freq'])
            time.sleep(pause_freak)
            new_values['power']=self.set_power(values['power'])
            time.sleep(pause_freak)
            new_values['high']=self.set_is_rf_high(int(values['high']))
            time.sleep(pause_freak)
            new_values['is_on']=self.set_rf_on(int(values['is_on']))
        except visa.VisaIOError as e:
            if str(e).startswith('VI_ERROR_SYSTEM_ERROR'):
                print('ERROR: Try waiting a bit longer between commands to the windfreak device')
            else:
                raise e
        print(new_values)
        return new_values





    def transition_to_buffered(self, device_name, h5file, initial_values, fresh):
        self.h5file = h5file
        new_values={}
        with h5py.File(h5file,'r') as hdf5_file:
            group = hdf5_file['devices/'][device_name]
            #device_properties = labscript_utils.properties.get(hdf5_file, device_name, 'device_properties')
            #connection_table_properties = labscript_utils.properties.get(hdf5_file, device_name, 'connection_table_properties')
            h5_data = group.get('FREQUENCIES')
            if h5_data:
                new_values = initial_values
                print(initial_values)
                print(h5_data)
                for laser in h5_data:
                    if laser['index'] == 1:
                        new_values['freq'] = laser['frequency']
                print('Now setting remote frequencies:')
                print(new_values)
        return self.set_values(new_values)



    def transition_to_manual(self):
        return True


    def abort_buffered(self):
        return True

    def abort_transition_to_buffered(self):
        return True

    def shutdown(self):
        visa.ResourceManager().close()
        return
