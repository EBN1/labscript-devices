# -*- coding: utf-8 -*-
"""

@author: eric norrgard

Python software for communicating with MultiHarp 150
Implements the Histogram mode of operation during Buffered Mode
Can set up device parameters during Manual Mode
"""
from __future__ import print_function, division, unicode_literals, absolute_import
#from labscript_utils import PY2
#if PY2:
#    str = unicode

# LABSCRIPT_DEVICES IMPORTS
from labscript_devices import labscript_device, BLACS_tab, BLACS_worker, runviewer_parser

# LABSCRIPT IMPORTS
from labscript import  Device, IntermediateDevice, LabscriptError, Output, config, set_passed_properties, TriggerableDevice
import numpy as np
#import visa
import os


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



# From mhdefin.h
LIB_VERSION = "1.0"
MAXDEVNUM = 8
MODE_HIST = 0
MAXLENCODE = 6
MAXINPCHAN = 8
MAXHISTLEN = 65536
FLAG_OVERFLOW = 0x001

# Measurement parameters, these are hardcoded since this is just a demo

offset = 0
tacq = 1000 # Measurement time in millisec, you can change this
syncDivider = 1 # you can change this
syncChannelOffset = 0#-5000 # you can change this (in ps, like a cable delay)
inputChannelOffset = 0 # you can change this (in ps, like a cable delay)
cmd = 0

# Variables to store information read from DLLs


mhlib = ct.CDLL("mhlib64.dll")
dev=[]


@labscript_device
class MultiHarp(TriggerableDevice):
    description ='MultiHarp 150'
    @set_passed_properties(property_names = {"connection_table_properties": ["serial_number",'meas_control','t_acquire']})

    def __init__(self, name, parent_device, connection, serial_number,t_acquire=1e-3,meas_control=1,**kwargs):
        TriggerableDevice.__init__(self, name, parent_device, connection, **kwargs)
        print('Made it to the labscript_device __init__ !!!')
        self.BLACS_connection = serial_number #Use the first device found
        print('Blacs will use S/N %s' % self.BLACS_connection)
        self.minimum_recovery_time=0.1
        self.t_acquire=t_acquire

        self.edges = []
        self.levels = []
        self.offsets=[]
        self.trigger_out=[]
        self.binning=[]
        self.meas_control=meas_control
        self.acquisitions=[]


    def acquire(self, name, t, t_acquire):
        self.trigger_device.trigger(t,t_acquire)
        start, end = t, t+t_acquire
        for exposure in self.acquisitions:
            _, other_t, other_duration = exposure
            other_start = other_t
            other_end = other_t + other_duration
            if abs(other_start - end) < self.minimum_recovery_time or abs(other_end - start) < self.minimum_recovery_time:
                raise LabscriptError('%s %s has two acquisitions closer together than the minimum recovery time: ' %(self.description, self.name) + \
                                     'one at t = %fs for %fs, and another at t = %fs for %fs. '%(t,t_acquire,start,t_acquire) + \
                                     'The minimum recovery time is %fs.'%self.minimum_recovery_time)
        self.acquisitions.append((name, t, t_acquire))
        return t_acquire



    def set_edge(self, channel, edge):
        self.edges.append((channel, edge))
    def set_level(self, channel, level):
        self.levels.append((channel, level))
    def set_offset(self, channel, offset):
        self.offsets.append((channel, offset))
    def set_trigger_out(self, period):
        self.trigger_out=[period]
    def set_binning(self, binning):
        self.binning=[binning]
    #def set_meas_control(self,meas_control):
    #    self.meas_control=meas_control

    def generate_code(self, hdf5_file):
        group = self.init_device_group(hdf5_file)
        if self.acquisitions:
            table_dtypes = [('name','a256'), ('time',float), ('t_acquire',float)]
            data = np.array(self.acquisitions,dtype=table_dtypes)
            group.create_dataset('ACQUISITIONS', data=data)
            # DEPRECATED backward campatibility for use of exposuretime keyword argument instead of exposure_time:
            self.set_property('t_acquire', self.t_acquire, location='device_properties', overwrite=True)
        if self.edges:
            table_dtypes = [('Ch', 'int'), ('Edge', 'bool')]
            data = np.array(self.edges, dtype=table_dtypes)
            group.create_dataset('EDGES', data=data)
        if self.levels:
            table_dtypes = [('Ch', 'int'), ('Level', 'int')]
            data = np.array(self.levels, dtype=table_dtypes)
            group.create_dataset('LEVELS', data=data)
        if self.offsets:
            table_dtypes = [('Ch', 'int'), ('Offset', 'int')]
            data = np.array(self.offsets, dtype=table_dtypes)
            group.create_dataset('OFFSETS', data=data)
        if not self.trigger_out:
            self.trigger_out=[0]
        if self.trigger_out:
            table_dtypes = [('Period x100 ns', 'int')]
            data = np.array(self.trigger_out, dtype=table_dtypes)
            group.create_dataset('TRIGGER OUT', data=data)
        if self.binning:
            table_dtypes = [('Binning', 'uint')]
            data = np.array(self.binning, dtype=table_dtypes)
            group.create_dataset('BINNING', data=data)
        if self.meas_control:
            if self.meas_control not in range(4):
                raise Error('Parameter meas_control may only be an integer 0 through 4')
            table_dtypes = [('Measurement Control', 'int')]
            data = np.array([self.meas_control], dtype=table_dtypes)
            group.create_dataset('MEASUREMENT CONTROL', data=data)


@BLACS_tab
class MultiHarpTab(DeviceTab):

    def initialise_GUI(self):
        print('Made it to initialise_GUI!!!')



        #A method to sort the widgets in the UI.  Since everthing in named "XXX YYY number", and number is either 1 or 2 digits, sort by the last 2 characters in the string.  Works fine unless this convention is broken
        def sort(channel):
            flag = channel[-2:]
            flag = int(flag)
            return '%02d'%(flag)
        # Capabilities
        self.num_ch=5 #number of channels, with SYNC being CH 0
        self.num_AO=2*self.num_ch  #For each channel,  Level, Offset
        self.num_DO=1*self.num_ch  #For each channel, Edge
        worker_initialisation_kwargs = self.connection_table.find_by_name(self.device_name).properties
        self.BLACS_connection = self.settings['connection_table'].find_by_name(self.device_name).BLACS_connection
        worker_initialisation_kwargs['num_ch']=self.num_ch


        do_proplist=[]
        do_hardware_names=[]
        for port_num in {'Edge'}:
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
        for port_str in {'Edge'}:
            do_widgets = do_widgets_by_port[port_str]
            name = "%s" % port_str
            widget_list.append((name, do_widgets, sort))

        #self.auto_place_widgets(*widget_list)


        self.ao_base_units     = 'mV'
        self.ao_base_min       = -1000
        self.ao_base_max       = 1000
        self.ao_base_step      = 1
        self.ao_base_decimals  = 1
        AO_proplist = []
        AO_hardware_names = []
        #make separate for loops for paramters with mV and ps times.
        for port_num in {'Level'}:
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
        self.ao_base_units     = 'ps'
        self.ao_base_min       = 0
        self.ao_base_max       = 99999
        self.ao_base_step      = 1
        self.ao_base_decimals  = 0
        for port_num in {'Offset'}:
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
        for port_str in {'Level','Offset'}:
            AO_widgets = AO_widgets_by_port[port_str]
            name = "%s" % port_str
            widget_list.append((name, AO_widgets, sort))
        self.auto_place_widgets(*widget_list)

        self.supports_remote_value_check(False)
        self.supports_smart_programming(False)


    #def initialise_workers(self):
        #self.create_worker("main_worker", MultiHarpWorker,{'serial_number':str(self.BLACS_connection),'num_ch':self.num_ch})
        self.create_worker("main_worker", MultiHarpWorker,worker_initialisation_kwargs)

        self.primary_worker = "main_worker"

#@BLACS_worker
class MultiHarpWorker(Worker):
    def init(self):
    #def init(self, name, visa_resource = 'COM5', baud=115200, timeout=1, termination='\n'):
        global h5py; import labscript_utils.h5_lock, h5py
        #global Queue; import Queue
        global time; import time
        global threading; import threading
        print('Made it to the Blacs_worker!!!')

        self.h5_file = None

        self.counts = [(ct.c_uint * MAXHISTLEN)() for i in range(0, 4)]
        libVersion = ct.create_string_buffer(b"", 8)
        hwSerial = ct.create_string_buffer(b"", 8)
        hwPartno = ct.create_string_buffer(b"", 8)
        hwVersion = ct.create_string_buffer(b"", 8)
        hwModel = ct.create_string_buffer(b"", 16)
        errorString = ct.create_string_buffer(b"", 40)
        numChannels = ct.c_int()
        histLen = ct.c_int()
        self.resolution = ct.c_double()
        self.syncRate = ct.c_int()
        self.countRate = ct.c_int()
        flags = ct.c_int()
        warnings = ct.c_int()
        warningstext = ct.create_string_buffer(b"", 16384)

        def closeDevices():
            for i in range(0, MAXDEVNUM):
                mhlib.MH_CloseDevice(ct.c_int(i))
            exit(0)

        def tryfunc(retcode, funcName):
            if retcode < 0:
                mhlib.MH_GetErrorString(errorString, ct.c_int(retcode))
                print("MH_%s error %d (%s). Aborted." % (funcName, retcode,
                      errorString.value.decode("utf-8")))
                closeDevices()
        print('Now looking for MultiHarp S/N %s' % self.serial_number)
        print("Devidx     Status")
        for i in range(0, MAXDEVNUM):
            retcode = mhlib.MH_OpenDevice(ct.c_int(i), hwSerial)
            if retcode == 0:
                print("  %1d        S/N %s" % (i, hwSerial.value.decode("utf-8")))
                if int(hwSerial.value.decode("utf-8"))==int(self.serial_number):
                    dev.append(i)
            else:
                if retcode == -1: # MH_ERROR_DEVICE_OPEN_FAIL
                    print("  %1d        no device" % i)
                else:
                    mhlib.MH_GetErrorString(errorString, ct.c_int(retcode))
                    print("  %1d        %s" % (i, errorString.value.decode("utf8")))
        print('Proceeding with Device %r' % dev)
        tryfunc(mhlib.MH_Initialize(ct.c_int(dev[0]), ct.c_int(MODE_HIST), ct.c_int(0)),
        "Initialize")
        tryfunc(mhlib.MH_GetHardwareInfo(dev[0], hwModel, hwPartno, hwVersion),
                "GetHardwareInfo")
        print("Initialized Model %s Part no %s Version %s" % (hwModel.value.decode("utf-8"),
              hwPartno.value.decode("utf-8"), hwVersion.value.decode("utf-8")))
        tryfunc(mhlib.MH_SetMeasControl(ct.c_int(dev[0]),ct.c_int(self.meas_control),ct.c_int(1),ct.c_int(1)),"SetMeasControl")
        print('Measurement Control Settings are set with mode %d' % self.meas_control)
        print('t_acquire = %f s' % self.t_acquire)
    def closeDevices():
        for i in range(0, MAXDEVNUM):
            mhlib.MH_CloseDevice(ct.c_int(i))
        exit(0)

    def tryfunc(retcode, funcName):
        if retcode < 0:
            mhlib.MH_GetErrorString(errorString, ct.c_int(retcode))
            print("MH_%s error %d (%s). Aborted." % (funcName, retcode,
                  errorString.value.decode("utf-8")))
            closeDevices()

    def mhSetInputEdgeTrg(self,device, channel, level, edge):
        return mhlib.MH_SetInputEdgeTrg(ct.c_int(device), ct.c_int(channel),ct.c_int(int(level)),ct.c_int(int(edge)))
    def mhSetSyncEdgeTrg(self,device, level, edge):
        return mhlib.MH_SetSyncEdgeTrg(ct.c_int(device),ct.c_int(int(level)),ct.c_int(int(edge)))
    def mhSetInputChannelOffset(self, device, channel, offset): #channel timing offset in ps
        return mhlib.MH_SetInputChannelOffset(ct.c_int(device), ct.c_int(channel), ct.c_int(int(offset)))
    def mhSetSyncChannelOffset(self, device, offset): #sync timing offset in ps
        return mhlib.MH_SetSyncChannelOffset(ct.c_int(device),  ct.c_int(int(offset)))

    def mhClearHistMem(self,device): #This clears the histogram memory of all channels.
        return mhlib.MH_ClearHistMem(ct.c_int(device))

    def mhGetResolution(self, device):
        return mhlib.MH_GetResolution(ct.c_int(device), byref(self.resolution))

    def mhSetMeasControl(self, device, meascontrol=0, startedge=1, stopedge=1):
        return mhlib.MH_SetMeasControl(ct.c_int(device),ct.c_int(meascontrol,ct.c_int(startedge),ct.c_int(stopedge)))
### meascontrol: measurement control code
#0 = MEASCTRL_SINGLESHOT_CTC
#1 = MEASCTRL_C1_GATED
#2 = MEASCTRL_C1_START_CTC_STOP
#3 = MEASCTRL_C1_START_C2_STOP
#4 = MEASCTRL_WR_M2S
#5 = MEASCTRL_WR_S2M
#### startedge: edge selection code
#0 = falling
#1 = rising
### stopedge: edge selection code
#0 = falling
#1 = rising
#### return value:
#=0 success
#<0 error
#####This sets the measurement control mode and must be called before starting a measurement. The default after initialization
#(if this function is not called) is 0, i.e. software controlled acquisition time. The other modes 1..5 allow hardware triggered
#measurements through TTL signals at the control port or through White Rabbit. Note that this needs custom software. For a
#guideline please see the demo set for the C language.

    def mhSetTriggerOutput(self, device, period=0):
        return mhlib.MH_SetTriggerOutput(ct.c_int(device),ct.c_int(period))
#period is in untis of 100 ns.  Set period =0 to turn off

    def mhSetBinning(self, device, binning):
        return mhlib.MH_SetBinning(ct.c_int(device),ct.c_int(binning))
#The base time step for the Multiharp 150 is 80 ps.
#mhSetBinning sets the time step to 80*(2**binning) ps, ie:
# binning   time step (ps)
#0            80
#1           160
#2           320
#3           640
#4          1280, etc.

    def mhStartMeas(self, device, t_acquire):
        print('starting MultiHarp measurement')
        return mhlib.MH_StartMeas(ct.c_int(device),ct.c_int(int(t_acquire)))
    def mhStopMeas(self, device):
        print('stopping MultiHarp measurement')
        return mhlib.MH_StopMeas(ct.c_int(device))

    def mhGetHistogram(self, device, channel):
        return mhlib.MH_GetHistogram(ct.c_int(device), byref(self.counts[channel]), ct.c_int(channel))




    def program_manual(self, values):
        #print(values)
        for channel in range(self.num_ch):
            for port_num in {'Edge'}:#This also handles 'Level'
                hardware_name = '{} Ch {}'.format(port_num, channel)
                if channel != 0:
                    print('Setting %s edge for dev %r , Level = %r , Edge = %r' % (hardware_name, dev[0], int(values['Level Ch {}'.format(channel)]), int(values[hardware_name])))
                    self.mhSetInputEdgeTrg(device = dev[0], channel=channel-1, level=values['Level Ch {}'.format(channel)], edge=values[hardware_name] )
                    #mhlib.MH_SetInputEdgeTrg(ct.c_int(dev[0]), ct.c_int(channel-1), ct.c_int(int(values['Level Ch {}'.format(channel)])), ct.c_int(int(values[hardware_name])))
                else:
                    print('Setting sync edge for dev %r , Level = %r , Edge = %r' % (dev[0], int(values['Level Ch {}'.format(channel)]), int(values[hardware_name])))
                    self.mhSetSyncEdgeTrg(device = dev[0],  level=values['Level Ch {}'.format(channel)], edge=values[hardware_name] )

            for port_num in {'Offset'}:
                hardware_name = '{} Ch {}'.format(port_num, channel)
                if channel != 0:
                    print('Setting %s offset for dev %r , offset = %r' % (hardware_name, dev[0],  int(values[hardware_name])))
                    self.mhSetInputChannelOffset(device=dev[0], channel=channel-1, offset=values[hardware_name])
                else:
                    print('Setting sync offset for dev %r , offset = %r' % (dev[0], int(values[hardware_name])))
                    self.mhSetSyncChannelOffset(device=dev[0], offset=values[hardware_name])
        mhlib.MH_GetSyncRate(ct.c_int(dev[0]),byref(self.syncRate))
        mhlib.MH_GetCountRate(ct.c_int(dev[0]),ct.c_int(0),byref(self.countRate))
        print('Sync rate: %r   Count rate %r' %(self.syncRate.value, self.countRate.value))
        return values


    def transition_to_buffered(self, device_name, h5file, initial_values, fresh):
        self.h5_file = h5file
        new_values=initial_values
        with h5py.File(h5file,'r') as hdf5_file:
            group = hdf5_file['devices/'][device_name]
            h5_data = group.get('EDGES')
            if h5_data:
                #print(initial_values)
                #print(h5_data)
                for datum in h5_data:
                    new_values['Edge Ch {}'.format(datum['Ch'])] = datum['Edge']
            h5_data = group.get('LEVELS')
            if h5_data:
                #print(initial_values)
                #print(h5_data)
                for datum in h5_data:
                    new_values['Level Ch {}'.format(datum['Ch'])] = datum['Level']
            h5_data = group.get('OFFSETS')
            if h5_data:
                #print(initial_values)
                #print(h5_data)
                for datum in h5_data:
                    new_values['Offset Ch {}'.format(datum['Ch'])] = datum['Offset']
#First we'll program all of the things that are not handled by program_manual, then use program_manual to take care of the rest
            h5_data = group.get('TRIGGER OUT')
            if h5_data:
                for datum in h5_data:
                    print('Setting Trigger Out Period to %d' % datum['Period x100 ns'])
                    self.mhSetTriggerOutput(device=dev[0],period=datum['Period x100 ns'])
            h5_data = group.get('BINNING')
            if h5_data:
                for datum in h5_data:
                    print('Setting Binning to %d' % datum['Binning'])
                    self.mhSetBinning(device=dev[0],binning=datum['Binning'])
                    self.mhGetResolution(device=dev[0])
                    #print(self.resolution)
                    print("Resolution is %1.1lfps" % self.resolution.value)
            new_values=self.program_manual(new_values)
            self.mhClearHistMem(device=dev[0])
            self.mhStartMeas(device=dev[0],t_acquire=1000*self.t_acquire) #mhStartMeas assumes t_acquire is in ms

        return new_values


    def transition_to_manual(self):
        start_time = time.time()
        self.mhStopMeas(device=dev[0])
        with h5py.File(self.h5_file) as f:
            groupname = self.device_name
            histogram_group = f['devices'][groupname]
            #histogram_group=group.create_group('histogram')
            self.mhGetHistogram(device=dev[0],channel=0)
            counts_Ch1= self.counts[0]  #This only retreives histogram data on Ch 1 at the moment
            print('Max counts were %d' % max(self.counts[0]))
            histogram_group.create_dataset('Histogram Ch 1',data=counts_Ch1)
        return True


    def abort_buffered(self):
        return True

    def abort_transition_to_buffered(self):
        return True

    def shutdown(self):
        for i in range(0, MAXDEVNUM):
            mhlib.MH_CloseDevice(ct.c_int(i))
        exit(0)
        return
