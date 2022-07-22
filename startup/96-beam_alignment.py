from ophyd import (SingleTrigger, TIFFPlugin, ProsilicaDetector,
                   ImagePlugin, StatsPlugin, ROIPlugin, DetectorBase, HDF5Plugin,
                   TransformPlugin, ProcessPlugin, AreaDetector, EpicsSignalRO, EpicsSignalWithRBV,
                   ColorConvPlugin, Device)

from ophyd import Device

import ophyd.areadetector.cam as cam

from ophyd.areadetector.filestore_mixins import (FileStoreTIFFIterativeWrite,
                                                 FileStoreHDF5IterativeWrite)

from ophyd import Component as Cpt

class SpecialProsilica(ProsilicaDetector):
    cc1 = Cpt(ColorConvPlugin, 'CC1:')
    roi1 = Cpt(ROIPlugin, 'ROI1:')
    roi4 = Cpt(ROIPlugin, 'ROI4:')
    trans1 = Cpt(TransformPlugin, 'Trans1:')
    proc1 = Cpt(ProcessPlugin, 'Proc1:')
    stats4 = Cpt(StatsPlugin, 'Stats4:')
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.read_attrs = ['stats4',
                           'trans1',
                           'cc1',
                           'proc1',
                           'roi1',
                           'roi4']
        self.cc1.enable_on_stage()
        self.proc1.enable_on_stage()
        self.stats4.read_attrs = ['total',
                                  'net']
        self.stats4.centroid.read_attrs = ['x','y']
    
    def stage(self):
        #settings for smoother centroid fitting
        self.proc1.enable_filter.put('1')
        self.proc1.filter_type.put('RecursiveAve')
        self.proc1.num_filter.put('5')
        #color conversion to monochromatic for stats plugin
        self.cc1.nd_array_port.put('PROC1')
        self.trans1.nd_array_port.put('CC1')
        #ROI4 must match ROI1, which defines center of rotation
        self.roi4.min_xyz.min_x.put(f'{self.roi1.min_xyz.min_x.get()}')
        self.roi4.min_xyz.min_y.put(f'{self.roi1.min_xyz.min_y.get()}')
        super().stage()
        
    def unstage(self):
        #process plugin
        self.proc1.num_filter.put('1')
        self.proc1.enable_filter.put('0')
        #put ports back for LSDC
        self.trans1.nd_array_port.put('CAM')
        self.cc1.nd_array_port.put('CAM')
        super().unstage()
                
class KBTweaker(Device):
    
    hor_voltage = Cpt(EpicsSignal, ':TwkCh1')
    ver_voltage = Cpt(EpicsSignal, ':TwkCh2')
    hor_voltage_step_size = Cpt(EpicsSignal, ':TwkCh1.INPA')
    ver_voltage_step_size = Cpt(EpicsSignal, ':TwkCh2.INPA')
    hor_step_actuate = Cpt(EpicsSignal, ':TwkCh1.B')
    ver_step_actuate = Cpt(EpicsSignal, ':TwkCh2.B')
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.hor_voltage_step_size.put('0.05')
        self.ver_voltage_step_size.put('0.05')
        self.voltage_limit = 2
        
    def voltage_check(func):
        def check(self, axis, direction):
            if ((-2 < self.hor_voltage.read()['kbt_hor_voltage']['value'] < 2)
                    and (-2 < self.ver_voltage.read()['kbt_ver_voltage']['value'] < 2)):
                func(self, axis, direction)
                print('voltage tweak attempted')
            else:
                print('voltage out of ranage, check alignment')
        return check
    
    def set(self):
        pass
    
    @voltage_check
    def tweak_voltage(self, axis, direction):
        if axis in ('h','v') and direction in ('-1','1'):
            if axis == 'h': #-1 is outboard, 1 is inboard
                self.hor_step_actuate.put(direction)
            if axis == 'v': #-1 is up, 1 is down, camera coords
                self.ver_step_actuate.put(direction)
        else:
            print('wrong axis or direction type')
    
kbt = KBTweaker('XF:17IDB-BI:AMX{Best:2}', name='kbt')      
cam_hi = SpecialProsilica('XF:17IDB-ES:AMX{Cam:7}', name='cam_hi')
