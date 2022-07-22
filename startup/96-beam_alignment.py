from ophyd import (SingleTrigger, TIFFPlugin, ProsilicaDetector,
                   ImagePlugin, StatsPlugin, ROIPlugin, DetectorBase, HDF5Plugin,
                   TransformPlugin, ProcessPlugin, AreaDetector, EpicsSignalRO, EpicsSignalWithRBV,
                   ColorConvPlugin, Device)

from ophyd import Device

import ophyd.areadetector.cam as cam

from ophyd.areadetector.filestore_mixins import (FileStoreTIFFIterativeWrite,
                                                 FileStoreHDF5IterativeWrite)

from ophyd import Component as Cpt
from ophyd.status import SubscriptionStatus


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

class KBTweakerAxis(Device):
    voltage = Cpt(EpicsSignal, "", kind="hinted")
    step_size = Cpt(EpicsSignal, ".INPA", kind="config")
    actuate = Cpt(EpicsSignal, ".B", kind="omitted")

    def __init__(self, *args, voltage_limit=2, **kwargs):
        super().__init__(*args, **kwargs)
        # do not mutate the hardware in device init, it makes it
        # makes profile start up have side-effects which might mess up
        # running experiments
        #  self.step_size.put('0.05')

        self.voltage_limit = voltage_limit
        self._status = None

    def set(self, target):
        if self._status is not None and not self._status.done:
            raise RuntimeError("Another set is in progress")

        if -self.voltage_limit < target < self.voltage_limit:
            raise ValueError(
                f"You passed {target!r} with out of ±{self.voltage_limit} gamut."
            )
        # get the step size
        step_size = self.step_size.get()
        # get the current voltage
        current = self.voltage.get()
        # compute (float) number of "steps" to take
        num_steps = (target - current) / step_size

        def deadband(*, old_value, value, **kwargs):
            # if with in half a step size, declare victory
            if abs(value - target) < (step_size / 2):
                return True
            return False

        self._status = status = SubscriptionStatus(self, deadband)
        if not status.done:
            self.actuate.put(num_steps)
        return status

    def stop(self, *, success=False):
        # TODO do something to actually stop the voltage moving it that is
        # possible?
        super().stop(success)
        if self._status is not None and not self._status.done:
            self._status.set_exception(Exception("Motion stopped"))
            self._status = None


class KBTweaker(Device):
    hor = Cpt(KBTweakerAxis, ":TwkCh1")
    ver = Cpt(KBTweakerAxis, ":TwkCh2")

kbt = KBTweaker("XF:17IDB-BI:AMX{Best:2}", name="kbt")

cam_hi = SpecialProsilica('XF:17IDB-ES:AMX{Cam:7}', name='cam_hi')
