from ophyd import (
    SingleTrigger,
    TIFFPlugin,
    ProsilicaDetector,
    ImagePlugin,
    StatsPlugin,
    ROIPlugin,
    DetectorBase,
    HDF5Plugin,
    TransformPlugin,
    ProcessPlugin,
    AreaDetector,
    EpicsSignalRO,
    EpicsSignalWithRBV,
    ColorConvPlugin,
    Device,
)

from ophyd import Device

import ophyd.areadetector.cam as cam

from ophyd.areadetector.filestore_mixins import (
    FileStoreTIFFIterativeWrite,
    FileStoreHDF5IterativeWrite,
)

from ophyd import Component as Cpt
from ophyd.status import SubscriptionStatus

import time


class SpecialProsilica(ProsilicaDetector):
    cc1 = Cpt(ColorConvPlugin, "CC1:")
    roi1 = Cpt(ROIPlugin, "ROI1:")
    roi4 = Cpt(ROIPlugin, "ROI4:")
    trans1 = Cpt(TransformPlugin, "Trans1:")
    proc1 = Cpt(ProcessPlugin, "Proc1:")
    stats4 = Cpt(StatsPlugin, "Stats4:")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cc1.enable_on_stage()
        self.proc1.enable_on_stage()
        self.read_attrs = ["stats4"]
        self.stats4.read_attrs = ["total", "net", "centroid"]
        self.stats4.centroid.read_attrs = ["x", "y"]
        self.stage_sigs.update(
            [
                ("cc1.nd_array_port", "CC1"),
                ("roi4.min_xyz.min_x", 818),
                ("roi4.min_xyz.min_y", 400),
            ]
        )


class KBTweakerAxis(Device):
    voltage = Cpt(EpicsSignal, "", kind="hinted")
    step_size = Cpt(EpicsSignal, ".INPA", kind="config")
    actuate = Cpt(EpicsSignal, ".B", kind="omitted")

    def __init__(self, *args, voltage_limit=2, **kwargs):
        super().__init__(*args, **kwargs)
        self.voltage_limit = voltage_limit
        self._status = None

    def set(self, target):
        if self._status is not None and not self._status.done:
            raise RuntimeError("Another set is in progress")

        if abs(target) > self.voltage_limit:
            raise ValueError(
                f"You passed {target!r} with out of ±{self.voltage_limit} gamut."
            )

        # get the step size
        step_size = float(self.step_size.get())
        # get the current voltage
        current = self.voltage.get()
        # compute number of "steps" to take
        num_steps = int(np.round(abs(target - current) / step_size))
        # step direction
        try:
            step_direction = (target - current) / abs(target - current)
        except ZeroDivisionError:
            step_direction = 0

        def deadband(*, old_value, value, **kwargs):
            # if with in half a step size, declare victory
            if abs(value - target) < (step_size / 2):
                return True
            return False

        self._status = status = SubscriptionStatus(self.voltage, deadband)
        if not status.done:
            for k in range(0, num_steps):
                self.actuate.put(step_direction)
                time.sleep(1)
        return status


class KBTweaker(Device):
    hor = Cpt(KBTweakerAxis, ":TwkCh1")
    ver = Cpt(KBTweakerAxis, ":TwkCh2")


kbt = KBTweaker("XF:17IDB-BI:AMX{Best:2}", name="kbt")

cam_hi_ba = SpecialProsilica("XF:17IDB-ES:AMX{Cam:7}", name="cam_hi")
