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

from ophyd.pv_positioner import (
    PVPositionerComparator,
    PVPositionerIsClose,
)


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


class KBTweakerAxis(PVPositionerIsClose):
    setpoint = Cpt(EpicsSignal, "")
    readback = Cpt(EpicsSignalRO, "RBV")
    rtol = 0.01

    """
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
        """


class KBTweaker(Device):
    hor = Cpt(KBTweakerAxis, "-Motor-X}Mtr")
    ver = Cpt(KBTweakerAxis, "-Motor-Y}Mtr")


class MXAttenuator(PVPositionerComparator):
    setpoint = Cpt(EpicsSignal, "Trans-SP")
    readback = Cpt(EpicsSignalRO, "Trans-I")
    actuate = Cpt(EpicsSignal, "Cmd:Set-Cmd.PROC")
    atten1 = Cpt(
        EpicsMotor,
        "XF:17IDB-OP:AMX{Attn:BCU-Ax:1}Mtr",
        add_prefix="",
        doc="150 um Sn",
    )
    atten2 = Cpt(
        EpicsMotor,
        "XF:17IDB-OP:AMX{Attn:BCU-Ax:2}Mtr",
        add_prefix="",
        doc="50 um Al",
    )
    atten3 = Cpt(
        EpicsMotor,
        "XF:17IDB-OP:AMX{Attn:BCU-Ax:3}Mtr",
        add_prefix="",
        doc="6 um Al",
    )
    atten4 = Cpt(
        EpicsMotor,
        "XF:17IDB-OP:AMX{Attn:BCU-Ax:4}Mtr",
        add_prefix="",
        doc="20 um Sn",
    )
    rtol = 0.01
    atol = None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def done_comparator(self, readback: float, setpoint: float) -> bool:
        """
        Check if the readback is close to the setpoint value.
        Uses numpy.isclose to make the comparison. Tolerance values
        atol and rtol for numpy.isclose are taken from the attributes
        self.atol and self.rtol, which can be defined as class attributes
        or passed in as init parameters.
        If atol or rtol are omitted, the default values from numpy are
        used instead.
        """
        kwargs = {}
        if self.atol is not None:
            kwargs["atol"] = self.atol
        if self.rtol is not None:
            kwargs["rtol"] = self.rtol

        sigs = [self.atten1, self.atten2, self.atten3, self.atten4]

        return all(
            [
                np.isclose(
                    np.float64(sig.user_setpoint.get()),
                    np.float64(sig.user_readback.get()),
                    **kwargs
                )
                for sig in sigs
            ]
        )


mxatten = MXAttenuator(
    "XF:17IDB-OP:AMX{Attn:BCU}", name="mxatten", settle_time=5
)
kbt = KBTweaker("XF:17ID-ES:AMX{Best:2", name="kbt")
cam_hi_ba = SpecialProsilica("XF:17IDB-ES:AMX{Cam:7}", name="cam_hi")
