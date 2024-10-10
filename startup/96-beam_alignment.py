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
    delta_px = Cpt(Signal, value=0, doc="distance to ROI center in pixels")
    aligned = Cpt(
        Signal,
        value=False,
        doc="bool representing whether or not axis has been brought to center"
    )
    rtol = 0.01
    limits = (-2.5, 2.5)


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


class SmartMagnet(Device):
    sample_detect = Cpt(
        EpicsSignalRO,
        "SampleDetected1-Sts",
        doc="1 is NO sample, 0 is YES sample",
    )


write_flux = EpicsSignal(
    "XF:17IDA-OP:AMX{Mono:DCM-dflux}Calc.PROC", name="write_flux")

smart_magnet = SmartMagnet("XF:17IDB-ES:AMX{Wago:1}", name="smart_magnet")
mxatten = MXAttenuator("XF:17IDB-OP:AMX{Attn:BCU}", name="mxatten")
kbt = KBTweaker("XF:17ID-ES:AMX{Best:2", name="kbt")
