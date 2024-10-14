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


class Screen4Cam(StandardProsilica):
    cv1 = Cpt(CVPlugin, "CV1:")
    jpeg = Cpt(
        JPEGPluginWithFileStore,
        "JPEG1:",
        write_path_template=f"/nsls2/data/amx/shared/calibration/{op_cycle}/screen4",
    )

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.read_attrs = [
            "cv1",
            "jpeg",
            "cam"
        ]

        [
            _plugin.enable_on_stage()
            for _plugin in [
                self.cv1,
                self.proc1,
                self.trans1,
            ]
        ]

        self.cam.read_attrs = [
            "size",
            "min_x",
            "min_y",
            "max_size"
        ]

        self.cam.size.read_attrs = [
            "size_x",
            "size_y"
        ]

        self.cam.max_size.read_attrs = [
            "max_size_x",
            "max_size_y"
        ]

        self.cv1.outputs.read_attrs = [
            "output1",
            "output2",

        ]  # output1 is x pos of centroid, output2 is y position (AD conv.)

        self._update_stage_sigs()

    def _update_stage_sigs(self, *args, **kwargs):
        self.stage_sigs.clear()
        self.stage_sigs.update(
            [
                ("cam.acquire", 0),
                ("cam.image_mode", 1),
            ]
        )
        self.jpeg.stage_sigs.clear()  # must disable inherited defaults

        self.jpeg.write_path_template = f"/nsls2/data/amx/shared/calibration/{op_cycle}/screen4"
        self.stage_sigs.update(
            [
                ("cam.acquire_time", 0.25),
                ("cam.acquire_period", 1.0),
                (
                    "cam.num_images",
                    1,
                ),  # this reduces missed triggers, why?
                ("cam.trigger_mode", 0),
                ("jpeg.enable", 1),
                ("jpeg.nd_array_port", "TRANS1"),
                ("jpeg.file_write_mode", 0),
                ("jpeg.num_capture", 1),
                ("jpeg.auto_save", 1),
                ("cv1.enable", 1),
                ("cv1.nd_array_port", "TRANS1"),
                ("cv1.func_sets.func_set1", "None"),
                ("cv1.func_sets.func_set2", "Centroid Identification"),
                ("cv1.func_sets.func_set3", "None"),
                ("cv1.inputs.input1", 1),  # num. contours
                ("cv1.inputs.input2", 3),  # blur
                ("cv1.inputs.input3", 100),  # threshold value
                ("cv1.inputs.input4", 500000),  # upper size
                ("cv1.inputs.input5", 2000),  # min. size
                ("proc1.data_type_out", "UInt8"),
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


# beam align/flux
write_flux = EpicsSignal(
    "XF:17IDA-OP:AMX{Mono:DCM-dflux}Calc.PROC", name="write_flux")
smart_magnet = SmartMagnet("XF:17IDB-ES:AMX{Wago:1}", name="smart_magnet")
mxatten = MXAttenuator("XF:17IDB-OP:AMX{Attn:BCU}", name="mxatten")
kbt = KBTweaker("XF:17ID-ES:AMX{Best:2", name="kbt")

# screen 4
screen4_cam = Screen4Cam("XF:17IDB-BI:AMX{FS:4-Cam:1}", name="screen4")
screen4_insert = EpicsSignal(
    "XF:17IDB-BI:AMX{BPM:2-Ax:Y}Pos-Sts",
    write_pv="XF:17IDB-BI:AMX{BPM:2-Ax:Y}Cmd:In-Cmd",
    name="screen4_insert",
)
screen4_retract = EpicsSignal(
    "XF:17IDB-BI:AMX{BPM:2-Ax:Y}Cmd:Out-Cmd",
    name="screen4_retract"
)
