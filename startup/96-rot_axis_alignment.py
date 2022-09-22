print(f"Loading {__file__}")

from ophyd import PseudoPositioner, PseudoSingle, Staged
from ophyd.pseudopos import pseudo_position_argument, real_position_argument
from ophyd import (
    SingleTrigger,
    ProsilicaDetector,
    ColorConvPlugin,
    Device,
)
from ophyd import Component as Cpt
from ophyd.areadetector.plugins import PluginBase, register_plugin
from ophyd.areadetector.base import DDC_EpicsSignal, DDC_EpicsSignalRO
from ophyd.areadetector.base import ADComponent as ADCpt


class GonioCameraPositioner(PseudoPositioner):
    """move gonio fine stage with camera basis for any omega

    omega=0 degrees:
        pinY: vertical, parallel with on-axis camera image
        pinZ: downstream, perpendicular to on-axis camera image

    omega=90 degrees
        pinY: upstream, perpendicular to on-axis camera image
        pinZ: vertical, parallel with on-axis camera image

    2D rotation matrix:
        [cos(w) -sin(w); sin(w) cos(w)]

    inverse rotation:
        [cos(w) sin(w); -sin(w) cos(w)]

    """

    # pseudo axes
    cam_y = Cpt(PseudoSingle, limits=(-1000, 1000))
    cam_z = Cpt(PseudoSingle, limits=(-1000, 1000))

    # real axes
    real_y = Cpt(EpicsMotor, "-Ax:PY}Mtr")
    real_z = Cpt(EpicsMotor, "-Ax:PZ}Mtr")

    # configuration value that determine change of basis matrix
    omega = Cpt(EpicsSignalRO, "-Ax:O}Mtr.RBV")

    def __init__(self, *args, concurrent=False, **kwargs):
        """PinY, PinZ smaract stages can jam up if moved simultaneously,
        therefore move in series"""
        super().__init__(*args, **kwargs)
        self.omega.subscribe(self.update_parameters)

    def update_parameters(self, *args, **kwargs):
        [_.sync() for _ in self._pseudo]

    def unstage(self):

        self.log.debug("Unstaging %s", self.name)
        self._staged = Staged.partially
        devices_unstaged = []

        # Call unstage() on child devices.
        for attr in self._sub_devices[::-1]:
            device = getattr(self, attr)
            if hasattr(device, "unstage"):
                device.unstage()
                devices_unstaged.append(device)

        # Restore original values.
        for sig, val in reversed(list(self._original_vals.items())):
            self.log.debug(
                "Setting %s back to its original value: %r", sig.name, val
            )
            sig.set(val, settle_time=0.1).wait()
            self._original_vals.pop(sig)
        devices_unstaged.append(self)

        self._staged = Staged.no
        return devices_unstaged

    @pseudo_position_argument
    def forward(self, pos):
        """pseudo -> real, motor I/O in degrees"""
        d = np.pi / 180
        omega = self.omega.get()
        return self.RealPosition(
            real_y=(
                pos.cam_y * np.cos(omega * d) - pos.cam_z * np.sin(omega * d)
            ),
            real_z=(
                pos.cam_y * np.sin(omega * d) + pos.cam_z * np.cos(omega * d)
            ),
        )

    @real_position_argument
    def inverse(self, pos):
        """real -> pseudo, motor I/O in degrees"""
        d = np.pi / 180
        omega = self.omega.get()
        return self.PseudoPosition(
            cam_y=(pos.real_y) * np.cos(omega * d)
            + (pos.real_z) * np.sin(omega * d),
            cam_z=-(pos.real_y) * np.sin(omega * d)
            + (pos.real_z) * np.cos(omega * d),
        )


@register_plugin
class CVPlugin(PluginBase):
    _default_suffix = "CV1:"
    _suffix_re = "CV1\d:"
    _default_read_attrs = ["outputs"]
    func_sets = DDC_EpicsSignal(
        *[(f"func_set{k}", f"CompVisionFunction{k}") for k in range(1, 4)]
    )
    inputs = DDC_EpicsSignal(
        *[(f"input{k}", f"Input{k}") for k in range(1, 11)]
    )
    outputs = DDC_EpicsSignalRO(
        *[(f"output{k}", f"Output{k}_RBV") for k in range(1, 11)]
    )
    cam_depth = ADCpt(EpicsSignal, "CompVisionCamDepth", kind="config")


class SingleTriggerProsilica(SingleTrigger, ProsilicaDetector):
    pass


class RotAlignLowMag(StandardProsilica):
    # cc1 = Cpt(ColorConvPlugin, "CC1:")
    pix_per_um = Cpt(Signal, value=1, kind="config")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.read_attrs = ["stats1", "stats2"]
        self.stats1.read_attrs = ["total"]
        self.stats2.read_attrs = ["total"]
        self.stage_sigs.cam = {
            "acquire_time": 0.0025,
            "acquire_period": 1,
        }


class RotAlignHighMag(StandardProsilica):
    cc1 = Cpt(ColorConvPlugin, "CC1:")
    cv1 = Cpt(CVPlugin, "CV1:")
    cam_mode = Cpt(Signal, value=None, kind="config")
    pix_per_um = Cpt(Signal, value=3.4, kind="config")
    roi_offset = Cpt(Signal, value=256, kind="config")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.read_attrs = [
            "cc1",
            "cv1",
            "roi1",
            "roi2",
            "roi3",
            "roi4",
            "stats4",
            "trans1",
        ]

        [
            _plugin.enable_on_stage()
            for _plugin in [
                self.cv1,
                self.roi4,
                self.stats1,
                self.stats4,
                self.proc1,
                self.cc1,
            ]
        ]
        self.cv1.outputs.read_attrs = [
            "output7",
            "output8",
        ]  # 7 left, 8 right pixel
        self.stats4.read_attrs = ["centroid", "max_xy"]
        self.stats4.centroid.read_attrs = ["x", "y"]
        self.stats4.max_xy.read_attrs = ["x", "y"]
        self.cam_mode.subscribe(self._update_stage_sigs, event_type="value")
        self._update_stage_sigs()
        self.roi1.min_xyz.min_y.subscribe(self._sync_rois, event_type="value")

    def _update_stage_sigs(self, *args, **kwargs):
        self.stage_sigs.clear()
        self.stage_sigs.update(
            [
                ("cam.acquire", 0),
                ("cam.image_mode", 1),
            ]
        )
        if self.cam_mode.get() == "edge_detection":
            self.stage_sigs.update(
                [
                    ("cam.acquire_time", 0.15),
                    ("cam.acquire_period", 0.15),
                    ("cv1.nd_array_port", "ROI4"),
                    ("cv1.func_sets.func_set1", "Canny Edge Detection"),
                    ("cv1.inputs.input1", 35),
                    ("cv1.inputs.input2", 5),
                    ("cv1.inputs.input3", 13),
                    ("cv1.inputs.input4", 5),
                    ("roi4.min_xyz.min_x", 672),
                    ("roi4.min_xyz.min_y", 0),
                    ("roi4.size.x", 898),
                    ("roi4.size.y", 1246),
                ]
            )

        elif self.cam_mode.get() == "rot_align":
            self.stage_sigs.update(
                [
                    ("cam.acquire_time", 0.15),
                    ("cam.acquire_period", 0.15),
                    ("proc1.nd_array_port", "CC1"),
                    ("proc1.enable_filter", 1),
                    ("proc1.filter_type", "CopyToFilter"),
                    ("proc1.o_scale", -1),
                    ("proc1.o_offset", 140),
                    ("roi4.min_xyz.min_x", 1117),
                    ("roi4.min_xyz.min_y", 0),
                    ("roi4.size.x", 25),
                    ("roi4.size.y", 1246),
                ]
            )

    def _sync_rois(self, *args, **kwargs):
        self.roi2.min_xyz.min_y.put(
            self.roi1.min_xyz.min_y.get() - self.roi_offset.get()
        )


class RotationAxisAligner(Device):
    cam_hi = Cpt(RotAlignHighMag, "{Cam:7}")
    cam_lo = Cpt(RotAlignLowMag, "{Cam:6}")
    gc_positioner = Cpt(GonioCameraPositioner, "{Gon:1")
    current_rot_axis = Cpt(
        Signal,
        value=None,
        doc="current rotation axis in pixels based on high mag ROI",
    )

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.current_rot_axis.put(self.cam_hi.roi1.min_xyz.min_y.get() + 256)
        self.current_rot_axis.subscribe(
            self._update_roi, event_type="value", run=False
        )

    def _update_roi(self, *args, **kwargs):
        self.cam_hi.roi1.min_xyz.min_y.put(
            round(self.current_rot_axis.get()) - 256
        )


rot_aligner = RotationAxisAligner("XF:17IDB-ES:AMX", name="rot_aligner")
