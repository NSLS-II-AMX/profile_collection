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
from ophyd.signal import DerivedSignal


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


class RotAlignLowMag(StandardProsilica):
    cv1 = Cpt(CVPlugin, "CV1:")
    cam_mode = Cpt(Signal, value=None, kind="config")
    pix_per_um = Cpt(Signal, value=1, kind="config")
    roi_offset = Cpt(Signal, value=256, kind="config")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.read_attrs = ["cv1", "stats1", "stats2"]
        self.cv1.read_attrs = ["outputs"]
        self.cv1.outputs.read_attrs = ["output1", "output2", "output3"]
        self.stats1.read_attrs = ["total"]
        self.stats2.read_attrs = ["total"]
        self.cam_mode.subscribe(self._update_stage_sigs, event_type="value")
        self.roi1.min_xyz.min_y.subscribe(self._sync_rois, event_type="value")

    def _update_stage_sigs(self, *args, **kwargs):
        self.stage_sigs.clear()
        self.stage_sigs.update(
            [
                ("cam.acquire", 0),
                ("cam.image_mode", 1),
                ("cam.acquire_time", 0.0025),
                ("cam.acquire_period", 1),
            ]
        )
        if self.cam_mode.get() == "centroid":
            self.stage_sigs.update(
                [
                    ("cv1.enable", 1),
                    ("cv1.func_sets.func_set2", "Centroid Identification"),
                    ("cv1.inputs.input1", 1),
                    ("cv1.inputs.input2", 5),
                    ("cv1.inputs.input3", 30),
                    ("cv1.inputs.input4", 3000000),
                    ("cv1.inputs.input5", 5000),
                ]
            )
        elif self.cam_mode.get() == "edge_detection":
            self.stage_sigs.update(
                [
                    ("cv1.enable", 1),
                    ("cv1.nd_array_port", "ROI4"),
                    ("cv1.func_sets.func_set1", "Canny Edge Detection"),
                    ("cv1.inputs.input1", 20),
                    ("cv1.inputs.input2", 8),
                    ("cv1.inputs.input3", 9),
                    ("cv1.inputs.input4", 5),
                    ("roi4.min_xyz.min_y", self.roi1.min_xyz.min_y.get()),
                    (
                        "roi4.min_xyz.min_x",
                        self.roi1.min_xyz.min_x.get() + 245,
                    ),
                    ("roi4.size.x", 240),
                    ("roi4.size.y", self.roi1.size.y.get()),
                ]
            )
        elif self.cam_mode.get() == "find_sheath":
            pass

    def _sync_rois(self, *args, **kwargs):
        self.roi2.min_xyz.min_y.put(
            self.roi1.min_xyz.min_y.get() - self.roi_offset.get()
        )

    def stage(self, *args, **kwargs):
        self._update_stage_sigs(*args, **kwargs)
        super().stage(*args, **kwargs)


class RotAlignHighMag(StandardProsilica):
    cc1 = Cpt(ColorConvPlugin, "CC1:")
    cv1 = Cpt(CVPlugin, "CV1:")
    cam_mode = Cpt(Signal, value=None, kind="config")
    pix_per_um = Cpt(Signal, value=3.4, kind="config")
    roi_offset = Cpt(Signal, value=256, kind="config")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.read_attrs = [
            "cv1",
            "roi4",
            "stats4",
        ]

        [
            _plugin.enable_on_stage()
            for _plugin in [
                self.cv1,
                self.roi4,
                self.stats4,
                self.proc1,
                self.cc1,
                self.trans1,
            ]
        ]

        self.cv1.outputs.read_attrs = [
            "output1",
            "output2",
            "output3",
            "output7",
            "output8",
        ]  # 3 vert center, 7 left pixel, 8 right pixel
        self.stats4.read_attrs = ["centroid", "max_xy"]
        self.stats4.centroid.read_attrs = ["x", "y"]
        self.stats4.max_xy.read_attrs = ["x", "y"]
        self._update_stage_sigs()
        self.roi1.min_xyz.min_y.subscribe(self._sync_rois, event_type="value")

    @property
    def trigger_signals(self):
        return [self.cam_mode]

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
                    ("cv1.enable", 1),
                    ("cv1.nd_array_port", "ROI4"),
                    ("cv1.func_sets.func_set1", "Canny Edge Detection"),
                    ("cv1.func_sets.func_set2", "None"),
                    ("cv1.func_sets.func_set3", "None"),
                    ("cv1.inputs.input1", 35),
                    ("cv1.inputs.input2", 5),
                    ("cv1.inputs.input3", 13),
                    ("cv1.inputs.input4", 5),
                    ("roi4.min_xyz.min_x", 672),
                    ("roi4.min_xyz.min_y", 0),
                    ("roi4.size.x", 1264),
                    ("roi4.size.y", 1246),
                ]
            )

        elif self.cam_mode.get() == "rot_align":
            self.stage_sigs.update(
                [
                    ("cam.acquire_time", 0.15),
                    ("cam.acquire_period", 0.15),
                    ("cv1.enable", 0),
                    ("cv1.nd_array_port", "CAM"),
                    ("proc1.nd_array_port", "CC1"),
                    ("proc1.enable_filter", 1),
                    ("proc1.filter_type", "CopyToFilter"),
                    ("proc1.o_scale", -1),
                    ("proc1.o_offset", 140),
                    (
                        "roi4.min_xyz.min_x",
                        self.roi1.min_xyz.min_x.get() + 302,
                    ),
                    ("roi4.min_xyz.min_y", 0),
                    ("roi4.size.x", 25),
                    ("roi4.size.y", 1246),
                ]
            )

            self.stats4.stage_sigs.clear()
            self.stats4.stage_sigs.update(
                [
                    ("enable", 1),
                    ("blocking_callbacks", "Yes"),
                ]
            )

        elif self.cam_mode.get() == "rot_align_contour":
            self.stage_sigs.update(
                [
                    ("cam.acquire_time", 0.15),
                    ("cam.acquire_period", 0.15),
                    (
                        "cam.num_images",
                        2,
                    ),  # this reduces missed triggers, why?
                    ("cam.trigger_mode", 5),
                    ("cc1.enable", 1),
                    ("cc1.nd_array_port", "CAM"),
                    ("proc1.nd_array_port", "CC1"),
                    ("proc1.enable", 0),
                    ("cv1.enable", 1),
                    ("cv1.nd_array_port", "ROI2"),
                    ("cv1.func_sets.func_set1", "None"),
                    ("cv1.func_sets.func_set2", "None"),
                    ("cv1.func_sets.func_set3", "User Function"),
                    ("cv1.inputs.input1", 33),
                    ("cv1.inputs.input2", 8),
                    ("cv1.inputs.input3", 7),
                    ("cv1.inputs.input4", 5),
                ]
            )

            self._disable_stats_plugins()

        elif self.cam_mode.get() == "beam_align":
            self.stage_sigs.update(
                [
                    ("cam.acquire_time", 0.15),
                    ("cam.acquire_period", 0.15),
                    ("cv1.enable", 1),
                    ("cv1.nd_array_port", "ROI1"),
                    ("cv1.func_sets.func_set1", "None"),
                    ("cv1.func_sets.func_set2", "Centroid Identification"),
                    ("cv1.func_sets.func_set3", "None"),
                    ("cv1.inputs.input1", 1),  # num. contours
                    ("cv1.inputs.input2", 5),  # blur
                    ("cv1.inputs.input3", 50),  # threshold value
                    ("cv1.inputs.input4", 40000),  # upper size
                    ("cv1.inputs.input5", 1000),  # min. size
                    ("cc1.enable", 1),
                    ("proc1.nd_array_port", "CC1"),
                ]
            )

            self._disable_stats_plugins()

    def _disable_stats_plugins(self):
        # disable stats plugins, reduce ioc load, avoid missing triggers
        stats_plugins = [
            self.stats1,
            self.stats2,
            self.stats3,
            self.stats4,
            self.stats5,
        ]

        [_plugin.stage_sigs.clear() for _plugin in stats_plugins]

        [
            _plugin.stage_sigs.update(
                [
                    ("enable", 0),
                    ("blocking_callbacks", "No"),
                ]
            )
            for _plugin in stats_plugins
        ]

    def _sync_rois(self, *args, **kwargs):
        self.roi2.min_xyz.min_y.put(
            self.roi1.min_xyz.min_y.get() - self.roi_offset.get()
        )

    def stage(self, *args, **kwargs):
        self._update_stage_sigs(*args, **kwargs)
        super().stage(*args, **kwargs)


class RotationAxisAligner(Device):
    cam_hi = Cpt(RotAlignHighMag, "{Cam:7}")
    cam_lo = Cpt(RotAlignLowMag, "{Cam:6}")
    gc_positioner = Cpt(GonioCameraPositioner, "{Gon:1")
    current_rot_axis = Cpt(
        Signal,
        value=None,
        doc="current rotation axis in hi mag pixels based on high mag ROI",
    )
    proposed_rot_axis = Cpt(
        Signal, value=None, doc="proposed rot axis in pixels"
    )
    acceptance_criterium = Cpt(
        Signal,
        value=20,
        doc="difference in pixels we will accept",
        kind="config",
    )

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.current_rot_axis.put(self.cam_hi.roi1.min_xyz.min_y.get() + 256)
        self.proposed_rot_axis.subscribe(
            self._update_rot_axis, event_type="value", run=False
        )

    def _update_rois(self, delta_pix, **kwargs):
        self.cam_hi.roi1.min_xyz.min_y.put(self.current_rot_axis.get() - 256)
        self.cam_lo.roi1.min_xyz.min_y.put(
            delta_pix
            * (self.cam_lo.pix_per_um.get() / self.cam_hi.pix_per_um.get())
            + self.cam_lo.roi1.min_xyz.min_y.get()
        )

    def _update_rot_axis(self, *args, **kwargs):
        delta_pix = round(
            self.current_rot_axis.get() - self.proposed_rot_axis.get()
        )
        if abs(delta_pix) < self.acceptance_criterium.get():
            self.current_rot_axis.put(self.proposed_rot_axis.get())
            self._update_rois(delta_pix)


rot_aligner = RotationAxisAligner("XF:17IDB-ES:AMX", name="rot_aligner")
cam_hi_ba = RotAlignHighMag("XF:17IDB-ES:AMX{Cam:7}", name="cam_hi")
