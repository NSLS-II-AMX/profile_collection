#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov  2 11:51:23 2022

@author: dkreitler
"""
from ophyd.areadetector.filestore_mixins import FileStoreTIFF
from ophyd import Kind


class TIFFPluginWithFileStore(TIFFPlugin, FileStoreTIFF):
    pass


class TopAlignCam(StandardProsilica):
    _default_read_attrs = ["cv1", "tiff"]
    cv1 = Cpt(CVPlugin, "CV1:")
    tiff = Cpt(
        TIFFPluginWithFileStore,
        "TIFF1:",
        write_path_template="/nsls2/data/amx/legacy/topcam",
    )
    cam_mode = Cpt(Signal, value=None, kind="config")
    pix_per_um = Cpt(Signal, value=0.164, doc="pixels per um")
    out10_buffer = Cpt(
        EpicsSignalRO,
        "Out10:compress",
        doc="circular buffer for ADCompVision output10 (thresholding loop profile)",
    )
    out10_reset = Cpt(EpicsSignal, "Out10:compress.RES")
    out9_buffer = Cpt(EpicsSignalRO, "Out9:compress")
    out9_reset = Cpt(EpicsSignal, "Out9:compress.RES")

    def __init__(self, *args, **kwargs):
        super().__init__(
            *args,
            **kwargs,
        )
        #self.tiff.read_attrs = []
        self.cv1.read_attrs = ["outputs"]
        self.cv1.outputs.read_attrs = [
            "output8",
            "output9",
            "output10",
        ]
        self.cam_mode.subscribe(self._update_stage_sigs, event_type="value")

    def _update_stage_sigs(self, *args, **kwargs):
        self.tiff.stage_sigs.update([("enable", 0)])
        self.stage_sigs.clear()
        self.stage_sigs.update(
            [
                ("cv1.func_sets.func_set1", "Canny Edge Detection"),
                ("cv1.inputs.input1", 8),
                ("cv1.inputs.input2", 3),
                ("cv1.inputs.input3", 1),
                ("cv1.inputs.input4", 3),
                ("cv1.inputs.input5", 13),
                ("cv1.inputs.input6", 1),
                ("trans1.nd_array_port", "PROC1"),
                ("tiff.nd_array_port", "CV1"),
            ]
        )
        if self.cam_mode.get() == "coarse_align":
            self.stage_sigs.update(
                [
                    ("cv1.nd_array_port", "ROI2"),
                    ("roi2.nd_array_port", "TRANS1"),
                ]
            )
        elif self.cam_mode.get() == "fine_face":
            self.stage_sigs.update(
                [
                    ("cv1.nd_array_port", "ROI1"),
                    ("roi1.nd_array_port", "TRANS1"),
                ]
            )

    def stage(self, *args, **kwargs):
        #self._update_stage_sigs(*args, **kwargs)
        super().stage(*args, **kwargs)

    def trigger(self):
        try:
            status = super().trigger()
            status.wait(6)
        except AttributeError:
            raise FailedStatus
        return status


class ZebraMXOr(Zebra):
    or3 = Cpt(EpicsSignal, "OR3_ENA:B3")
    or3loc = Cpt(EpicsSignal, "OR3_INP4")
    armsel = Cpt(EpicsSignal, "PC_ARM_SEL")


class GovernorError(Exception):
    def __init__(self, message):
        super().__init__(self, message)


class TopAlignerBase(Device):

    topcam = Cpt(TopAlignCam, "XF:17IDB-ES:AMX{Cam:9}")
    gonio_o = Cpt(EpicsMotor, "XF:17IDB-ES:AMX{Gon:1-Ax:O}Mtr", timeout=6)
    gonio_py = Cpt(
        EpicsMotorSPMG, "XF:17IDB-ES:AMX{Gon:1-Ax:PY}Mtr", timeout=6
    )
    gonio_pz = Cpt(
        EpicsMotorSPMG, "XF:17IDB-ES:AMX{Gon:1-Ax:PZ}Mtr", timeout=6
    )
    kill_py = Cpt(EpicsSignal, "XF:17IDB-ES:AMX{Gon:1-Ax:PY}Cmd:Kill-Cmd")
    kill_pz = Cpt(EpicsSignal, "XF:17IDB-ES:AMX{Gon:1-Ax:PY}Cmd:Kill-Cmd")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.read_attrs = []
        self._configure_device()

    def _configure_device(self, *args, **kwargs):
        raise NotImplementedError(
            "Subclasses must implement custom device configuration"
        )

    def stage(self, *args, **kwargs):
        if type(self) == TopAlignerBase:
            raise NotImplementedError("TopAlignerBase has no stage method")
        return super().stage(*args, **kwargs)

    def trigger(self):
        raise NotImplementedError("Subclasses must implement custom trigger")

    def unstage(self, *args, **kwargs):
        if type(self) == TopAlignerBase:
            raise NotImplementedError("TopAlignerBase has no unstage method")
        super().unstage(*args, **kwargs)


class TopAlignerFast(TopAlignerBase):

    zebra = Cpt(ZebraMXOr, "XF:17IDB-ES:AMX{Zeb:2}:")
    target_gov_state = Cpt(
        Signal, value=None, doc="target governor state used to trigger device"
    )

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.target_gov_state.subscribe(
            self._update_stage_sigs, event_type="value"
        )

    def _configure_device(self, *args, **kwargs):
        self.read_attrs = ["topcam", "zebra"]
        self.stage_sigs.clear()
        self.stage_sigs.update(
            [
                ("topcam.cam.trigger_mode", 1),
                ("topcam.cam.image_mode", 2),
                ("topcam.cam.acquire", 1),
                ("gonio_o.velocity", 90),  # slow down to get picture taken
                ("zebra.pos_capt.source", "Enc4"),
                ("zebra.armsel", 0),
                ("zebra.pos_capt.gate.start", 0.5),  # very important
                ("zebra.pos_capt.gate.width", 4.5),
                ("zebra.pos_capt.gate.step", 9),
                ("zebra.pos_capt.gate.num_gates", 20),
                ("zebra.pos_capt.pulse.start", 0),
                ("zebra.pos_capt.pulse.width", 4),
                ("zebra.pos_capt.pulse.step", 5),
                ("zebra.pos_capt.pulse.delay", 0),
                ("zebra.pos_capt.pulse.max_pulses", 1),
                ("zebra.or3", 1),
                ("zebra.or3loc", 30),

            ]
        )

        self.topcam.read_attrs = ["out9_buffer", "out10_buffer"]
        self.zebra.read_attrs = ["pos_capt.data.enc4"]

    def _update_stage_sigs(self, *args, **kwargs):

        if self.target_gov_state.get() == "TA":
            self.stage_sigs.update(
                [
                    ("zebra.pos_capt.gate.start", 0.1),
                ]
            )

        elif self.target_gov_state.get() == "SA":
            self.stage_sigs.update(
                [
                    ("zebra.pos_capt.gate.start", 180),

                ]
            )

        else:
            raise Exception("Target gov state not implemented!")

    def stage(self, *args, **kwargs):

        if gov_rbt.state.get() == 'M':
            raise GovernorError("Governor busy or in M during staging attempt")

        # Resolve any stage_sigs keys given as strings: 'a.b' -> self.a.b
        stage_sigs = OrderedDict()
        for k, v in self.stage_sigs.items():
            if isinstance(k, str):
                # Device.__getattr__ handles nested attr lookup
                stage_sigs[getattr(self, k)] = v
            else:
                stage_sigs[k] = v

        # Read current values, to be restored by unstage()
        original_vals = {sig: sig.get() for sig in stage_sigs}

        # We will add signals and values from original_vals to
        # self._original_vals one at a time so that
        # we can undo our partial work in the event of an error.

        # Apply settings.
        devices_staged = []
        try:
            for sig, val in stage_sigs.items():
                self.log.debug(
                    "Setting %s to %r (original value: %r)",
                    sig.name,
                    val,
                    original_vals[sig],
                )
                sig.set(val).wait(10)
                # It worked -- now add it to this list of sigs to unstage.
                self._original_vals[sig] = original_vals[sig]
            devices_staged.append(self)

            # Call stage() on child devices.
            for attr in self._sub_devices:
                device = getattr(self, attr)
                if hasattr(device, "stage"):
                    device.stage()
                    devices_staged.append(device)
        except Exception:
            self.log.debug(
                "An exception was raised while staging %s or "
                "one of its children. Attempting to restore "
                "original settings before re-raising the "
                "exception.",
                self.name,
            )
            self.unstage()
            raise

        def callback_armed(value, old_value, **kwargs):
            if old_value == 0 and value == 1:
                return True
            else:
                return False

        callback_armed_status = SubscriptionStatus(
            self.zebra.pos_capt.arm.output,
            callback_armed,
            run=False,
            settle_time=0.5,
        )
        self.zebra.pos_capt.arm.arm.set(1)
        callback_armed_status.wait(timeout=6)
        return devices_staged

    def unstage(self, **kwargs):
        super().unstage(**kwargs)

    def trigger(self):
        print('top aligner triggered')

        def callback_unarmed(value, old_value, **kwargs):
            if old_value == 1 and value == 0:
                return True
            else:
                return False

        callback_unarmed_status = SubscriptionStatus(
            self.zebra.pos_capt.arm.output,
            callback_unarmed,
            run=False,
            timeout=6,
        )

        if self.target_gov_state.get() in gov_rbt.reachable.get():
            gov_rbt.set(self.target_gov_state.get(), wait=True)
            # self.gonio_o.set(0)
            return callback_unarmed_status

        else:
            raise FailedStatus(
                f'{self.target_gov_state.get()} is wrong governor state for transition'
            )


class TopAlignerSlow(TopAlignerBase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def _configure_device(self, *args, **kwargs):

        self.stage_sigs.clear()
        self.stage_sigs.update(
            [
                ("topcam.cam.trigger_mode", 5),
                ("topcam.cam.image_mode", 1),
                ("topcam.cam.acquire", 1),
            ]
        )
        self.topcam.cam_mode.set("coarse_align")
        self.read_attrs = [
            "topcam.cv1.outputs.output8",
            "topcam.cv1.outputs.output9",
            "topcam.cv1.outputs.output10",
        ]

    def trigger(self):
        return self.topcam.trigger()

    def read(self):
        return self.topcam.read()


topcam = TopAlignCam("XF:17IDB-ES:AMX{Cam:9}", name="topcam")
top_aligner_fast = TopAlignerFast(name="top_aligner_fast")
top_aligner_slow = TopAlignerSlow(name="top_aligner_slow")
