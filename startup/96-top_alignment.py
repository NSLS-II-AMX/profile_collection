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


class EpicsMotorSPMG(EpicsMotor):
    SPMG = Cpt(EpicsSignal, ".SPMG")


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
        self._update_stage_sigs(*args, **kwargs)
        super().stage(*args, **kwargs)


class ZebraMXOr(Zebra):
    or3 = Cpt(EpicsSignal, "OR3_ENA:B3")
    or3loc = Cpt(EpicsSignal, "OR3_INP4")
    armsel = Cpt(EpicsSignal, "PC_ARM_SEL")


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
        super().stage(*args, **kwargs)

    def trigger(self):
        raise NotImplementedError("Subclasses must implement custom trigger")

    def unstage(self, *args, **kwargs):
        if type(self) == TopAlignerBase:
            raise NotImplementedError("TopAlignerBase has no unstage method")
        super().unstage(*args, **kwargs)


class TopAlignerFast(TopAlignerBase):

    zebra = Cpt(ZebraMXOr, "XF:17IDB-ES:AMX{Zeb:2}:")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def _configure_device(self, *args, **kwargs):
        self.read_attrs = ["topcam", "zebra"]
        self.stage_sigs.clear()
        self.topcam.cam_mode.set("fine_face")

        self.stage_sigs.update(
            [
                ("topcam.cam.trigger_mode", 1),
                ("topcam.cam.image_mode", 2),
                ("topcam.cam.acquire", 1),
                ("gonio_o.velocity", 90),  # slow down to get picture taken
            ]
        )
        self.zebra.stage_sigs.update(
            [
                ("pos_capt.source", "Enc4"),
                ("pos_capt.direction", 1),
                ("armsel", 0),
                ("pos_capt.gate.start", 179.5),
                ("pos_capt.gate.width", 4.5),
                ("pos_capt.gate.step", 9),
                ("pos_capt.gate.num_gates", 20),
                ("pos_capt.pulse.start", 0),
                ("pos_capt.pulse.width", 4),
                ("pos_capt.pulse.step", 5),
                ("pos_capt.pulse.delay", 0),
                ("pos_capt.pulse.max_pulses", 1),
                ("or3", 1),
                ("or3loc", 30),
            ]
        )
        self.topcam.read_attrs = ["out9_buffer", "out10_buffer"]
        self.zebra.read_attrs = ["pos_capt.data.enc4"]

    def stage(self, *args, **kwargs):
        super().stage(*args, **kwargs)

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
        callback_armed_status.wait(timeout=3)

    def unstage(self, **kwargs):
        super().unstage(**kwargs)
        # self.zebra.pos_capt.arm.disarm.set(1)

    def trigger(self):
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
        self.gonio_o.set(0)
        return callback_unarmed_status


class TopAlignerSlow(TopAlignerBase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def _configure_device(self, *args, **kwargs):

        self.stage_sigs.clear()
        self.stage_sigs.update(
            [
                ("topcam.cam.trigger_mode", 5),
                ("topcam.cam.image_mode", 1),
                ("topcam.cam.acquire", 0),
            ]
        )
        self.topcam.cam_mode.set("coarse_align")
        self.read_attrs = [
            "topcam.cv1.outputs.output9",
            "topcam.cv1.outputs.output10",
            "gonio_o",
        ]

    def trigger(self):
        return self.topcam.trigger()

    def read(self):
        return self.topcam.read()


top_aligner_fast = TopAlignerFast(name="top_aligner_fast")
top_aligner_slow = TopAlignerSlow(name="top_aligner")
