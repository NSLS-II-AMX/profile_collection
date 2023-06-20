#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov  2 11:51:23 2022

@author: dkreitler
"""
from ophyd.areadetector.filestore_mixins import FileStoreTIFF


class TIFFPluginWithFileStore(TIFFPlugin, FileStoreTIFF):
    pass


class TopAlignCam(StandardProsilica):
    _default_read_attrs = ["cv1", "tiff"]
    cv1 = Cpt(CVPlugin, "CV1:")
    tiff = Cpt(
        TIFFPluginWithFileStore,
        "TIFF1:",
        write_path_template="/nsls2/data/staff/dkreitler/softioc-amx_data/topcam",
    )
    cam_mode = Cpt(Signal, value=None, kind="config")
    pix_per_um = Cpt(Signal, value=0.164, doc="pixels per um")
    out10_buffer = Cpt(EpicsSignalRO, "Out10:compress",
                       doc="circular buffer for ADCompVision output10 (thresholding loop profile)")
    out10_reset = Cpt(EpicsSignal, "Out10:compress.RES")
    out9_buffer = Cpt(EpicsSignalRO, "Out9:compress")
    out9_reset = Cpt(EpicsSignal, "Out9:compress.RES")

    def __init__(self, *args, **kwargs):
        super().__init__(
            *args,
            **kwargs,
        )
        self.tiff.read_attrs = []
        self.cv1.read_attrs = ["outputs"]
        self.cv1.outputs.read_attrs = [
            "output8",
            "output9",
            "output10",
        ]
        self.cam_mode.subscribe(self._update_stage_sigs, event_type="value")

    def _update_stage_sigs(self, *args, **kwargs):
        self.stage_sigs.clear()
        self.stage_sigs.update(
            [
                ("cam.acquire", 0),
                ("cam.image_mode", 0),
                ("cv1.func_sets.func_set1", "Canny Edge Detection"),
                ("cv1.inputs.input1", 8),
                ("cv1.inputs.input2", 3),
                ("cv1.inputs.input3", 1),
                ("cv1.inputs.input4", 3),
                ("cv1.inputs.input5", 6),
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


class RotationalAligner(Device):
    zebra = Cpt(ZebraMXOr, 'XF:17IDB-ES:AMX{Zeb:2}:')
    camera = Cpt(TopAlignCam, 'XF:17IDB-ES:AMX{Cam:9}', kind="hinted")
    gonio_o = Cpt(EpicsMotor, "XF:17IDB-ES:AMX{Gon:1-Ax:O}Mtr")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.stage_sigs.update(
            [
                ("zebra.pos_capt.source", "Enc4"),
                ("zebra.pos_capt.direction", 1),
                ("zebra.armsel", 0),
                ("zebra.pos_capt.gate.start", 179.9),
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
        self.camera.stage_sigs.update(
            [
                ("cam.trigger_mode", 1),
                ("cam.image_mode", 2),
                ("cam.acquire", 1),
            ]
        )
        self.read_attrs = ["camera.out10_buffer", "camera.out9_buffer"]

    def stage(self, *args, **kwargs):
        # self.camera.out10_reset.set(1)
        # self.camera.out9_reset.set(1)
        super().stage(*args, **kwargs)

        def callback_armed(value, old_value, **kwargs):
            if old_value == 0 and value == 1:
                return True
            else:
                return False
        callback_armed_status = SubscriptionStatus(
            self.zebra.pos_capt.arm.output, callback_armed, run=False)
        self.zebra.pos_capt.arm.arm.set(1, settle_time=0.5)
        callback_armed_status.wait()
        return callback_armed_status

    def unstage(self, **kwargs):
        super().unstage(**kwargs)
        self.zebra.pos_capt.arm.disarm.set(1)

    def trigger(self):
        def callback_unarmed(value, old_value, **kwargs):
            if old_value == 1 and value == 0:
                return True
            else:
                return False
        callback_unarmed_status = SubscriptionStatus(
            self.zebra.pos_capt.arm.output, callback_unarmed, run=False)
        self.gonio_o.set(0)
        return(callback_unarmed_status)

    # def read(self):
    #
    #    return(


rot_align_test = RotationalAligner(name='rat')
