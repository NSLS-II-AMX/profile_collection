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


topcam = TopAlignCam(
    "XF:17IDB-ES:AMX{Cam:9}",
    name="topcam",
)
