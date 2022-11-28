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
    pix_per_um = Cpt(Signal, value=0.164, doc="pixels per um")

    def __init__(self, *args, **kwargs):
        super().__init__(
            *args,
            **kwargs,
        )
        self.tiff.read_attrs = []
        self.cv1.read_attrs = ["outputs"]
        self.cv1.outputs.read_attrs = ["output9", "output10"]
        self.stage_sigs.update(
            [
                ("cam.acquire", 0),
                ("cam.image_mode", 0),
                ("cv1.nd_array_port", "ROI1"),
                ("cv1.func_sets.func_set1", "Canny Edge Detection"),
                ("cv1.inputs.input1", 8),
                ("cv1.inputs.input2", 3),
                ("cv1.inputs.input3", 1),
                ("cv1.inputs.input4", 3),
                ("cv1.inputs.input5", 20),
                ("cv1.inputs.input6", 1),
                ("trans1.nd_array_port", "PROC1"),
                ("roi1.nd_array_port", "TRANS1"),
                ("tiff.nd_array_port", "CV1"),
            ]
        )


topcam = TopAlignCam(
    "XF:17IDB-ES:AMX{Cam:9}",
    name="topcam",
)
