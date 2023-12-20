#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 18 13:16:27 2023

@author: dkreitler
"""
from collections import OrderedDict
from ophyd.areadetector.filestore_mixins import FileStorePluginBase
from ophyd.areadetector.plugins import JPEGPlugin
from ophyd import Kind, DeviceStatus

import requests


class FileStoreJPEG(FileStorePluginBase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.filestore_spec = "AD_JPEG"  # spec name stored in resource doc
        self.stage_sigs.update(
            [
                ("file_template", "%s%s_%6.6d.jpg"),
                ("file_write_mode", "Single"),
            ]
        )
        # 'Single' file_write_mode means one image : one file.
        # It does NOT mean that 'num_images' is ignored.

    def get_frames_per_point(self):
        return self.parent.cam.num_images.get()

    def stage(self):
        super().stage()
        # this over-rides the behavior is the base stage
        self._fn = self._fp

        resource_kwargs = {
            "template": self.file_template.get(),
            "filename": self.file_name.get(),
            "frame_per_point": self.get_frames_per_point(),
        }
        self._generate_resource(resource_kwargs)


class JPEGPluginWithFileStore(JPEGPlugin, FileStoreJPEG):
    pass


class LoopDetector(Device):
    url = Cpt(
        Signal, value='http://mars8.nsls2.bnl.gov:8000/predict', kind='config'
    )
    filename = Cpt(Signal, value=None)
    box = Cpt(Signal, value=[])

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.read_attrs = ['box']

    def trigger(self):

        filename_dict = {"file": Path(self.filename.get()).open('rb')}
        response = requests.post(self.url.get(), files=filename_dict)
        response.raise_for_status()
        json_response = response.json()
        if json_response['pred_boxes']:
            self.box.put(response.json()['pred_boxes'][0]['box'])
        else:
            self.box.put([])
        response_status = DeviceStatus(self.box, timeout=10)
        response_status.set_finished()

        return response_status


class TwoClickLowMag(StandardProsilica):
    cv1 = Cpt(CVPlugin, "CV1:")
    cam_mode = Cpt(Signal, value=None, kind="config")
    pix_per_um = Cpt(Signal, value=1, kind="config")

    jpeg = Cpt(
        JPEGPluginWithFileStore,
        "JPEG1:",
        write_path_template="/nsls2/data/amx/legacy/topcam",
    )

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.read_attrs = ["cv1", "jpeg"]
        self.jpeg.read_attrs = ['full_file_name']
        self.cv1.read_attrs = ["outputs"]
        self.cv1.outputs.read_attrs = ["output1", "output2", "output3"]
        self.cam_mode.subscribe(self._update_stage_sigs, event_type="value")

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
        elif self.cam_mode.get() == "two_click":
            self.stage_sigs.update(
                [
                    ("jpeg.nd_array_port", "ROI2")
                ]
            )

    def stage(self, *args, **kwargs):
        self._update_stage_sigs(*args, **kwargs)
        super().stage(*args, **kwargs)


class WorkPositions(Device):
    gx = Cpt(EpicsSignal, '{Gov:Robot-Dev:gx}Pos:Work-Pos')
    py = Cpt(EpicsSignal, '{Gov:Robot-Dev:gpy}Pos:Work-Pos')
    pz = Cpt(EpicsSignal, '{Gov:Robot-Dev:gpz}Pos:Work-Pos')
    o = Cpt(EpicsSignal, '{Gov:Robot-Dev:go}Pos:Work-Pos')


work_pos = WorkPositions("XF:17IDB-ES:AMX", name="work_pos")

two_click_low = TwoClickLowMag("XF:17IDB-ES:AMX{Cam:6}", name="two_click_low")
loop_detector = LoopDetector(name="loop_detector")
