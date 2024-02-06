#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 18 15:54:11 2023

@author: dkreitler
"""


def loop_detection_plan():

    # face on attempt, most features, should work
    yield from bps.abs_set(two_click_low.cam_mode, "two_click", wait=True)

    yield from bp.count([two_click_low], 1)
    yield from bps.abs_set(
        loop_detector.filename, two_click_low.jpeg.full_file_name.get()
    )

    scan_uid = yield from bp.count([loop_detector], 1)
    box_coords_face = db[scan_uid].table()['loop_detector_box'][1]

    try:
        mean_x = (box_coords_face[0] + box_coords_face[2]) / 2
        mean_y = (box_coords_face[1] + box_coords_face[3]) / 2
    except IndexError:
        print("loop detection failed")
        return

    delta_x = (mean_x - 320) * 2*two_click_low.pix_per_um.get()
    delta_cam_y = (mean_y - 256) * 2*two_click_low.pix_per_um.get()
    omega = gonio.o.user_readback.get()
    d = np.pi/180

    real_y = delta_cam_y * np.cos(omega * d)
    real_z = delta_cam_y * np.sin(omega * d)

    yield from bps.mvr(gonio.gx, delta_x)
    yield from bps.mvr(gonio.py, -real_y)
    yield from bps.mvr(gonio.pz, -real_z)

    # orthogonal face, use loop model only if predicted width matches face on
    # otherwise, threshold
    yield from bps.mvr(gonio.o, 90)
    yield from bps.abs_set(
        two_click_low.x_min, box_coords_face[0], wait=True
    )
    yield from bps.abs_set(
        two_click_low.x_max, box_coords_face[2], wait=True
    )

    scan_uid = yield from bp.count([two_click_low], 1)
    mean_y_threshold = (
        db[scan_uid].table()['two_click_low_cv1_outputs_output2'][1]
        + db[scan_uid].table()['two_click_low_cv1_outputs_output1'][1]) / 2

    yield from bps.abs_set(
        loop_detector.filename, two_click_low.jpeg.full_file_name.get()
    )

    scan_uid = yield from bp.count([loop_detector], 1)
    box_coords_ortho = db[scan_uid].table()['loop_detector_box'][1]

    try:
        mean_y = (box_coords_ortho[1] + box_coords_ortho[3]) / 2
        # sum of squared difference, face-on vs. ortho width similarity
        ssd_ratio = (
            ((box_coords_face[0] - box_coords_ortho[0])**2 +
             (box_coords_face[2] - box_coords_ortho[2])**2)
        ) / (box_coords_face[0] - box_coords_face[2])**2

    except IndexError:
        print("loop detection failed")
        mean_y = mean_y_threshold
        ssd_ratio = 10000

    if (ssd_ratio > 0.01):
        print(f'ssd_ratio of {ssd_ratio}, thresholding for loop sideview')
        mean_y = mean_y_threshold
        if mean_y == -1:
            print('threshold of -1 detected, something is wrong')
            yield from bps.mvr(gonio.o, -90)
            return

    delta_cam_y = (mean_y - 256) * 2*two_click_low.pix_per_um.get()
    omega = gonio.o.user_readback.get()
    d = np.pi/180
    real_y = delta_cam_y * np.cos(omega * d)
    real_z = delta_cam_y * np.sin(omega * d)

    yield from bps.mvr(gonio.py, -real_y)
    yield from bps.mvr(gonio.pz, -real_z)
    yield from bps.mvr(gonio.o, -90)


def two_click_center():

    yield from topview_optimized()
    yield from loop_detection_plan()
