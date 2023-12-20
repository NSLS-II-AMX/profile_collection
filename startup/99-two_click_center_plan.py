#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 18 15:54:11 2023

@author: dkreitler
"""


def loop_detection_plan():

    yield from bp.count([two_click_low], 1)
    yield from bps.abs_set(
        loop_detector.filename, two_click_low.jpeg.full_file_name.get()
    )

    scan_uid = yield from bp.count([loop_detector], 1)
    box_coords = db[scan_uid].table()['loop_detector_box'][1]

    try:
        mean_x = (box_coords[0] + box_coords[2]) / 2
        mean_y = (box_coords[1] + box_coords[3]) / 2
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

    # orthogonal face
    yield from bps.mvr(gonio.o, 90)
    yield from bp.count([two_click_low], 1)
    yield from bps.abs_set(
        loop_detector.filename, two_click_low.jpeg.full_file_name.get()
    )

    scan_uid = yield from bp.count([loop_detector], 1)
    box_coords = db[scan_uid].table()['loop_detector_box'][1]

    try:
        mean_y = (box_coords[1] + box_coords[3]) / 2
    except IndexError:
        print("loop detection failed")
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
    yield from bps.abs_set(gov_rbt, "TA", wait=True)
    yield from topview_plan()

    yield from bps.abs_set(work_pos.gx, gonio.gx.user_readback.get())
    yield from bps.abs_set(work_pos.py, gonio.py.user_readback.get())
    yield from bps.abs_set(work_pos.pz, gonio.pz.user_readback.get())
    yield from bps.abs_set(work_pos.o, gonio.o.user_readback.get())

    yield from bps.abs_set(gov_rbt, "SA", wait=True)
    yield from loop_detection_plan()
