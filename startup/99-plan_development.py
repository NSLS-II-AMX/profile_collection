#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov  2 17:52:45 2022

@author: dkreitler
"""


def top_plan():
    def inner_rot_scan(*args, **kwargs):
        # scan yields info for simultaneous face on, rot axis centering
        scan_uid = yield from bp.scan(*args, **kwargs)
        omega_list = db[scan_uid].table()["gonio_o"]
        d = np.pi / 180

        # rot axis calculation
        A_rot = np.matrix(
            [[np.cos(omega * d), np.sin(omega * d), 1] for omega in omega_list]
        )
        b_rot = db[scan_uid].table()[f"{topcam.cv1.outputs.output9.name}"]
        p = (
            np.linalg.inv(A_rot.transpose() * A_rot)
            * A_rot.transpose()
            * np.matrix(b_rot.to_numpy()).transpose()
        )
        delta_z_pix, delta_y_pix, rot_axis_pix = p[0], p[1], p[2]
        delta_y, delta_z = (
            delta_y_pix / topcam.pix_per_um.get(),
            delta_z_pix / topcam.pix_per_um.get(),
        )

        # face on calculation
        A = np.matrix(
            [
                [np.cos(2 * omega * d), np.sin(2 * omega * d), 1]
                for omega in omega_list
            ]
        )

        b = db[scan_uid].table()[f"{topcam.cv1.outputs.output10.name}"]
        p0 = (
            np.linalg.inv(A.transpose() * A)
            * A.transpose()
            * np.matrix(b.to_numpy()).transpose()
        )
        min1 = (270 - 180 * np.arctan2(p0[0], p0[1]) / np.pi) / 2
        min2 = (-90 - 180 * np.arctan2(p0[0], p0[1]) / np.pi) / 2
        omega_min = [min1, min2][np.abs([min1, min2]).argmin()]
        return delta_y, delta_z, omega_min

    # coarse rotation axis align
    delta_y, delta_z, _ = yield from inner_rot_scan(
        [topcam], gonio.o, 0, 180, 3
    )
    yield from bps.mvr(gonio.py, delta_y)
    yield from bps.mvr(gonio.pz, -delta_z)

    # finer rotation axis align, plus more in-focus face on omega determined
    delta_y, delta_z, omega_min = yield from inner_rot_scan(
        [topcam], gonio.o, 110, 0, 6
    )
    yield from bps.mvr(gonio.py, delta_y)
    yield from bps.mvr(gonio.pz, -delta_z)
    yield from bps.mv(gonio.o, omega_min)
