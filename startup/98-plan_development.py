#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 27 16:39:13 2022

@author: dkreitler
"""


def beam_align():
    yield from bps.abs_set(rot_aligner.cam_hi.cam_mode, "beam_align")

    def lin_reg(independent, dependent, target, **kwargs):
        b = dependent
        A = np.matrix([[pos, 1] for pos in independent])
        p = (
            np.linalg.inv(A.transpose() * A)
            * A.transpose()
            * np.matrix(b.to_numpy()).transpose()
        )
        best = (target - p[1]) / p[0]
        return best

    scan_uid = yield from bp.rel_scan(
        [rot_aligner.cam_hi], kbt.hor, -0.2, 0.2, 10
    )
    scan_df = db[scan_uid].table()
    best_hor_voltage = lin_reg(
        scan_df[f"{kbt.hor.voltage.name}"],
        scan_df[f"{rot_aligner.cam_hi.stats4.centroid.x.name}"],
        320,
    )

    yield from bps.mv(kbt.hor, best_hor_voltage)
    yield from bps.sleep(1)

    scan_uid = yield from bp.scan([rot_aligner.cam_hi], kbt.ver, -0.2, 0.2, 10)
    scan_df = db[scan_uid].table()
    best_ver_voltage = lin_reg(
        scan_df[f"{kbt.ver.voltage.name}"],
        scan_df[f"{rot_aligner.cam_hi.stats4.centroid.y.name}"],
        256,
    )
    yield from bps.mv(kbt.ver, best_ver_voltage)
