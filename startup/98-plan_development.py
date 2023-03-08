#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 27 16:39:13 2022

@author: dkreitler
"""

from toolz import partition


def rel_scan_no_reset(detectors, *args, num=None, per_step=None, md=None):
    """
    Scan over one multi-motor trajectory relative to current position.
    Parameters
    ----------
    detectors : list
        list of 'readable' objects
    *args :
        For one dimension, ``motor, start, stop``.
        In general:
        .. code-block:: python
            motor1, start1, stop1,
            motor2, start2, start2,
            ...,
            motorN, startN, stopN,
        Motors can be any 'settable' object (motor, temp controller, etc.)
    num : integer
        number of points
    per_step : callable, optional
        hook for customizing action of inner loop (messages per step).
        See docstring of :func:`bluesky.plan_stubs.one_nd_step` (the default)
        for details.
    md : dict, optional
        metadata
    """
    _md = {"plan_name": "rel_scan"}
    md = md or {}
    _md.update(md)
    motors = [motor for motor, start, stop in partition(3, args)]

    @bpp.relative_set_decorator(motors)
    def inner_rel_scan():
        return (
            yield from bp.scan(
                detectors, *args, num=num, per_step=per_step, md=_md
            )
        )

    return (yield from inner_rel_scan())


def beam_align():
    def lin_reg(independent, dependent, goal, **kwargs):
        b = dependent
        A = np.matrix([[pos, 1] for pos in independent])
        p = (
            np.linalg.inv(A.transpose() * A)
            * A.transpose()
            * np.matrix(b.to_numpy()).transpose()
        )
        best = (goal - p[1]) / p[0]
        return best

    yield from bps.abs_set(rot_aligner.cam_hi.cam_mode, "beam_align")

    # which direction, x pos. pitch beam outboard (-), y pos. pitch beam up (+)
    scan_uid = yield from bp.count([rot_aligner.cam_hi], 1)
    centroid_x, centroid_y = (
        db[scan_uid].table()[rot_aligner.cam_hi.cv1.outputs.output1.name][1],
        db[scan_uid].table()[rot_aligner.cam_hi.cv1.outputs.output2.name][1],
    )
    delta_x_pix, delta_y_pix = (centroid_x - 320), (centroid_y - 256)
    print(delta_x_pix, delta_y_pix)
    if abs(delta_x_pix) > 2:

        scan_uid = yield from rel_scan_no_reset(
            [rot_aligner.cam_hi],
            kbt.hor,
            0,
            0.4 * -(delta_x_pix / abs(delta_x_pix)),
            10,
        )
        scan_df = db[scan_uid].table()
        print(scan_df)
        best_hor_voltage = lin_reg(
            scan_df[kbt.hor.readback.name],
            scan_df[rot_aligner.cam_hi.cv1.outputs.output1.name],
            320,
        )
        print(best_hor_voltage)
        yield from bps.mv(kbt.hor, best_hor_voltage)
        yield from bps.sleep(1)

    if abs(delta_y_pix) > 2:
        scan_uid = yield from rel_scan_no_reset(
            [rot_aligner.cam_hi],
            kbt.ver,
            0,
            0.4 * (delta_y_pix / abs(delta_y_pix)),
            10,
        )
        scan_df = db[scan_uid].table()
        best_ver_voltage = lin_reg(
            scan_df[kbt.ver.readback.name],
            scan_df[rot_aligner.cam_hi.cv1.outputs.output2.name],
            256,
        )
        print(best_ver_voltage)
        yield from bps.mv(kbt.ver, best_ver_voltage)
