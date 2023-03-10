#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 27 16:39:13 2022

@author: dkreitler
"""

from toolz import partition
from mxtools.governor import _make_governors

gov = _make_governors("XF:17IDB-ES:AMX", name="gov")
gov_rbt = gov.gov.Robot


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
    """bluesky plan for beam alignment with ADCompVision plugin and KB mirror
    piezo tweaks. This plan can be run from any governor state that can access
    BL if no sample is mounted."""

    # do nothing if there is a sample mounted to avoid collisions
    if smart_magnet.sample_detect.get() == 0:
        return

    # wait for attenuators to finish moving
    yield from bps.abs_set(mxatten, 0.002)
    yield from bps.sleep(5)

    # transition to BL and open shutter
    yield from bps.abs_set(gov_rbt, "BL", wait=True)
    yield from bps.mv(sht.r, 0)

    yield from bps.abs_set(rot_aligner.cam_hi.cam_mode, "beam_align")

    # which direction, x pos. pitch beam outboard (-), y pos. pitch beam up (+)
    scan_uid = yield from bp.count([rot_aligner.cam_hi], 1)
    centroid_x, centroid_y = (
        db[scan_uid].table()[rot_aligner.cam_hi.cv1.outputs.output1.name][1],
        db[scan_uid].table()[rot_aligner.cam_hi.cv1.outputs.output2.name][1],
    )
    yield from bps.abs_set(kbt.hor.delta_px, (centroid_x - 320))
    yield from bps.abs_set(kbt.ver.delta_px, (centroid_y - 256))

    def lin_reg(independent, dependent, goal, **kwargs) -> float:
        b = dependent
        A = np.matrix([[pos, 1] for pos in independent])
        p = (
            np.linalg.inv(A.transpose() * A)
            * A.transpose()
            * np.matrix(b.to_numpy()).transpose()
        )
        best = (goal - p[1]) / p[0]
        return best

    for axis, center in (kbt.hor, 320), (kbt.ver, 256):
        # skip if we are within 1 um
        if abs(axis.delta_px.get()) > 2:
            scan_uid = yield from rel_scan_no_reset(
                [rot_aligner.cam_hi],
                axis,
                0,
                0.4 * -(axis.delta_px.get() / abs(axis.delta_px.get())),
                10,
            )
            scan_df = db[scan_uid].table()
            best_voltage = lin_reg(
                scan_df[axis.readback.name],
                scan_df[rot_aligner.cam_hi.cv1.outputs.output1.name],
                center,
            )
            yield from bps.mv(axis, best_voltage)
            yield from bps.sleep(1)

    # close shutters and reset attenuators for manual viewing
    yield from bps.mv(sht.r, 20)
    yield from bps.abs_set(mxatten, 0.01)
