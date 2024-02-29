#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 27 16:39:13 2022

@author: dkreitler
"""

from toolz import partition
from bluesky.preprocessors import reset_positions_decorator
print(f"Loading {__file__}")


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
    _md = {"plan_name": "rel_scan_no_reset"}
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


def cleanup_beam_align():
    yield from bps.mv(sht.r, 20)  # close shutter
    # safely disable jpeg plugin
    yield from bps.abs_set(cam_hi_ba.jpeg.auto_save, 0)
    yield from bps.abs_set(cam_hi_ba.cam_mode, "beam_align")


@finalize_decorator(cleanup_beam_align)
@reset_positions_decorator([gov_rbt])
def beam_align():
    """bluesky plan for beam alignment with ADCompVision plugin and KB mirror
    piezo tweaks. This plan can be run from any governor state that can access
    BL if no sample is mounted."""

    # do nothing if there is a sample mounted to avoid collisions
    if smart_magnet.sample_detect.get() == 0:
        raise Exception("Sample mounted on gonio! Avoided collision")

    # wait for attenuators to finish moving, due to IOC
    yield from bps.abs_set(mxatten, 0.002)
    yield from bps.sleep(5)

    # transition to BL and open shutter
    yield from bps.abs_set(gov_rbt, "BL", wait=True)
    yield from bps.mv(sht.r, 0)

    yield from bps.abs_set(cam_hi_ba.cam_mode, "beam_align")

    # set aligned to false to before align attempt
    yield from bps.abs_set(kbt.hor.aligned, False)
    yield from bps.abs_set(kbt.ver.aligned, False)

    n_tries = 0
    while n_tries <= 3:

        # beam is aligned, no need to proceed
        if (kbt.hor.aligned.get() and kbt.ver.aligned.get()):
            break

        # which direction, x pos. pitch beam outboard (-), y pos. pitch beam up (+)
        scan_uid = yield from bp.count([cam_hi_ba], 1)
        centroid_x, centroid_y = (
            db[scan_uid].table()[cam_hi_ba.cv1.outputs.output1.name][1],
            db[scan_uid].table()[cam_hi_ba.cv1.outputs.output2.name][1],
        )

        if np.isclose(0, centroid_x) or np.isclose(0, centroid_y):
            raise Exception("No centroid detected!")

        yield from bps.abs_set(kbt.hor.delta_px, (centroid_x - 320))
        yield from bps.abs_set(kbt.ver.delta_px, -(centroid_y - 256))

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

        for axis, signal, center in (
            kbt.hor,
            cam_hi_ba.cv1.outputs.output1,
            320,
        ), (kbt.ver, cam_hi_ba.cv1.outputs.output2, 256):
            # skip if we are within 1 um
            if abs(axis.delta_px.get()) > 2 and not axis.aligned.get():
                scan_uid = yield from rel_scan_no_reset(
                    [cam_hi_ba],
                    axis,
                    0,
                    0.4 * -(axis.delta_px.get() / abs(axis.delta_px.get())),
                    10,
                )
                scan_df = db[scan_uid].table()
                best_voltage = lin_reg(
                    scan_df[axis.readback.name],
                    scan_df[signal.name],
                    center,
                )
                yield from bps.mv(axis, best_voltage)
                yield from bps.sleep(1)
            else:
                print(f'{axis.name} is aligned')
                yield from bps.abs_set(axis.aligned, True, wait=True)
        n_tries += 1

    # annotate reference image
    yield from bps.abs_set(cam_hi_ba.cam_mode, "beam_align_check")
    yield from bps.abs_set(cam_hi_ba.jpeg.auto_save, 1, wait=True)
    scan_uid = yield from bp.count([cam_hi_ba], 1)

    rd = [k for k in db[scan_uid].documents('primary') if k[0] == 'resource']
    filename = rd[-1][-1]['resource_kwargs']['filename']
    resource_path = rd[-1][-1]['resource_path']
    template = rd[-1][-1]['resource_kwargs']['template']
    root = rd[-1][-1]['root']
    full_resource_path = root + resource_path + '/'

    # grab the 0 index, single image
    _fp = template % (full_resource_path, filename, 0)
    print(_fp)
    add_cross(_fp)
    t_ = db[scan_uid].table()['time'][1]
    add_text_bottom_left(_fp, f'{t_}')
