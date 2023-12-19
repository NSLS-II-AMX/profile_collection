#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov  2 17:52:45 2022

@author: dkreitler
"""

from enum import Enum
from scipy.interpolate import interp1d
from bluesky.utils import FailedStatus
from bluesky.preprocessors import relative_set_decorator


def retry_move(func):
    def wrapper(*args, **kwargs):
        for j in range(2):
            try:
                yield from func(*args, **kwargs)
                break
            except FailedStatus:
                if j == 0:
                    print(f"{args[0].name} is stuck, retrying...")
                    yield from bps.sleep(0.2)
                else:
                    raise RuntimeError(
                        f"{args[0].name} is really stuck!")
    return wrapper


@retry_move
def mvr_with_retry(*args, **kwargs):
    yield from bps.mvr(*args, **kwargs)


@retry_move
def mv_with_retry(*args, **kwargs):
    yield from bps.mv(*args, **kwargs)


class TopPlanLimit(Enum):
    """Prevent large goniometer movements during topview plan. Rotating gonio
    with piezo stages extended can hit things like the collimator or beamstop.

    ...
    Attributes
    ----------
    DELTAY : float
        Gonio pin Y stage movement limit (microns)
    DELTAZ : float
        Gonio pin Z stage movement limit (microns)
    OMEGAMIN : float
        Gonio omega rotation stage movement limit (degrees)
    """

    DELTAY = DELTAZ = 2500
    OMEGAMIN = 400


def topview_plan():
    """Optical loop alignment plan that uses off-axis ProsilicaCam.
    1. Rotates loop to calculate rotation axis based on loop tip canny edge
    2. Safely moves loop tip to rotation axis
    3. Safely translate horizontal stage to ROI loosely defined by beam pos.
    4. Rotate loop to calculate rotation axis based on loop tip canny edge and
    phase angle that minimizes loop profile in off-axis cam, i.e. face on axis
    5. Safely move loop tip to rotation axis
    6. Safely rotate to face-on (on axis cam) omega

    Yields
    ------
    None

    """

    def inner_rot_scan(*args, **kwargs):
        """rotate omega with ADCompVision plugin configured to read loop tip
        Canny edge for bringing loop tip onto rotation axis.
        """

        # scan yields info for simultaneous face on, rot axis centering
        for j in range(2):
            try:
                scan_uid = yield from bp.scan(*args, **kwargs)
                break
            except FailedStatus:
                if j == 0:
                    print("Trigger or stage move failed, retrying")
                    yield from bps.sleep(0.1)
                else:
                    raise RuntimeError("Something went really wrong")

        omega_list = db[scan_uid].table()[top_aligner_slow.gonio_o.name]
        d = np.pi / 180

        # rot axis calculation
        A_rot = np.matrix(
            [[np.cos(omega * d), np.sin(omega * d), 1] for omega in omega_list]
        )
        b_rot = db[scan_uid].table()[
            top_aligner_slow.topcam.cv1.outputs.output9.name
        ]
        p = (
            np.linalg.inv(A_rot.transpose() * A_rot)
            * A_rot.transpose()
            * np.matrix(b_rot.to_numpy()).transpose()
        )
        delta_z_pix, delta_y_pix, rot_axis_pix = p[0], p[1], p[2]
        delta_y, delta_z = (
            delta_y_pix / top_aligner_slow.topcam.pix_per_um.get(),
            delta_z_pix / top_aligner_slow.topcam.pix_per_um.get(),
        )
        return delta_y, delta_z

    def inner_pseudo_fly_scan(*args, **kwargs):
        scan_uid = yield from bp.count(*args, **kwargs)
        omegas = db[scan_uid].table()[
            top_aligner_fast.zebra.pos_capt.data.enc4.name
        ][1]

        d = np.pi / 180

        # rot axis calculation, use linear regression
        A_rot = np.matrix(
            [[np.cos(omega * d), np.sin(omega * d), 1] for omega in omegas]
        )

        b_rot = db[scan_uid].table()[top_aligner_fast.topcam.out9_buffer.name][
            1
        ]
        p = (
            np.linalg.inv(A_rot.transpose() * A_rot)
            * A_rot.transpose()
            * np.matrix(b_rot).transpose()
        )

        delta_z_pix, delta_y_pix, rot_axis_pix = p[0], p[1], p[2]
        delta_y, delta_z = (
            delta_y_pix / top_aligner_fast.topcam.pix_per_um.get(),
            delta_z_pix / top_aligner_fast.topcam.pix_per_um.get(),
        )

        # face on calculation
        b = db[scan_uid].table()[top_aligner_fast.topcam.out10_buffer.name][1]

        sample = 300
        f_splines = interp1d(omegas, b)
        b_splines = f_splines(np.linspace(omegas[0], omegas[-1], sample))
        omega_min = np.linspace(omegas[0], omegas[-1], sample)[
            b_splines.argmin()
        ]
        print(f"SPLINES / {omega_min}")

        return delta_y, delta_z, omega_min

    # ROI check for pin
    scan_uid = yield from bp.count([top_aligner_slow.topcam], 1)
    if db[scan_uid].table()[top_aligner_slow.topcam.cv1.outputs.output9.name][1] < 0:
        print("no pin detected")
        return

    # coarse rotation axis align
    delta_y, delta_z = yield from inner_rot_scan(
        [top_aligner_slow.topcam], top_aligner_slow.gonio_o, 0, 180, 4
    )

    # prevent large, erratic movements
    if (abs(delta_y) > TopPlanLimit.DELTAY.value) or (
        abs(delta_z) > TopPlanLimit.DELTAZ.value
    ):
        return

    yield from mvr_with_retry(top_aligner_slow.gonio_py, delta_y)
    # yield from bps.sleep(0.1)
    yield from mvr_with_retry(top_aligner_slow.gonio_pz, -delta_z)

    # horizontal bump
    scan_uid = yield from bp.count([top_aligner_slow.topcam], 1)
    x = db[scan_uid].table()[top_aligner_slow.topcam.cv1.outputs.output8.name][1]
    delta_x = ((top_aligner_slow.topcam.roi2.size.x.get() / 2) -
               x) / top_aligner_slow.topcam.pix_per_um.get()
    yield from bps.mvr(gonio.gx, delta_x)

    # finer rotation axis align, plus more in-focus face on omega determined

    try:
        delta_y, delta_z, omega_min = yield from inner_pseudo_fly_scan(
            [top_aligner_fast]
        )
    except FailedStatus:
        print("arming problem...trying again")
        yield from bps.abs_set(
            top_aligner_fast.zebra.pos_capt.arm.disarm, 1, wait=True
        )
        yield from bps.sleep(0.5)
        yield from bps.mv(top_aligner_fast.gonio_o, 180)
        delta_y, delta_z, omega_min = yield from inner_pseudo_fly_scan(
            [top_aligner_fast]
        )

    # prevent large movements
    if (
        (abs(delta_y) > TopPlanLimit.DELTAY.value)
        or (abs(delta_z) > TopPlanLimit.DELTAZ.value)
        or (abs(omega_min) > TopPlanLimit.OMEGAMIN.value)
    ):
        return

    yield from mvr_with_retry(top_aligner_fast.gonio_py, delta_y)
    # yield from bps.sleep(0.1)
    yield from mvr_with_retry(top_aligner_fast.gonio_pz, -delta_z)
    yield from bps.mv(top_aligner_fast.gonio_o, omega_min)
