#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov  2 17:52:45 2022

@author: dkreitler
"""

from enum import Enum


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
        Canny edge and thresholding for face on loop centering.
        """

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

    # configure cam
    yield from bps.abs_set(topcam.cam_mode, "coarse_align")

    # ROI check for pin
    scan_uid = yield from bp.count([topcam], 1)
    if db[scan_uid].table()[f"{topcam.cv1.outputs.output9.name}"][1] < 0:
        return

    # coarse rotation axis align
    delta_y, delta_z, _ = yield from inner_rot_scan(
        [topcam], gonio.o, 0, 120, 3
    )

    # prevent large, erratic movements
    if (abs(delta_y) > TopPlanLimit.DELTAY.value) or (
        abs(delta_z) > TopPlanLimit.DELTAZ.value
    ):
        return

    yield from bps.mvr(gonio.py, delta_y)
    yield from bps.mvr(gonio.pz, -delta_z)

    # horizontal bump
    scan_uid = yield from bp.count([topcam], 1)
    x = db[scan_uid].table()[f"{topcam.cv1.outputs.output8.name}"][1]
    delta_x = ((topcam.roi2.size.x.get() / 2) - x) / topcam.pix_per_um.get()
    yield from bps.mvr(gonio.gx, delta_x)

    # finer rotation axis align, plus more in-focus face on omega determined
    yield from bps.abs_set(topcam.cam_mode, "fine_face")
    delta_y, delta_z, omega_min = yield from inner_rot_scan(
        [topcam], gonio.o, 120, 0, 6
    )

    # prevent large movements
    if (
        (abs(delta_y) > TopPlanLimit.DELTAY.value)
        or (abs(delta_z) > TopPlanLimit.DELTAZ.value)
        or (abs(omega_min) > TopPlanLimit.OMEGAMIN.value)
    ):
        return

    yield from bps.mvr(gonio.py, delta_y)
    yield from bps.mvr(gonio.pz, -delta_z)
    yield from bps.mv(gonio.o, omega_min)
