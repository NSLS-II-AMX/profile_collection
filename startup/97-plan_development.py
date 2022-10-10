"""bluesky plans for aligning a rotation alignment pin

Some assumptions of alignment pin hardware are inevitiably built in. Here,
we measure the vertical position of a rotation axis using a tapered alignment
pin with a protective sheath, then the pin is moved to that position and ROIs
are updated so that the center of the ROIs coincide with the rotation axis.

No assumptions or updates are made to horizontal positions."""


def measure_rot_axis(
    detector, rot_motor, start, *, sweep_width=270, n_steps=4
):
    """make n_steps number of measurements and perform linear regression to
    calculate the vertical position of the rotation axis in camera coordinates
    in addition to the delta_y, delta_z offsets

    y0 = delta_y_pix*cos(omega) + delta_z_pix*sin(omega)

    y0: vertical position of rotation axis in camera pixels
    delta_y_pix: vertical displacement of object from y0 at omega=0
    delta_z_pix: vertical displacement of object from y0 at omega=90
    """
    detector.cam_mode.put("rot_align")
    scan_uid = yield from bp.scan(
        [detector], rot_motor, start, start + sweep_width, n_steps
    )
    b = db[scan_uid].table()[f"{detector.stats4.max_xy.y.name}"]
    A = np.matrix(
        [
            [np.cos(np.deg2rad(omega)), np.sin(np.deg2rad(omega)), 1]
            for omega in np.linspace(*(start, start + sweep_width, n_steps))
        ]
    )
    p = (
        np.linalg.inv(A.transpose() * A)
        * A.transpose()
        * np.matrix(b.to_numpy()).transpose()
    )
    delta_y_pix, delta_z_pix, rot_axis_pix = p[0], p[1], p[2]
    delta_y, delta_z = (
        delta_y_pix / detector.pix_per_um.get(),
        delta_z_pix / detector.pix_per_um.get(),
    )
    return delta_y, delta_z, rot_axis_pix


def measure_tip_dist(detector, *, coarse_align_pix=215):
    """use canny edge detection to calculate current distance in um of
    alignment pin tip from arbitrary camera position"""
    yield from bps.abs_set(detector.cam_mode, "edge_detection")
    scan_uid = yield from bp.count([detector], 1)
    left_pixel = db[scan_uid].table()[f"{detector.cv1.outputs.output7.name}"][
        1
    ]
    delta_x_pix = left_pixel - coarse_align_pix
    return delta_x_pix / detector.pix_per_um.get()


def measure_opening_dist(detector, roi):
    """use openCV ADPlugin to detect largest centroid and make best effort
    to calculate the stage movements required to center of roi. Omega may be
    at an arbitrary position so movements must be made using camera axes"""
    yield from bps.abs_set(detector.cam_mode, "centroid")
    scan_uid = yield from bp.count([detector], 1)
    centroid_x = db[scan_uid].table()[f"{detector.cv1.outputs.output1.name}"][
        1
    ]
    centroid_y = db[scan_uid].table()[f"{detector.cv1.outputs.output2.name}"][
        1
    ]
    # change origin to center of roi
    # TODO define roi plugins with center
    delta_x_pix = centroid_x - (
        roi.min_xyz.min_x.get() + (roi.size.x.get() / 2)
    )
    # unfortunately openCV centroids place origin at bottom left, must correct
    # TODO common coordinate system
    delta_y_pix = (roi.max_xy.y.get() - centroid_y) - (
        roi.min_xyz.min_y.get() + (roi.size.y.get() / 2)
    )
    delta_x, delta_y = (
        delta_x_pix / detector.pix_per_um.get(),
        delta_y_pix / detector.pix_per_um.get(),
    )
    return delta_x, delta_y


def align_centroid(detector, roi, x_axis, y_axis, rot_axis, *, n_steps=3):
    """a crude rotation alignment assuming that current omega value allows
    for visualization through sheath opening"""

    def move_centroid_inner(n_steps):
        for _ in range(0, n_steps):
            delta_x, delta_y = yield from measure_opening_dist(detector, roi)
            yield from bps.mvr(x_axis, delta_x)
            yield from bps.sleep(0.5)
            yield from bps.mvr(y_axis, -delta_y)

    yield from move_centroid_inner(n_steps)
    yield from bps.mvr(rot_axis, 90)
    yield from move_centroid_inner(n_steps)
    yield from bps.mvr(rot_axis, -90)


def pin_focus_scan(detector, axis):
    """use an axis perpendicular to camera image to adjust focus, determine
    optimally focused position with canny edge detection. The geometry assumes
    that pin tip is coming from the right, thus argmin used for OpenCV
    convention. Pin tip coming from left would require argmax."""
    yield from bps.abs_set(detector.cam_mode, "edge_detection")
    scan_uid = yield from bp.rel_scan([detector], axis, -400, 400, 80)
    left_pixels = db[scan_uid].table()[f"{detector.cv1.outputs.output7.name}"]
    best_cam_z = db[scan_uid].table()[f"{axis.name}"][left_pixels.argmin()]
    return best_cam_z


def rot_pin_align(
    rot_aligner=rot_aligner, rot_motor=gonio.o, long_motor=gonio.gx
):

    # find optimal omega for sheath opening
    omega_scan_uid = yield from bp.scan(
        [rot_aligner.cam_lo], rot_motor, -10, 100, 20
    )
    omega_scan_df = db[omega_scan_uid].table()
    omega_start = omega_scan_df[f"{rot_motor.name}"][
        omega_scan_df[f"{rot_aligner.cam_lo.stats1.total.name}"].idxmax()
    ]
    yield from bps.mv(rot_motor, omega_start)

    # prepare pin position for alignment
    yield from align_centroid(
        rot_aligner.cam_lo,
        rot_aligner.cam_lo.roi1,
        gonio.gx,
        rot_aligner.gc_positioner.cam_y,
        gonio.o,
    )
    best_z = yield from pin_focus_scan(
        rot_aligner.cam_hi, rot_aligner.gc_positioner.cam_z
    )
    yield from bps.mv(rot_aligner.gc_positioner.cam_z, best_z)

    # move pin tip past ROI
    delta_long = yield from measure_tip_dist(rot_aligner.cam_hi)
    yield from bps.mvr(long_motor, delta_long)

    # first coarse alignment
    delta_y, delta_z, rot_axis_pix = yield from measure_rot_axis(
        rot_aligner.cam_hi, rot_motor, omega_start
    )
    # move to approximate rotation axis
    yield from bps.mvr(rot_aligner.gc_positioner.real_y, -delta_y)
    yield from bps.sleep(0.1)
    yield from bps.mvr(rot_aligner.gc_positioner.real_z, -delta_z)

    # move closer to pin tip for new measurement
    yield from bps.mvr(gonio.gx, -70)

    # second alignment
    delta_y, delta_z, rot_axis_pix = yield from measure_rot_axis(
        rot_aligner.cam_hi, rot_motor, omega_start
    )
    yield from bps.mvr(rot_aligner.gc_positioner.real_y, -delta_y)
    yield from bps.sleep(0.1)
    yield from bps.mvr(rot_aligner.gc_positioner.real_z, -delta_z)

    # third alignment, do not attempt to move, just measure
    _, _, rot_axis_pix = yield from measure_rot_axis(
        rot_aligner.cam_hi, rot_motor, omega_start
    )

    def tune_pin_inner(cam_axis):
        """pixel to um precision is too low to properly position pin tip
        based on calculated move, so scan around calculated rotation axis and
        use linear regression to empirically measure optimal value of
        pseudopositioner axis cam_y
        """
        yield from bps.abs_set(rot_aligner.cam_hi.cam_mode, "rot_align")
        scan_uid = yield from bp.rel_scan(
            [rot_aligner.cam_hi], cam_axis, -5, 5, 20
        )
        scan_df = db[scan_uid].table()
        b = scan_df[f"{rot_aligner.cam_hi.stats4.max_xy.y.name}"]
        A = np.matrix([[pos, 1] for pos in scan_df[f"{cam_axis.name}"]])
        p = (
            np.linalg.inv(A.transpose() * A)
            * A.transpose()
            * np.matrix(b.to_numpy()).transpose()
        )
        best_cam_pos = (rot_axis_pix - p[1]) / p[0]
        return best_cam_pos

    # move pin to calculated rot axis with cam_y axis at omega_start
    best_cam_y = yield from tune_pin_inner(rot_aligner.gc_positioner.cam_y)
    yield from bps.mv(rot_aligner.gc_positioner.cam_y, best_cam_y)

    # ensure pin is centered by repeating previous step at omega_start + 90 deg
    yield from bps.mvr(gonio.o, 90)
    best_cam_y = yield from tune_pin_inner(rot_aligner.gc_positioner.cam_y)
    yield from bps.mv(rot_aligner.gc_positioner.cam_y, best_cam_y)

    # update rotation axis signal, if move is reasonable rois will auto-update
    yield from bps.abs_set(rot_aligner.proposed_rot_axis, rot_axis_pix.item(0))

    # bump pin tip to line up with cross-hair for human result inspector
    yield from bps.mvr(gonio.gx, -28)
