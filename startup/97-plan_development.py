# algorithm development at the beamline


def measure_rot_axis(
    detector, rot_motor, start, *, sweep_width=270, n_steps=4
):
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
    yield from bps.abs_set(detector.cam_mode, "edge_detection")
    scan_uid = yield from bp.count([detector], 1)
    left_pixel = db[scan_uid].table()[f"{detector.cv1.outputs.output7.name}"][
        1
    ]
    delta_x_pix = left_pixel - coarse_align_pix
    return delta_x_pix / detector.pix_per_um.get()


def rot_pin_align(rot_aligner, rot_motor, long_motor):

    LAST_X = 5100
    LAST_Y = -450
    LAST_Z = 200

    # move to last centered position
    yield from bps.mv(rot_aligner.gc_positioner.real_y, LAST_Y)
    yield from bps.mv(rot_aligner.gc_positioner.real_z, LAST_Z)
    yield from bps.mv(long_motor, LAST_X)

    # find optimal omega for sheath opening
    omega_scan_uid = yield from bp.scan(
        [rot_aligner.cam_lo], rot_motor, -10, 100, 20
    )
    omega_scan_df = db[omega_scan_uid].table()
    omega_start = omega_scan_df[f"{rot_motor.name}"][
        omega_scan_df[f"{rot_aligner.cam_lo.stats1.total.name}"].idxmax()
    ]
    yield from bps.mv(rot_motor, omega_start)

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
    yield from bps.mvr(gonio.gx, -55)

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

    best_cam_y = yield from tune_pin_inner(rot_aligner.gc_positioner.cam_y)
    yield from bps.mv(rot_aligner.gc_positioner.cam_y, best_cam_y)
    yield from bps.mvr(gonio.o, 90)
    best_cam_y = yield from tune_pin_inner(rot_aligner.gc_positioner.cam_y)
    yield from bps.mv(rot_aligner.gc_positioner.cam_y, best_cam_y)

    yield from bps.abs_set(rot_aligner.proposed_rot_axis, rot_axis_pix)
