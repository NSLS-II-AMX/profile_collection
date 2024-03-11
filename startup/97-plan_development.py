"""bluesky plans for aligning a rotation alignment pin

Some assumptions of alignment pin hardware are inevitiably built in. Here,
we measure the vertical position of a rotation axis using a tapered alignment
pin with a protective sheath, then the pin is moved to that position and ROIs
are updated so that the center of the ROIs coincide with the rotation axis.

No assumptions or updates are made to horizontal positions."""


from functools import partial
print(f"Loading {__file__}")

alignment_pins = {
    'pin_1': {'general_puck_pos': 1, 'start': (5760, 961, -216)},
    'pin_7': {'general_puck_pos': 7, 'start': (5595, 600, -240)}
}


def five_per_step(detectors, step, pos_cache):
    """A per_step hook for acquiring multiple images when we cannot alter
    lighting conditions. Noise from the cryo-stream during edge detection steps
    is minimized by taking multiple measurements (instead of a long exposure).
    Uses the signature f(detectors, step, pos_cache) required by bluesky.

    Parameters
    ----------
    detectors : ophyd Device
        Devices used by outer scan.
    step : OrderedDict
        Motors as keys, values are trajectory of motor for a given step.
    pos_cache : Deprecated

    Yields
    ------

    """

    motor = list(step.keys())[0]  # assume 1d scan
    yield from bps.mv(motor, step[motor])
    yield from bps.repeat(
        partial(bps.trigger_and_read,
                (list(detectors) + list(step.keys()))),
        num=5,
    )


def measure_rot_axis(
    detector,
    signal,
    mode,
    rot_motor,
    start,
    *,
    filter_signal={},
    sweep_width=270,
    n_steps=4,
    per_step=None,
    md=None
):
    """make n_steps number of measurements and perform linear regression to
    calculate the vertical position of the rotation axis in camera coordinates
    in addition to the delta_y, delta_z offsets

    y0 = delta_y_pix*cos(omega) + delta_z_pix*sin(omega)

    y0: vertical position of rotation axis in camera pixels
    delta_y_pix: vertical displacement of object from y0 at omega=0
    delta_z_pix: vertical displacement of object from y0 at omega=90

    Parameters
    ----------
    detector: ophyd Device
        Signals to calculate rotation axis will be read from this device.
        Assumes normal to camera sensor perpendicular to rot axis.
    signal: ophyd Signal
        Signal from ADPlugin for calculating rotation axis.
    mode: String
        Camera mode for area detector device, e.g. "edge_detection".
    rot_motor: ophyd EpicsMotor
        Rotation axis motor.
    start: float
        Starting omega angle of rotation axis (degrees).
    filter_signal: dict
        Dictionary with key ophyd signal and value filtering criterium. This
        parameter is used to exclude outliers from regression.
    sweep_width: float
        Angular range of rotation scan (degrees).
    n_steps: int
        Number of steps visited in scan, should match number of openings in pin
        cover ideally.
    per_step: func
        Hook for injecting custom trigger/read methods into scan.
    """

    _md = md or {}

    yield from bps.abs_set(detector.cam_mode, mode, wait=True)

    omega_list = np.concatenate(
        [np.linspace(start + k, start + sweep_width + k, n_steps)
         for k in (-10, 0, 10)]
    )
    omega_list.sort()
    omega_list = list(omega_list)

    try:
        scan_uid = yield from bp.list_scan(
            [detector],
            rot_motor,
            omega_list,
            per_step=per_step,
            md=_md
        )
    except WaitTimeoutError as error:
        print(f"caught {error} in measure_rot_axis")
        yield from bps.abs_set(detector.cam.acquire, 0, wait=True)
        scan_uid = yield from bp.list_scan(
            [detector],
            rot_motor,
            omega_list,
            per_step=per_step,
            md=_md
        )

    # apply signal filter to scan data
    df = db[scan_uid].table()
    for sig in filter_signal.keys():
        df = df[df[sig.name] > filter_signal[sig]]
        break

    b = df[signal.name]
    A = np.matrix(
        [
            [np.cos(np.deg2rad(omega)), np.sin(np.deg2rad(omega)), 1]
            for omega in df[rot_motor.name]
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
    """Use canny edge detection to calculate current distance in um of
    alignment pin tip from arbitrary camera position.

    Parameters
    ----------
    detector: ophyd Device
        Detector for reading tip distance in coordinates.
    coarse_align_pix: int
        pixel coordinate for calculating step size."""

    yield from bps.abs_set(detector.cam_mode, "edge_detection")
    scan_uid = yield from bp.count([detector], 1)
    left_pixel = db[scan_uid].table()[detector.cv1.outputs.output7.name][1]
    delta_x_pix = left_pixel - coarse_align_pix
    return delta_x_pix / detector.pix_per_um.get()


def measure_opening_dist(detector, roi):
    """use openCV ADPlugin to detect largest centroid and make best effort
    to calculate the stage movements required to center of roi. Omega may be
    at an arbitrary position so movements must be made using camera axes"""
    yield from bps.abs_set(detector.cam_mode, "centroid")
    scan_uid = yield from bp.count([detector], 1)
    centroid_x = db[scan_uid].table()[detector.cv1.outputs.output1.name][1]
    centroid_y = db[scan_uid].table()[detector.cv1.outputs.output2.name][1]
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


def rot_pin_align_cleanup():
    """Safety measure in case jpeg plugin stuck enabled. For some reason plugin
    write mode must be "Single" for bluesky/ophyd, if we leave this signal as a
    stage_sig and file write mode is not "Single", e.g. "Capture" or "Stream" this
    affects some timing and results in a mismatched resource name. The wrong
    filename ends up in the corresponding event document. Some experimenting
    shows that if we leave write mode as "Single" we can set auto save outside
    of staging to obtain the expected behavior. Similarly we set auto_save in 
    the plan and then reset it back to a safe default (off) here."""

    yield from bps.abs_set(rot_aligner.cam_hi.jpeg.file_write_mode, 0)
    yield from bps.abs_set(rot_aligner.cam_hi.jpeg.enable, 0)
    yield from bps.abs_set(rot_aligner.cam_hi.jpeg.auto_save, 0)


@finalize_decorator(rot_pin_align_cleanup)
def rot_pin_align(
    rot_aligner=rot_aligner,
    rot_motor=gonio.o,
    long_motor=gonio.gx,
    start=alignment_pins['pin_7']['start'],
    pin=alignment_pins['pin_7']['general_puck_pos']
):
    """A bluesky plan for aligning a rotation alignment pin and calculating
    the (horizontal) rotation axis. The plan performs several increasingly
    accurate rotation axis measurements using the ADCompVision area detector
    plugin.

    Parameters
    ----------
    rot_aligner : ophyd RotationAxisAligner
        Custom ophyd device for measuring the rotation axis of a goniometer
        with a horizontal rotation axis and two on-axis cameras.
    rot_motor : ophyd EpicsMotor
        Rotation motor (horizontal).
    long_motor : ophyd EpicsMotor
        Longitudinal motor for horizontal translations along rotation axis.
        The default is gonio.gx.
    start : tuple
        x, y, z position of last good rotation axis

    Returns
    -------
    None.
    """

    _md = {'plan_group': 'rot_pin_align',
           'plan_group_id': str(uuid.uuid4()),
           'alignment_pin': pin}

    if gov_rbt.state.get() == 'M':
        print('found governor in M, awaiting sentinel recovery')
        yield from bps.sleep(15)
    yield from bps.mv(gov_rbt, 'PA')

    # move to last good gonio xyz values
    yield from bps.mv(rot_aligner.gc_positioner.real_y, start[1])
    yield from bps.mv(rot_aligner.gc_positioner.real_z, start[2])
    yield from bps.mv(long_motor, start[0])

    yield from bps.abs_set(rot_aligner.cam_hi.cam_mode, "rot_align", wait=True)

    # find optimal omega for sheath opening
    omega_scan_uid = yield from bp.scan(
        [rot_aligner.cam_lo], rot_motor, -10, 100, 20, md=_md
    )

    print('done with opening scan')

    omega_scan_df = db[omega_scan_uid].table()
    omega_start = omega_scan_df[rot_motor.name][
        omega_scan_df[rot_aligner.cam_lo.stats1.total.name].idxmax()
    ]
    yield from bps.mv(rot_motor, omega_start)

    # first coarse alignment
    delta_y, delta_z, rot_axis_pix = yield from measure_rot_axis(
        rot_aligner.cam_hi,
        rot_aligner.cam_hi.stats4.max_xy.y,
        "rot_align",
        rot_motor,
        omega_start,
        filter_signal={rot_aligner.cam_hi.stats4.centroid.y: 10},
        md=_md
    )
    # move to approximate rotation axis
    yield from bps.mvr(rot_aligner.gc_positioner.real_y, -delta_y)
    yield from bps.sleep(0.1)
    yield from bps.mvr(rot_aligner.gc_positioner.real_z, -delta_z)
    print('moved after first alignment')

    # move closer to pin tip for new measurement
    # yield from bps.mvr(gonio.gx, -70)

    # second alignment
    delta_y, delta_z, rot_axis_pix = yield from measure_rot_axis(
        rot_aligner.cam_hi,
        rot_aligner.cam_hi.stats4.max_xy.y,
        "rot_align",
        rot_motor,
        omega_start,
        filter_signal={rot_aligner.cam_hi.stats4.centroid.y: 10},
        md=_md
    )

    yield from bps.mvr(rot_aligner.gc_positioner.real_y, -delta_y)
    yield from bps.sleep(0.1)
    yield from bps.mvr(rot_aligner.gc_positioner.real_z, -delta_z)

    print('moved after second alignment')

    # bring tip to center, using openCV contour to define pin tip
    yield from bps.abs_set(rot_aligner.cam_hi.cam_mode, "rot_align_contour")
    try:
        scan_uid = yield from bp.count([rot_aligner.cam_hi], 5, md=_md)
    except (FailedStatus, WaitTimeoutError) as error:
        print(f"caught {error} during horizontal positioning, retrying...")
        scan_uid = yield from bp.count([rot_aligner.cam_hi], 5, md=_md)

    delta_x = (
        rot_aligner.cam_hi.roi1.bin_.x.get()
        * (
            np.mean(
                db[scan_uid].table()[
                    rot_aligner.cam_hi.cv1.outputs.output1.name
                ]
            )
            - 320
        )
        / (rot_aligner.cam_hi.pix_per_um.get())
    )  # scale to account for ROI binning

    yield from bps.mvr(gonio.gx, delta_x)
    # third alignment, do not attempt to move, just measure
    _, _, rot_axis_pix = yield from measure_rot_axis(
        rot_aligner.cam_hi,
        rot_aligner.cam_hi.cv1.outputs.output2,  # contours
        "rot_align_contour",
        rot_motor,
        omega_start,
        per_step=five_per_step,
        md=_md
    )

    print('done with horizontal adjustment')

    def tune_pin_inner(cam_axis):
        """pixel to um precision is too low to properly position pin tip
        based on calculated move, so scan around calculated rotation axis and
        use linear regression to empirically measure optimal value of
        pseudopositioner axis cam_y
        """
        yield from bps.abs_set(
            rot_aligner.cam_hi.cam_mode, "rot_align_contour"
        )
        scan_uid = yield from bp.rel_scan(
            [rot_aligner.cam_hi], cam_axis, -5, 5, 5, per_step=five_per_step, md=_md
        )
        scan_df = db[scan_uid].table()
        b = scan_df[rot_aligner.cam_hi.cv1.outputs.output2.name]
        A = np.matrix([[pos, 1] for pos in scan_df[cam_axis.name]])
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
    # put in reference of entire cam_hi image
    proposed_rot_axis = rot_aligner.cam_hi.roi1.min_xyz.min_y.get() + \
        rot_axis_pix.item(0)
    yield from bps.abs_set(rot_aligner.proposed_rot_axis, proposed_rot_axis)
    print('completed fine adjustments, saving reference images')

    yield from bp.count(
        [gonio.gx, gonio.py, gonio.pz, rot_aligner.current_rot_axis,
            rot_aligner.proposed_rot_axis],
        1, md={'plan_description': 'rotation_reference', **_md})

    print(f'gx: {gonio.gx.user_readback.get()}, '
          f'gy: {gonio.py.user_readback.get()}, '
          f'gz: {gonio.pz.user_readback.get()}')

    # image check
    yield from bps.abs_set(rot_aligner.cam_hi.cam_mode, "align_check", wait=True)
    yield from bps.abs_set(rot_aligner.cam_hi.jpeg.auto_save, 1, wait=True)

    scan_uid = yield from bp.rel_scan([rot_aligner.cam_hi], gonio.o, -90, 180, 4, md=_md)
    rd = [k for k in db[scan_uid].documents('primary') if k[0] == 'resource']
    filename = rd[-1][-1]['resource_kwargs']['filename']
    resource_path = rd[-1][-1]['resource_path']
    template = rd[-1][-1]['resource_kwargs']['template']
    root = rd[-1][-1]['root']
    full_resource_path = root + resource_path + '/'

    for fp, omega, dt in zip(
        [template % (full_resource_path, filename, k-1)  # account for dataframe shift
         for k in db[scan_uid].table().index],
        db[scan_uid].table()[gonio.o.name],
        db[scan_uid].table()['time']
    ):

        add_cross(fp)
        add_text_bottom_left(fp, f'{omega} deg.', f'{dt}')


def rot_pin_align_with_robot(pin='pin_7', timeout=100000):

    if pin not in alignment_pins.keys():
        raise Exception('alignment pin is not defined, use known pin')

    general_puck_pos = alignment_pins[pin]['general_puck_pos']
    st = alignment_pins[pin]['start']

    yield from bps.abs_set(gov_rbt, "SE", wait=True)

    print(f"mounting alignment {pin}")
    robrob.mount_special(general_puck_pos, timeout)
    yield from bps.abs_set(gov_rbt, "PA", wait=True)
    yield from rot_pin_align(start=st)
    yield from bps.abs_set(gov_rbt, "SE", wait=True)
    robrob.unmount_special(general_puck_pos, timeout)


def compare_plans():
    """A test function for comparing precision and accuracy when using
    ADCompVision or ADStats plugins."""

    for i in range(0, 10):
        yield from measure_rot_axis(
            rot_aligner.cam_hi,
            rot_aligner.cam_hi.stats4.max_xy.y,
            "rot_align",
            gonio.o,
            94,
            per_step=five_per_step,
        )

    for i in range(0, 10):
        yield from measure_rot_axis(
            rot_aligner.cam_hi,
            rot_aligner.cam_hi.cv1.outputs.output2,
            "rot_align_contour",
            gonio.o,
            94,
            per_step=five_per_step,
        )
