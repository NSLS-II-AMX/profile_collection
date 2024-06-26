import os
from bluesky.callbacks import LiveTable, LivePlot
import bluesky.preprocessors as bpp
import bluesky.plan_stubs as bps
import bluesky.plans as bp
import epics
import numpy as np
from math import sin, cos, radians
print(f"Loading {__file__}")


def simple_ascan(camera, stats, motor, start, end, steps):
    """ Simple absolute scan of a single motor against a single camera.

    Automatically plots the results.
    """

    stats_name = "_".join((camera.name, stats)) if stats else camera.name
    try:
        motor_name = motor.readback.name
    except AttributeError:
        motor_name = motor.name

    # @bpp.subs_decorator([LivePlot(stats_name, motor_name), LiveTable([motor_name, stats_name])])
    @bpp.reset_positions_decorator([motor])
    def inner():
        yield from bp.scan([camera], motor, start, end, steps)

    yield from inner()


def mirror_scan(mir, start, end, steps, gap=None, speed=None, camera=None):
    """Scans a slit aperture center over a mirror against a camera

    Parameters
    ----------

    mir: str
        One of "kbh" or "kbv". This is what is scanned:

         mirror |    slits    |    camera
        --------+-------------+--------------
          kbh   | slt:2 horiz |   High Mag
          kbv   | slt:2 vert  |   High Mag

    start: float
        The starting position (um) of the aperture center

    end: float
        The ending position (um) of the aperture center

    steps: int
        The number of steps (number of points) to take

    speed: float (default=None)
        The speed (um/s) with which to move the aperture. If `None`, this
        scan will try to calculate the maximum theoretical speed based on
        the current frame rate of the camera. Failing that, the speed will
        be arbitrarily set to 15 um/s.

    gap: float (default=None)
        The size of the gap in um. If `None`, the current gap will be used.

    camera: camera object (default=None)
        The camera to be used in this scan. If `None`, the camera listed
        in the table above will be used depending on the selected mirror.

    """
    mirrors = {
        'kbh': {
            'name': "Horizontal KB Mirror",
            'zebra': zebra1,
            'slt_minus': slits2.i,
            'slt_ctr': slits2.x_ctr,
            'slt_gap': slits2.x_gap,
            'camera': cam_7,  # Hi-Mag
            'encoder_idx': 2,
        },
        'kbv': {
            'name': "Vertical KB Mirror",
            'zebra': zebra1,
            'slt_minus': slits2.b,
            'slt_ctr': slits2.y_ctr,
            'slt_gap': slits2.y_gap,
            'camera': cam_7,  # Hi-mag
            'encoder_idx': 3,
        },
    }

    m = mirrors[mir]
    name = m['name']
    zebra = m['zebra']
    slt_minus = m['slt_minus']
    slt_ctr = m['slt_ctr']
    slt_gap = m['slt_gap']
    cam = camera.cam if camera else m['camera'].cam
    stats = camera.stats4 if camera else m['camera'].stats4
    encoder_idx = m['encoder_idx']

    # Calculate parameters
    abs_move = abs(end - start)
    move_slack = abs_move*0.02

    requested_time = cam.acquire_time.get() * steps
    time_slack = requested_time
    total_time = requested_time + time_slack

    gap = slt_gap.position if gap is None else gap
    minus_start = start - gap / 2
    minus_end = end + gap / 2

    if speed is None:
        fps = cam.array_rate.get()
        if fps:
            speed = 0.9*abs_move*fps/steps
        else:
            speed = 15

    print("speed:", speed, "um/s")

    encoders = [False]*4
    encoders[encoder_idx] = True

    zebra.setup(
        master=encoder_idx,
        arm_source=0,  # soft
        gate_start=minus_start,
        gate_width=abs_move/steps/2,
        gate_step=abs_move/steps,
        num_gates=steps,
        direction=int(start > end),

        # Pulse configuration is irrelevant
        # Pulse width must be less than pulse step
        pulse_width=0.5,
        pulse_step=1,
        capt_delay=0,
        max_pulses=1,

        # Only collect the relevant encoder
        collect=encoders
    )

    class CustomFlyer(Device):
        def __init__(self, *args, **kwargs):
            self._last_point = 0
            self._collection_ts = None

            self._ts = zebra.pos_capt.data.time
            self._centroid_x = stats.ts_centroid.x
            self._centroid_y = stats.ts_centroid.y
            self._enc = getattr(zebra.pos_capt.data, f'enc{encoder_idx+1}')

            self._data_sources = (
                self._centroid_x, self._centroid_y, self._enc)

            super().__init__(*args, **kwargs)

        def kickoff(self):
            self._collection_ts = time.time()
            return zebra.kickoff()

        def complete(self):
            return zebra.complete()

        def collect(self):
            data = {
                sig: sig.get(use_monitor=False) for sig in self._data_sources
            }

            timestamps = self._ts.get(use_monitor=False) + self._collection_ts

            min_len = min([len(d) for d in data.values()])
            cur_time = time.time()

            for i in range(self._last_point, min_len):
                yield {
                    'data': {sig.name: data[sig][i] for sig in data},
                    'timestamps': {sig.name: timestamps[i] for sig in data},
                    'time': cur_time
                }

            self._last_point = min_len

        def describe_collect(self):
            return {
                'primary': {
                    sig.name: {
                        'source': 'PV:' + sig.pvname,
                        'shape': [],
                        'dtype': 'number'
                    } for sig in self._data_sources
                }
            }

    flyer = CustomFlyer('', name='flyer')

    # Setup plot
    y1 = stats.ts_centroid.x.name
    y2 = stats.ts_centroid.y.name
    x = getattr(zebra.pos_capt.data, f'enc{encoder_idx+1}').name

    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()

    lp1 = LivePlot(y1, x, ax=ax1, color='r')
    lp2 = LivePlot(y2, x, ax=ax2, color='b')

    # Set axes labels after creating LivePlots
    ax1.set_title(name)
    ax1.set_xlabel('Center Position')
    ax1.set_ylabel('Centroid X', color='r')
    ax2.set_ylabel('Centroid Y', color='b')

    @bpp.subs_decorator([lp1, lp2, LiveTable([y1, y2])])
    @bpp.reset_positions_decorator([cam.acquire, cam.trigger_mode, slt_gap,  # slt_ctr, <- this fails with FailedStatus
                                    stats.enable, stats.compute_centroid])
    # slt_ctr.velocity has to be restored before slt_ctr
    @bpp.reset_positions_decorator([slt_ctr.velocity])
    @bpp.run_decorator()
    def inner():
        # Prepare statistics plugin
        yield from bps.mv(
            stats.enable, 1,
            stats.compute_centroid, 1
        )

        # Prepare Camera
        yield from bps.mv(cam.acquire, 0)      # Stop camera...
        # ...and wait for the pipeline to empty.
        yield from bps.sleep(.5)
        yield from bps.mv(
            cam.trigger_mode, "Sync In 1",    # External Trigger
            cam.array_counter, 0,
        )
        yield from bps.abs_set(cam.acquire, 1)  # wait=False

        # Move to the starting positions
        yield from bps.mv(
            slt_gap, gap,                     # Move gap to desired position
            slt_ctr, start - move_slack,      # Move slits to the beginning of the motion
            stats.ts_control, "Erase/Start",  # Prepare statistics Time Series
        )

        # Set Slits Center velocity for the scan
        yield from bps.mv(slt_ctr.velocity, speed)

        # Go
        yield from bps.kickoff(flyer, wait=True)
        st = yield from bps.complete(flyer)
        yield from bps.abs_set(slt_ctr, end + move_slack)

        while not st.done:
            yield from bps.collect(flyer, stream=True)
            # RE._uncollected.add(flyer)        # TODO: This is a hideous hack until the next bluesky version. Remove this line
            yield from bps.sleep(0.5)

        yield from bps.sleep(1)
        yield from bps.collect(flyer, stream=True)

        yield from bps.mv(stats.ts_control, "Stop")

    yield from inner()


def focus_scan(steps, step_size=2, speed=None, cam=cam_6, filename='test', folder='/tmp/', use_roi4=False):
    """ Scans a sample along Z against a camera, taking pictures in the process.

    Parameters
    ----------

    steps: int
        The number of steps (number of points) to take

    step_size: float
        The size of each step (um). Default: 2 um

    speed: float (default=None)
        The speed (um/s) with which to move. If `None`, this
        scan will try to calculate the maximum theoretical speed based on
        the current frame rate of the camera. Failing that, the speed will
        be arbitrarily set to 15 um/s.

    cam: camera object (default=cam_6 (Low Mag))
        The camera that will take the pictures.

    filename: str
        The file name for the acquired pictures. Default: 'test'

    folder: str
        The folder where to write the images to. Default: '/tmp'

    use_roi4: bool
        If True, temporarily set the camera ROI to the same dimensions as the ROI4
        plugin during the acquisition. Default: False
    """
    if folder[-1] != '/':
        folder += '/'

    # Devices
    py = gonio.py
    pz = gonio.pz
    zebra = zebra2
    tiff = cam.tiff
    roi = cam.roi4
    cam = cam.cam

    # Calculate parameters
    total_move = steps*step_size
    move_slack = total_move*0.02

    if speed is None:
        fps = cam.array_rate.get()
        if fps:
            speed = 0.9*total_move*fps/steps
        else:
            speed = 15

    print("speed:", speed, "um/s")

    omega = gonio.o.user_setpoint.get()

    def calc_params(mtr):
        f = sin(radians(-omega)) if mtr == py else cos(radians(omega))
        cur = mtr.position
        start = cur - f*total_move/2
        end = cur + f*total_move/2
        total = abs(end-start)
        slack = move_slack*f if f else 0
        spd = abs(speed*f) if f else 0
        return start, end, total, slack, spd

    start_y, end_y, total_y, slack_y, speed_y = calc_params(py)
    start_z, end_z, total_z, slack_z, speed_z = calc_params(pz)

    # Choose master motor
    if(total_y > total_z):
        start, end, total, encoder_idx = start_y, end_y, total_y, 1
    else:
        start, end, total, encoder_idx = start_z, end_z, total_z, 2

    print("Master motor:", {1: "y", 2: "z"}[encoder_idx])

    zebra.setup(
        master=encoder_idx,
        arm_source=0,  # soft
        gate_start=start,
        gate_width=total/steps/2,
        gate_step=total/steps,
        num_gates=steps,
        direction=int(start > end),

        # Pulse configuration is irrelevant
        # Pulse width must be less than pulse step
        pulse_width=0.5,
        pulse_step=1,
        capt_delay=0,
        max_pulses=1,

        # Only collect PY and PZ
        collect=[False, True, True, False]
    )

    @bpp.reset_positions_decorator([cam.acquire, cam.trigger_mode, cam.min_x, cam.min_y,
                                    cam.size.size_x, cam.size.size_y, gonio.py, gonio.pz,
                                    tiff.file_write_mode, tiff.num_capture, tiff.auto_save,
                                    tiff.auto_increment, tiff.file_path, tiff.file_name,
                                    tiff.file_number, tiff.enable])
    @bpp.reset_positions_decorator([gonio.py.velocity, gonio.pz.velocity])
    @bpp.run_decorator()
    def inner():
        # Prepare Camera
        yield from bps.mv(cam.acquire, 0)      # Stop camera...
        # ...and wait for the pipeline to empty.
        yield from bps.sleep(.5)
        yield from bps.mv(
            cam.trigger_mode, "Sync In 1",    # External Trigger
            cam.array_counter, 0,
        )

        if use_roi4:
            yield from bps.mv(
                cam.min_x, roi.min_xyz.min_x.get(),
                cam.min_y, roi.min_xyz.min_y.get(),
                cam.size.size_x, roi.size.x.get(),
                cam.size.size_y, roi.size.y.get()
            )

        # Prepare TIFF Plugin
        yield from bps.mv(
            tiff.file_write_mode, "Stream",
            tiff.num_capture, steps,
            tiff.auto_save, 1,
            tiff.auto_increment, 1,
            tiff.file_path, folder,
            tiff.file_name, filename,
            tiff.file_template, "%s%s_%d.tif",
            tiff.file_number, 1,
            tiff.enable, 1)

        yield from bps.abs_set(tiff.capture, 1)

        yield from bps.abs_set(cam.acquire, 1)  # wait=False

        # Move to the starting positions
        yield from bps.mv(
            gonio.py, start_y - slack_y,
            gonio.pz, start_z - slack_z,
        )

        # Set velocity for the scan
        yield from bps.mv(
            gonio.py.velocity, speed_y,
            gonio.pz.velocity, speed_z
        )

        # Arm Zebra
        yield from bps.abs_set(zebra.pos_capt.arm.arm, 1)

        # Wait Zebra armed
        while not zebra2.download_status.get():
            time.sleep(0.1)

        # Go
        yield from bps.mv(
            gonio.py, end_y + slack_y,
            gonio.pz, end_z + slack_z
        )

        yield from bps.abs_set(tiff.capture, 0)

        print(f"{cam.array_counter.get()} images captured")

    yield from inner()


def find_peak(det, mot, start, stop, steps):
    print(f"Scanning {mot.name} vs {det.name}...")

    uid = yield from bp.rel_scan([det], mot, start, stop, steps)
    sp = '_setpoint' if mot is ivu_gap else '_user_setpoint'

    if det.name == "bpm3":
        detector_response_name = det.name + '_sum_all'
    else:
        detector_response_name = det.name

    data = np.array(
        db[uid].table()[[detector_response_name, mot.name + sp]])[1:]

    peak_idx = np.argmax(data[:, 0])
    peak_x = data[peak_idx, 1]
    peak_y = data[peak_idx, 0]

    if mot is ivu_gap:
        m = ivu_gap.gap
    else:
        m = mot
    print(
        f"Found peak for {m.name} at {peak_x} {m.egu} [BPM reading {peak_y}]")
    return peak_x, peak_y


@bpp.reset_positions_decorator([vdcm.p.SPMG, vdcm_hold_pitch, gov_rbt])
def set_energy(energy, use_diode=True):

    yield from bps.abs_set(gov_rbt, 'FM', wait=True)
    yield from bps.abs_set(vdcm_hold_pitch, 0)

    #bpm = bpm3
    if use_diode:
        detector_ = keithley
    else:
        detector_ = bpm3

    # Values on 2017-09-20 adjusted on march 18 2019
    energies = [9500, 12000, 13475, 15000, 18000]

    # LookUp Tables
    LUT = {
        ivu_gap: (energies, np.array([7.642, 6.483, 7.040, 6.473, 6.465])*1000),
        vdcm.g: (energies, [14.93, 14.83, 14.79, 14.75, 14.67]),
        vdcm.r:  (energies, [5.891, 5.840, 5.810, 5.769, 5.690]),
        vdcm.p: (energies, [6.3305, 6.3262, 6.3245, 6.3214, 6.3185])
    }

    # Lookup Table
    def lut(motor):
        return motor, np.interp(energy, *LUT[motor])

    yield from bps.mv(vdcm.p.SPMG, 3)

    yield from bps.mv(
        *lut(ivu_gap),   # Set IVU Gap interpolated position
        vdcm.e, energy,  # Set Bragg Energy pseudomotor
        *lut(vdcm.g),    # Set DCM Gap interpolated position
        *lut(vdcm.r),     # Set DCM Roll interpolated position
        *lut(vdcm.p),     # Set Pitch to last known good position
    )

    # Setup plots
    ax1 = plt.subplot(311)
    ax1.grid(True)
    ax2 = plt.subplot(312)
    ax2.grid(True)
    ax3 = plt.subplot(313)
    plt.tight_layout()

    # Decorate find_peaks to play along with our plot and plot the peak location
    def find_peak_inner(detector, motor, start, stop, num, ax):

        if detector.name == "bpm3":
            det_name = detector.name + '_sum_all'
        else:
            det_name = detector.name

        mot_name = motor.name + '_setpoint' if motor is ivu_gap else motor.name + '_user_setpoint'

        # Prevent going below the lower limit or above the high limit
        if motor is ivu_gap:
            step_size = (stop - start) / (num - 1)
            while ivu_gap.gap.user_setpoint.get() + start < ivu_gap.gap.low_limit:
                start += 5 * step_size
                stop += 5 * step_size

            while ivu_gap.gap.user_setpoint.get() + stop > ivu_gap.gap.high_limit:
                start -= 5 * step_size
                stop -= 5 * step_size

        @bpp.subs_decorator(LivePlot(det_name, mot_name, ax=ax))
        def inner():
            peak_x, peak_y = yield from find_peak(detector, motor, start, stop, num)
            ax.plot([peak_x], [peak_y], 'or')
            return peak_x, peak_y
        return inner()

    # Scan DCM Pitch
    peak_x, peak_y = yield from find_peak_inner(detector_, vdcm.p, -0.02, 0.02, 41, ax1)
    yield from bps.mv(vdcm.p, peak_x)
    yield from bps.abs_set(vdcm_hold_pitch_here, 1, wait=True)

    # Scan IVU Gap
    peak_x, peak_y = yield from find_peak_inner(detector_, ivu_gap, -40, 40, 21, ax2)
    yield from bps.mv(ivu_gap, peak_x)

    # TODO: DAMA must fix this on the next visit (no cagets!).
    # Get image
    prefix = 'XF:17IDA-BI:AMX{FS:2-Cam:1}image1:'
    image = epics.caget(prefix+'ArrayData')
    width = epics.caget(prefix+'ArraySize0_RBV')
    height = epics.caget(prefix+'ArraySize1_RBV')
    ax3.imshow(image.reshape(height, width), cmap='jet')


def vdcm_rock_test(vdcm_p_range=0.03, vdcm_p_points=61, logging=True):
    """
    Scan vdcm crystal 2 pitch to maximize flux on BPM1

    Optional arguments:
    vdcm_p_range: vdcm rocking curve range [mrad]. Default 0.03 mrad
    vdcm_p_points: vdcm rocking curve points. Default 51

    Example:
    RE(vdcm_rock())
    RE(vdcm_rock(vdcm_p_range=0.035, vdcm_p_points=71))
    """

    yield from bps.mv(
        vdcm.p, 6.5040     # Set Pitch interpolated position
    )

    ax1 = plt.subplot(111)
    # Setup plots
    ax1.grid(True)
    # plt.tight_layout()

    # Decorate find_peaks to play along with our plot and plot the peak location
    def find_peak_inner(detector, motor, start, stop, num, ax):
        det_name = detector.name+'_sum_all'
        mot_name = motor.name+'_user_setpoint'

        @bpp.subs_decorator(LivePlot(det_name, mot_name, ax=ax))
        def inner():
            peak_x, peak_y = yield from find_peak(detector, motor, start, stop, num)
            ax.plot([peak_x], [peak_y], 'or')
            return peak_x, peak_y
        return inner()

    # Scan DCM Pitch
    peak_x, peak_y = yield from find_peak_inner(bpm3, vdcm.p, -vdcm_p_range, vdcm_p_range, vdcm_p_points, ax1)
    yield from bps.mv(vdcm.p, peak_x)

    if logging:
        print('BPM3 sum = {:.4g} A'.format(bpm3.sum_all.get()))
