from bluesky import plans as bp

def mirror_scan(mir, start, end, steps, gap=None, speed=None, camera=None):
    """Scans a slit aperture center over a mirror against a camera

    Parameters
    ----------

    mir: str
        One of "kbh" or "kbv". This is what is scanned:

         mirror |    slits    |    camera
        --------+-------------+--------------
          kbh   | slt:2 horiz |   Low Mag
          kbv   | slt:2 vert  |   Low Mag

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
            'camera': cam_6, # Lo-Mag
            'encoder_idx': 2,
        },
        'kbv': {
            'name': "Vertical KB Mirror",
            'zebra': zebra1,
            'slt_minus': slits2.b,
            'slt_ctr': slits2.y_ctr,
            'slt_gap': slits2.y_gap,
            'camera': cam_6, # Lo-mag
            'encoder_idx': 3,
        },
    }

    m = mirrors[mir]
    name        = m['name']
    zebra       = m['zebra']
    slt_minus   = m['slt_minus']
    slt_ctr     = m['slt_ctr']
    slt_gap     = m['slt_gap']
    cam         = camera.cam if camera else m['camera'].cam
    stats       = camera.stats4 if camera else m['camera'].stats4
    encoder_idx = m['encoder_idx']

    # Calculate parameters
    abs_move = abs(end - start)
    move_slack = abs_move*0.02

    requested_time = cam.acquire_time.value*steps
    time_slack = requested_time
    total_time = requested_time + time_slack

    gap = slt_gap.position if gap is None else gap
    minus_start = start - gap / 2
    minus_end = end + gap / 2

    if speed is None:
        fps = cam.array_rate.value
        if fps:
            speed = 0.9*abs_move*fps/steps
        else:
            speed = 15

    print("speed:", speed, "um/s")

    encoders = [False]*4
    encoders[encoder_idx] = True

    zebra.setup(
        master=encoder_idx,
        arm_source=0, # soft
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

            self._data_sources = (self._centroid_x, self._centroid_y, self._enc)

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
                    'data': { sig.name: data[sig][i] for sig in data },
                    'timestamps': { sig.name: timestamps[i] for sig in data },
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

    @subs_decorator([lp1, lp2, LiveTable([y1, y2])])
    @reset_positions_decorator([cam.acquire, cam.trigger_mode, slt_ctr, slt_gap,
                                stats.enable, stats.compute_centroid])
    @reset_positions_decorator([slt_ctr.velocity]) # slt_ctr.velocity has to be restored before slt_ctr
    @run_decorator()
    def inner():
        # Prepare statistics plugin
        yield from bp.mv(
            stats.enable, 1,
            stats.compute_centroid, 1
        )

        # Prepare Camera
        yield from bp.mv(cam.acquire, 0)      # Stop camera...
        yield from bp.sleep(.5)               # ...and wait for the pipeline to empty.
        yield from bp.mv(
            cam.trigger_mode, "Sync In 1",    # External Trigger
            cam.array_counter, 0,
        )
        yield from bp.abs_set(cam.acquire, 1) # wait=False

        # Move to the starting positions
        yield from bp.mv(
            slt_gap, gap,                     # Move gap to desired position
            slt_ctr, start - move_slack,      # Move slits to the beginning of the motion
            stats.ts_control, "Erase/Start",  # Prepare statistics Time Series
        )

        # Set Slits Center velocity for the scan
        yield from bp.mv(slt_ctr.velocity, speed)

        # Go
        yield from bp.kickoff(flyer, wait=True)
        st = yield from bp.complete(flyer)
        yield from bp.abs_set(slt_ctr, end + move_slack)

        while not st.done:
            yield from bp.collect(flyer, stream=True)
            RE._uncollected.add(flyer)        # TODO: This is a hideous hack until the next bluesky version. Remove this line
            yield from bp.sleep(0.5)

        yield from bp.sleep(1)
        yield from bp.collect(flyer, stream=True)

        yield from bp.mv(stats.ts_control, "Stop")

    yield from inner()

