from ophyd import PseudoPositioner, PseudoSingle, Staged
from ophyd.pseudopos import pseudo_position_argument, real_position_argument


class GonioCameraPositioner(PseudoPositioner):
    """move gonio fine stage with camera basis for any omega

    omega=0 degrees:
        pinY: vertical, parallel with on-axis camera image
        pinZ: downstream, perpendicular to on-axis camera image

    omega=90 degrees
        pinY: upstream, perpendicular to on-axis camera image
        pinZ: vertical, parallel with on-axis camera image

    2D rotation matrix:
        [cos(w) -sin(w); sin(w) cos(w)]

    inverse rotation:
        [cos(w) sin(w); -sin(w) cos(w)]

    """

    # pseudo axes
    cam_y = Cpt(PseudoSingle, limits=(-1000, 1000))
    cam_z = Cpt(PseudoSingle, limits=(-1000, 1000))

    # real axes
    real_y = Cpt(EpicsMotor, "-Ax:PY}Mtr")
    real_z = Cpt(EpicsMotor, "-Ax:PZ}Mtr")

    # configuration value that determine change of basis matrix
    omega = Cpt(EpicsSignalRO, "-Ax:O}Mtr.RBV")

    def __init__(self, *args, concurrent=False, **kwargs):
        """PinY, PinZ smaract stages can jam up if moved simultaneously,
        therefore move in series"""
        super().__init__(*args, **kwargs)
        self.omega.subscribe(self.update_parameters)

    def update_parameters(self, *args, **kwargs):
        [_.sync() for _ in self._pseudo]

    def update_origin(self):
        self._y0.set(self.real_y.user_readback.get())
        self._z0.set(self.real_z.user_readback.get())

    def unstage(self):

        self.log.debug("Unstaging %s", self.name)
        self._staged = Staged.partially
        devices_unstaged = []

        # Call unstage() on child devices.
        for attr in self._sub_devices[::-1]:
            device = getattr(self, attr)
            if hasattr(device, "unstage"):
                device.unstage()
                devices_unstaged.append(device)

        # Restore original values.
        for sig, val in reversed(list(self._original_vals.items())):
            self.log.debug("Setting %s back to its original value: %r", sig.name, val)
            sig.set(val, settle_time=0.1).wait()
            self._original_vals.pop(sig)
        devices_unstaged.append(self)

        self._staged = Staged.no
        return devices_unstaged

    @pseudo_position_argument
    def forward(self, pos):
        """pseudo -> real, motor I/O in degrees"""
        d = np.pi / 180
        omega = self.omega.get()
        return self.RealPosition(
            real_y=(pos.cam_y * np.cos(omega * d) - pos.cam_z * np.sin(omega * d)),
            real_z=(pos.cam_y * np.sin(omega * d) + pos.cam_z * np.cos(omega * d)),
        )

    @real_position_argument
    def inverse(self, pos):
        """real -> pseudo, motor I/O in degrees"""
        d = np.pi / 180
        omega = self.omega.get()
        return self.PseudoPosition(
            cam_y=(pos.real_y) * np.cos(omega * d) + (pos.real_z) * np.sin(omega * d),
            cam_z=-(pos.real_y) * np.sin(omega * d) + (pos.real_z) * np.cos(omega * d),
        )


gcp3 = GonioCameraPositioner("XF:17IDB-ES:AMX{Gon:1", name="gcp")

