from ophyd import (EpicsSignal, EpicsSignalRO, EpicsMotor, Device, Component as Cpt,
                   DeviceStatus)

class PBSignalWithRBV(EpicsSignal):
    # An EPICS signal that uses the NSLS-II convention of 'pvname-SP' being the
    # setpoint and 'pvname-I' being the read-back

    def __init__(self, prefix, **kwargs):
        super().__init__(prefix + '-I', write_pv=prefix + '-SP', **kwargs)


class PowerBrickVectorMotor(Device):
    start = Cpt(PBSignalWithRBV, 'Start')
    end = Cpt(PBSignalWithRBV, 'End')

    def __init__(self, prefix, *, parent=None, **kwargs):
        cfg_attrs = ['start', 'end']
        super().__init__(prefix, configuration_attrs=cfg_attrs,
                         parent=parent, **kwargs)


class PowerBrickVectorBase(Device):
    x = Cpt(PowerBrickVectorMotor, 'Pos:X')
    y = Cpt(PowerBrickVectorMotor, 'Pos:Y')
    z = Cpt(PowerBrickVectorMotor, 'Pos:Z',)
    o = Cpt(PowerBrickVectorMotor, 'Pos:O')

    exposure = Cpt(PBSignalWithRBV, 'Val:Exposure')
    num_samples = Cpt(PBSignalWithRBV, 'Val:NumSamples')

    expose = Cpt(EpicsSignal, 'Expose-Sel')
    hold = Cpt(EpicsSignal, 'Hold-Sel')

    state = Cpt(EpicsSignalRO, 'Sts:State-Sts', auto_monitor=True)
    running = Cpt(EpicsSignalRO, 'Sts:Running-Sts', auto_monitor=True)

    go = Cpt(EpicsSignal, 'Cmd:Go-Cmd')
    proceed = Cpt(EpicsSignal, 'Cmd:Proceed-Cmd')
    abort = Cpt(EpicsSignal, 'Cmd:Abort-Cmd')
    sync = Cpt(EpicsSignal, 'Cmd:Sync-Cmd')

    def __init__(self, prefix, configuration_attrs=None, *args, **kwargs):
        cfg_attrs = ['x', 'y', 'z', 'o', 'exposure', 'num_samples', 'expose', 'hold']
        if configuration_attrs is not None:
            cfg_attrs = configuration_attrs + cfg_attrs
        super().__init__(prefix, configuration_attrs=cfg_attrs, *args, **kwargs)

        self.motors = [self.x, self.y, self.z, self.o]


class PowerBrickVector(PowerBrickVectorBase):
    def __init__(self, prefix, *args, **kwargs):
        self._running_status = None
        super().__init__(prefix, *args, **kwargs)

    def kickoff(self):
        self._running_status = running_status = DeviceStatus(self)
        holding_status = DeviceStatus(self)

        if self.hold.get():
            def state_cb(value, old_value, **kwargs):
                if old_value != 2 and value == 2:
                    holding_status._finished()
                    self.state.clear_sub(state_cb)

            self.state.subscribe(state_cb, run=False)
        else:
            holding_status._finished()

        self.go.put(1, wait=True)

        def running_cb(value, old_value, obj, **kwargs):
            if old_value == 1 and value == 0:
                obj.clear_sub(running_cb)
                running_status._finished()

        self.running.subscribe(running_cb, run=False)
        return holding_status

    def complete(self):
        if self.hold.get():
            self.proceed.put(1, wait=True)
        return self._running_status


pb_vector = PowerBrickVector('XF:17IDC-ES:FMX{Gon:1-Vec}', name='pb_vector')
