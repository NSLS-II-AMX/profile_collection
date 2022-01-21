print(f"Loading {__file__}")

from ophyd import Device, EpicsSignal, EpicsSignalRO, EpicsMotor
from ophyd import Component as Cpt
from ophyd.utils import set_and_wait


# Undulator
class InsertionDevice(Device):
    gap = Cpt(EpicsMotor, '-Ax:Gap}-Mtr',
              name='')
    brake = Cpt(EpicsSignal, '}BrakesDisengaged-Sts',
                write_pv='}BrakesDisengaged-SP',
                kind='omitted', add_prefix=('read_pv', 'write_pv', 'suffix'))

    def set(self, *args, **kwargs):
        set_and_wait(self.brake, 1)
        return self.gap.set(*args, **kwargs)

    def stop(self, *, success=False):
        return self.gap.stop(success=success)


ivu_gap = InsertionDevice('SR:C17-ID:G1{IVU21:1', name='ivu_gap')
ivu_gap.gap.user_setpoint.name = 'ivu_gap_setpoint'
ivu_gap.gap.user_readback.name = 'ivu_gap_readback'
