from ophyd import PVPositionerPC, EpicsSignal, EpicsSignalRO
from ophyd import Component as Cpt

# Undulator

class Undulator(PVPositionerPC):
    readback = Cpt(EpicsSignalRO, '-LEnc}Gap')
    setpoint = Cpt(EpicsSignal, '-Mtr:2}Inp:Pos')
    actuate = Cpt(EpicsSignal, '-Mtr:2}Sw:Go')
    actuate_value = 1
    stop_signal = Cpt(EpicsSignal, '-Mtr:2}Pos.STOP')
    stop_value = 1

ivu_gap = Undulator('SR:C17-ID:G1{IVU21:1', name='ivu_gap')
