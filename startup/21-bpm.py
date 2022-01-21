print(f"Loading {__file__}")

from ophyd import Device, EpicsSignal, Component as Cpt, EpicsSignalRO

class Bpm(Device):
    x = Cpt(EpicsSignalRO, 'PosX:MeanValue_RBV')
    y = Cpt(EpicsSignalRO, 'PosY:MeanValue_RBV')
    a = Cpt(EpicsSignalRO, 'Current1:MeanValue_RBV')
    b = Cpt(EpicsSignalRO, 'Current2:MeanValue_RBV')
    c = Cpt(EpicsSignalRO, 'Current3:MeanValue_RBV')
    d = Cpt(EpicsSignalRO, 'Current4:MeanValue_RBV')
    sum_x = Cpt(EpicsSignalRO, 'SumX:MeanValue_RBV')
    sum_y = Cpt(EpicsSignalRO, 'SumY:MeanValue_RBV')
    sum_all = Cpt(EpicsSignalRO, 'SumAll:MeanValue_RBV')

class Best(Device):
    x_mean  = Cpt(EpicsSignal, 'PosX_Mean')
    x_std = Cpt(EpicsSignal, 'PosX_Std')
    y_mean  = Cpt(EpicsSignal, 'PosY_Mean')
    y_std = Cpt(EpicsSignal, 'PosY_Std')
    int_mean  = Cpt(EpicsSignal, 'Int_Mean')
    int_std = Cpt(EpicsSignal, 'Int_Std')

#best = Best('XF:16IDB-CT{Best}:BPM0:', name='best')
bpm2 = Bpm('XF:17IDB-BI:AMX{BPM:2}', name='bpm2')
bpm3 = Bpm('XF:17IDB-BI:AMX{BPM:3}', name='bpm3')

bpm3.kind = 'hinted'
bpm3.sum_all.kind = 'hinted'
