from ophyd import Device, Component as Cpt, EpicsMotor, EpicsSignalRO


class XYMotor(Device):
	x = Cpt(EpicsMotor, '-Ax:X}Mtr')
	y = Cpt(EpicsMotor, '-Ax:Y}Mtr')


class XYPitchMotor(XYMotor):
	pitch = Cpt(EpicsMotor, '-Ax:P}Mtr')


class Slits(Device):
	b = Cpt(EpicsMotor, '-Ax:B}Mtr')
	i = Cpt(EpicsMotor, '-Ax:I}Mtr')
	o = Cpt(EpicsMotor, '-Ax:O}Mtr')
	t = Cpt(EpicsMotor, '-Ax:T}Mtr')
	x_ctr = Cpt(EpicsMotor, '-Ax:XCtr}Mtr')
	x_gap = Cpt(EpicsMotor, '-Ax:XGap}Mtr')
	y_ctr = Cpt(EpicsMotor, '-Ax:YCtr}Mtr')
	y_gap = Cpt(EpicsMotor, '-Ax:YGap}Mtr')


class VerticalDCM(Device):
	b = Cpt(EpicsMotor, '-Ax:B}Mtr')
	g = Cpt(EpicsMotor, '-Ax:G}Mtr')
	p = Cpt(EpicsMotor, '-Ax:P}Mtr')
	r = Cpt(EpicsMotor, '-Ax:R}Mtr')


class TandemMirrors(Device):
	pd = Cpt(EpicsMotor, '-Ax:PD}Mtr')
	pu = Cpt(EpicsMotor, '-Ax:PU}Mtr')
	rd = Cpt(EpicsMotor, '-Ax:RD}Mtr')
	xd = Cpt(EpicsMotor, '-Ax:XD}Mtr')
	xu = Cpt(EpicsMotor, '-Ax:XU}Mtr')
	yd = Cpt(EpicsMotor, '-Ax:YD}Mtr')
	yu = Cpt(EpicsMotor, '-Ax:YU}Mtr')


class KBMirror(Device):
	hp = Cpt(EpicsMotor, ':KBH-Ax:P}Mtr')
	hr = Cpt(EpicsMotor, ':KBH-Ax:R}Mtr')
	hx = Cpt(EpicsMotor, ':KBH-Ax:X}Mtr')
	hy = Cpt(EpicsMotor, ':KBH-Ax:Y}Mtr')
	vp = Cpt(EpicsMotor, ':KBV-Ax:P}Mtr')
	vx = Cpt(EpicsMotor, ':KBV-Ax:X}Mtr')
	vy = Cpt(EpicsMotor, ':KBV-Ax:Y}Mtr')


class GoniometerStack(Device):
	gx = Cpt(EpicsMotor, '-Ax:GX}Mtr')
	gy = Cpt(EpicsMotor, '-Ax:GY}Mtr')
	gz = Cpt(EpicsMotor, '-Ax:GZ}Mtr')
	o = Cpt(EpicsMotor, '-Ax:O}Mtr')
	py = Cpt(EpicsMotor, '-Ax:PY}Mtr')
	pz = Cpt(EpicsMotor, '-Ax:PZ}Mtr')

class ShutterTranslation(Device):
	x = Cpt(EpicsMotor, '-Ax:X}Mtr')

#######################################################
### FMX
#######################################################

## High Heat Load Slits
hhls = Slits('XF:17IDA-OP:AMX{Slt:0', name='hhls')

## Vertical Double Crystal Monochromator
vdcm = VerticalDCM('XF:17IDA-OP:AMX{Mono:DCM', name='vdcm')


## Tandem Deflection Mirrors
tdm = TandemMirrors('XF:17IDA-OP:AMX{Mir:TDM', name='tdm')

## BPMS Motions
mbpm1 = XYMotor('XF:17IDA-BI:AMX{BPM:1', name='mbpm1')
mbpm2 = XYMotor('XF:17IDB-BI:AMX{BPM:2', name='mbpm2')

## Slits
slits1 = Slits('XF:17IDA-OP:AMX{Slt:1', name='slits1')
slits2 = Slits('XF:17IDB-OP:AMX{Slt:2', name='slits2')

## KB Mirror
kbm = KBMirror('XF:17IDB-OP:AMX{Mir', name='kbm')

## Microscope
mic = XYMotor('XF:17IDB-ES:AMX{Mic:1', name='mic')

## Goniometer Stack
gonio = GoniometerStack('XF:17IDB-ES:AMX{Gon:1', name='gonio')

## Beam Conditioning Unit Shutter Translation
sht = ShutterTranslation('XF:17IDB-ES:AMX{Sht:1', name='sht')

