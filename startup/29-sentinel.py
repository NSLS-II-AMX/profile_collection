#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 17 15:43:19 2024

@author: xf17id1
"""


class PYZHomer(Device):

    status = Cpt(EpicsSignalRO, "XF:17IDB-ES:AMX{Sentinel}Homing_Sts")
    home_actuate = Cpt(EpicsSignal, "XF:17ID:AMX{Sentinel}pin_home")

    kill_home = Cpt(EpicsSignal, "XF:17IDB-ES:AMX{Sentinel}Homing_Kill")
    kill_py = Cpt(EpicsSignal, "XF:17IDB-ES:AMX{Gon:1-Ax:PY}Cmd:Kill-Cmd")
    kill_pz = Cpt(EpicsSignal, "XF:17IDB-ES:AMX{Gon:1-Ax:PY}Cmd:Kill-Cmd")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def trigger(self):

        def callback_homed(value, old_value, **kwargs):
            if old_value == 1 and value == 0:
                return True
            else:
                return False

        self.home_actuate.put(1)

        homing_status = SubscriptionStatus(
            self.status,
            callback_homed,
            run=False,
            timeout=180,
        )

        return homing_status


goniomon = EpicsSignal("XF:17ID:AMX{Karen}goniomon", name="goniomon")
govmon = EpicsSignal("XF:17ID:AMX{Karen}govmon", name="govmon")
pyz_homer = PYZHomer("", name="pyz_homer")
