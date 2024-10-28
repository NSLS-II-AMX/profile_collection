#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 15:59:31 2024

@author: xf17id1
"""

from ophyd.utils.errors import StatusTimeoutError


@reset_positions_decorator([govmon, goniomon])
def home_pins():

    yield from bps.mv(
        govmon, 0,
        goniomon, 0,
        settle_time=1,
    )

    yield from bps.mv(
        pyz_homer.kill_py, 1,
        pyz_homer.kill_pz, 1,
        settle_time=1,
    )

    yield from bps.mv(gonio.o, 90)

    try:
        yield from bp.count([pyz_homer], 1)

    except StatusTimeoutError as e:
        print(f'Caught {e} during pinYZ home attempt, retrying')

        # kill 2x
        yield from bps.mv(
            pyz_homer.kill_py, 1,
            pyz_homer.kill_pz, 1,
            settle_time=1,
        )
        yield from bps.mv(
            pyz_homer.kill_py, 1,
            pyz_homer.kill_pz, 1,
            settle_time=1,
        )
        yield from bp.count([pyz_homer, gonio.o], 1)

    yield from bps.mv(gonio.o, 0)


def do_all():
    yield from screen4_centroid()
    yield from rot_pin_align_with_robot()
    yield from beam_align_flux()


def test_home_pins(n):
    for k in range(n):
        yield from bps.abs_set(gov_rbt, 'SE', wait=True)
        print(f'running test {k+1} of {n}')
        robrob.mount_special(6, 1000000)
        yield from home_pins()
        yield from bps.abs_set(gov_rbt, 'SE', wait=True)
        robrob.unmount_special(6, 1000000)
