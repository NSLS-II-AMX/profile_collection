print(f"Loading {__file__}")

from pyOlog.ophyd_tools import get_all_positioners
from bluesky.magics import BlueskyMagics


def wh_pos():
    raise RuntimeError("wh_pos() was removed. Use wa with no parenthesis")

BlueskyMagics.positioners = get_all_positioners()
