print(f"Loading {__file__}")

import os
import matplotlib
from IPython import get_ipython

# get_ipython().run_line_magic('matplotlib', 'widget')  # i.e. %matplotlib widget
import matplotlib.pyplot

from ophyd import Device, Component, EpicsSignal
from ophyd.signal import EpicsSignalBase
from ophyd.areadetector.filestore_mixins import resource_factory
import uuid
import os
from pathlib import Path
import numpy as np

# Set up a RunEngine and use metadata backed by a sqlite file.
from bluesky import RunEngine
from bluesky.utils import get_history
RE = RunEngine(get_history())

# Set up SupplementalData.
from bluesky import SupplementalData
sd = SupplementalData()
RE.preprocessors.append(sd)

# Set up a Broker.
from databroker import Broker
db = Broker.named('amx')

# and subscribe it to the RunEngine
RE.subscribe(db.insert)

# Add a progress bar.
# from bluesky.utils import ProgressBarManager
# pbar_manager = ProgressBarManager()
# RE.waiting_hook = pbar_manager

# Register bluesky IPython magics.
from bluesky.magics import BlueskyMagics
get_ipython().register_magics(BlueskyMagics)

# Set up the BestEffortCallback.
from bluesky.callbacks.best_effort import BestEffortCallback
bec = BestEffortCallback()
bec.overplot = False
RE.subscribe(bec)
peaks = bec.peaks

# Import matplotlib and put it in interactive mode.
import matplotlib.pyplot as plt
plt.ion()

# Make plots update live while scans run.
from bluesky.utils import install_nb_kicker
install_nb_kicker()

# convenience imports
# some of the * imports are for 'back-compatibility' of a sort -- we have
# taught BL staff to expect LiveTable and LivePlot etc. to be in their
# namespace
import numpy as np

import bluesky.plans as bp
import bluesky.plan_stubs as bps
import bluesky.preprocessors as bpp

#Optional: set any metadata that rarely changes.
RE.md['beamline_id'] = 'AMX'
