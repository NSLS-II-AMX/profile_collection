import asyncio
from functools import partial
from bluesky.plans import *
#from bluesky.spec_api import *
from bluesky.callbacks import *
from bluesky.callbacks.olog import logbook_cb_factory
from bluesky.utils import install_qt_kicker
import nslsii

nslsii.configure_base(get_ipython().user_ns, 'amx')

# The following line allows bluesky and pyqt4 GUIs to play nicely together:
install_qt_kicker()


# Subscribe metadatastore to documents.
# If this is removed, data is not saved to metadatastore.
#import metadatastore.commands
#from bluesky.global_state import gs

from bluesky import RunEngine
RE = RunEngine()

#RE = gs.RE
abort = RE.abort
resume = RE.resume
stop = RE.stop

#RE.subscribe_lossless('all', metadatastore.commands.insert)


RE.md['group'] = 'amx'
RE.md['beamline_id'] = 'AMX'
RE.ignore_callback_exceptions = True 

loop = asyncio.get_event_loop()
loop.set_debug(False)
# RE.verbose = True

# sr_shutter_status = EpicsSignal('SR-EPS{PLC:1}Sts:MstrSh-Sts', rw=False,
#                                 name='sr_shutter_status')
# sr_beam_current = EpicsSignal('SR:C03-BI{DCCT:1}I:Real-I', rw=False,
#                               name='sr_beam_current')

