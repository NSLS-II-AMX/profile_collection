import asyncio
from functools import partial
from bluesky.plans import *
from bluesky.spec_api import *
from bluesky.callbacks import *
from bluesky.callbacks.olog import logbook_cb_factory

# Import matplotlib and put it in interactive mode
import matplotlib.pyplot as plt
plt.ion()

import os
if is_notebook():
    from bluesky.utils import install_nb_kicker
    install_nb_kicker()
else:
    from bluesky.utils import install_qt_kicker
    install_qt_kicker()
    print("Insalling Qt Kicker...")

# Subscribe metadatastore to documents.
# If this is removed, data is not saved to metadatastore.
# import metadatastore.commands
from bluesky.global_state import gs

RE = gs.RE
abort = RE.abort
resume = RE.resume
stop = RE.stop


from metadatastore.mds import MDS
# from metadataclient.mds import MDS
from databroker import Broker
from databroker.core import register_builtin_handlers
from filestore.fs import FileStore


mds = MDS({'host':'xf17id2-ca1', 'database': 'datastore', 'port': 27017, 
           'timezone': 'US/Eastern'}, auth=False)
# mds = MDS({'host': 'xf17id2-ca1', 'port': 7770})

# pull configuration from /etc/filestore/connection.yaml or
# /home/BLUSER/.config/filestore/connection.yml
db = Broker(mds, FileStore({'host': 'xf17id2-ca1', 'port': 27017, 
                            'database': 'filestore'}))
register_builtin_handlers(db.fs)
# hook run engine up to metadatastore
RE.subscribe('all', mds.insert)


# RE.subscribe_lossless('all', metadatastore.commands.insert)


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

