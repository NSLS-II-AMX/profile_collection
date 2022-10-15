print(f"Loading {__file__}")

import os
import uuid
from pathlib import Path
import time

import appdirs
import matplotlib
import matplotlib.pyplot
import nslsii
import numpy as np
from IPython import get_ipython
from ophyd import Component, Device, EpicsSignal
from ophyd.areadetector.filestore_mixins import resource_factory
from ophyd.signal import EpicsSignalBase

try:
    ###############################################################################
    # TODO: remove this block once https://github.com/bluesky/ophyd/pull/959 is
    # merged/released.
    from datetime import datetime

    from ophyd.signal import (DEFAULT_CONNECTION_TIMEOUT, EpicsSignal,
                              EpicsSignalBase)

    def print_now():
        return datetime.strftime(datetime.now(), '%Y-%m-%d %H:%M:%S.%f')

    def wait_for_connection_base(self, timeout=DEFAULT_CONNECTION_TIMEOUT):
        '''Wait for the underlying signals to initialize or connect'''
        if timeout is DEFAULT_CONNECTION_TIMEOUT:
            timeout = self.connection_timeout
        # print(f'{print_now()}: waiting for {self.name} to connect within {timeout:.4f} s...')
        start = time.time()
        try:
            self._ensure_connected(self._read_pv, timeout=timeout)
            # print(f'{print_now()}: waited for {self.name} to connect for {time.time() - start:.4f} s.')
        except TimeoutError:
            if self._destroyed:
                raise DestroyedError('Signal has been destroyed')
            raise

    def wait_for_connection(self, timeout=DEFAULT_CONNECTION_TIMEOUT):
        '''Wait for the underlying signals to initialize or connect'''
        if timeout is DEFAULT_CONNECTION_TIMEOUT:
            timeout = self.connection_timeout
        # print(f'{print_now()}: waiting for {self.name} to connect within {timeout:.4f} s...')
        start = time.time()
        self._ensure_connected(self._read_pv, self._write_pv, timeout=timeout)
        # print(f'{print_now()}: waited for {self.name} to connect for {time.time() - start:.4f} s.')

#    EpicsSignalBase.wait_for_connection = wait_for_connection_base
    EpicsSignal.wait_for_connection = wait_for_connection
    ###############################################################################

    from ophyd.signal import EpicsSignalBase

    # EpicsSignalBase.set_default_timeout(timeout=10, connection_timeout=10)  # old style
#    EpicsSignalBase.set_defaults(timeout=10, connection_timeout=10)  # new style

except ImportError:
    pass

'''
nslsii.configure_base(get_ipython().user_ns, 'amx', bec=True, pbar=False,
                      publish_documents_with_kafka=True)

# Disable plots via BestEffortCallback:
bec.disable_plots()

try:
    from bluesky.utils import PersistentDict
    runengine_metadata_dir = appdirs.user_data_dir(appname="bluesky") / Path(
        "runengine-metadata"
    )
    # PersistentDict will create the directory if it does not exist
    RE.md = PersistentDict(runengine_metadata_dir)
except ImportError:
    print('Older bluesky did not have PersistentDict, moving on.')

#Optional: set any metadata that rarely changes.
RE.md['beamline_id'] = 'AMX'
'''