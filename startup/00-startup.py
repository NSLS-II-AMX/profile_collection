import sys
import logging

import bluesky
from ophyd import *
#from ophyd.commands import *

from databroker import DataBroker as db, get_table, get_images, get_events
from datamuxer import DataMuxer

def is_notebook():
    try:
        shell = get_ipython().__class__.__name__
        if shell == 'ZMQInteractiveShell':
            return True   # Jupyter notebook or qt console
        elif shell == 'TerminalInteractiveShell':
            return False  # Terminal running IPython
        else:
            return False  # Other type (?)
    except NameError:
        return False      # Probably
