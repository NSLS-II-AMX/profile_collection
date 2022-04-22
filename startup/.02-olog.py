print(f"Loading {__file__}")

from functools import partial
from pyOlog import SimpleOlogClient
from bluesky.callbacks.olog import logbook_cb_factory

# Set up the logbook. This configured bluesky's summaries of
# data acquisition (scan type, ID, etc.). It does NOT affect the
# convenience functions in ophyd (log_pos, etc.) or the IPython
# magics (%logit, %grabit). Those are configured in ~/.pyOlog.conf
# or wherever the pyOlog configuration file is stored.
LOGBOOKS = ['Commissioning']  # list of logbook names to publish to
simple_olog_client = SimpleOlogClient()
generic_logbook_func = simple_olog_client.log
configured_logbook_func = partial(generic_logbook_func, logbooks=LOGBOOKS)

cb = logbook_cb_factory(configured_logbook_func)

# Commented out by @mrakitin on 2021-05-12 due to the following issue:
# HTTPError: 403 Client Error: Forbidden for url: https://xf17id1-ca1.nsls2.bnl.local:9191/Olog/resources/logs
# Prior to that, the password in /etc/pyOlog.conf was updated to correctly escape the percent sign.
# RE.subscribe('start', cb)

