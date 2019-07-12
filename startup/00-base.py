from IPython import get_ipython
get_ipython().run_line_magic('matplotlib', 'notebook') # i.e. %matplotlib notebook

import nslsii
nslsii.configure_base(get_ipython().user_ns, 'amx', bec=True, mpl=True, pbar=False)
# nslsii.configure_olog(get_ipython().user_ns, subscribe=False)

#Optional: set any metadata that rarely changes.
RE.md['beamline_id'] = 'AMX'