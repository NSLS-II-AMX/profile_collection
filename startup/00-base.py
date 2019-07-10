print(f'Loading {__file__}')

import nslsii
nslsii.configure_base(get_ipython().user_ns, 'amx')
# nslsii.configure_olog(get_ipython().user_ns, subscribe=False)

#Optional: set any metadata that rarely changes.
RE.md['beamline_id'] = 'AMX'


