
def print_scanid(name, doc):
    global last_scan_uid
    global last_scan_id
    if name == 'start':
        last_scan_id = doc['scan_id']
        last_scan_uid = doc['uid']
        print('Scan ID:', doc['scan_id'])
        print('Unique ID:', doc['uid'])

def print_md(name, doc):
    if name == 'start':
        print('Metadata:\n', repr(doc))

gs.RE.subscribe('start', print_scanid)

# For debug purpose to see the metadata being stored
#gs.RE.subscribe('start', print_md)
