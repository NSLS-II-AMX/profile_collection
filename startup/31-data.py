last_scan_uid = None
last_scan_id = None


def fetch_scan(**kwargs):
    if len(kwargs) == 0:  # Retrieve last dataset
        header = db[-1]
        return header, header.table()
    else:
        headers = db(**kwargs)
        return headers, pd.concat([hdr.table() for hdr in headers]) 
