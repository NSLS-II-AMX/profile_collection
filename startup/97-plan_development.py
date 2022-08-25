# algorithm development at the beamline


def test_plan3():
    
    #find optimal omega for sheath opening
    omega_scan_uid = yield from bp.scan([cam_6],gonio.o,-10,100,20)
    omega_scan_df = db[omega_scan_uid].table()
    omega_start = omega_scan_df['gonio_o'][omega_scan_df['cam_6_stats1_total'].idxmax()]
    yield from bps.mv(gonio.o, omega_start)
    omega_list = [omega_start + _ for _ in [0, 90, 180, 270]]
    
    #1st approximate rotation
    scan_uid = yield from bp.list_scan([cam_7], gonio.o, omega_list)
    d = np.pi/180
    A = np.matrix([[np.cos(omega*d), np.sin(omega*d), 1] for omega in omega_list])
    b = db[scan_uid].table()['cam_7_stats4_max_xy_y']
    print(f'b: {b}')
    p0 = np.linalg.inv(A.transpose()*A)*A.transpose()*np.matrix(b.to_numpy()).transpose()
    delta_y_pix = p0[0]
    delta_z_pix = p0[1]
    rot_y_pix = p0[2]
    print(rot_y_pix)
    pix_per_um = 3.4
    delta_y_um = delta_y_pix / pix_per_um
    delta_z_um = delta_z_pix / pix_per_um
    yield from bps.mvr(gonio.py, -delta_y_um)
    yield from bps.sleep(0.1)
    yield from bps.mvr(gonio.pz, -delta_z_um)
    
    scan_uid = yield from bp.list_scan([cam_7], gonio.o, omega_list)
    d = np.pi/180
    A = np.matrix([[np.cos(omega*d), np.sin(omega*d), 1] for omega in omega_list])
    b = db[scan_uid].table()['cam_7_stats4_max_xy_y']
    print(f'b: {b}')
    p0 = np.linalg.inv(A.transpose()*A)*A.transpose()*np.matrix(b.to_numpy()).transpose()
    delta_y_pix = p0[0]
    delta_z_pix = p0[1]
    rot_y_pix = p0[2]
    print(rot_y_pix)
    pix_per_um = 3.4
    delta_y_um = delta_y_pix / pix_per_um
    delta_z_um = delta_z_pix / pix_per_um
    yield from bps.mvr(gonio.py, -delta_y_um)
    yield from bps.sleep(0.1)
    yield from bps.mvr(gonio.pz, -delta_z_um)
    
    yield from bps.mvr(gonio.gx,-25)
    
    scan_uid = yield from bp.list_scan([cam_7], gonio.o, omega_list)
    d = np.pi/180
    A = np.matrix([[np.cos(omega*d), np.sin(omega*d), 1] for omega in omega_list])
    b = db[scan_uid].table()['cam_7_stats4_max_xy_y']
    print(f'b: {b}')
    p0 = np.linalg.inv(A.transpose()*A)*A.transpose()*np.matrix(b.to_numpy()).transpose()
    delta_y_pix = p0[0]
    delta_z_pix = p0[1]
    rot_y_pix = p0[2]
    print(rot_y_pix)
    pix_per_um = 3.4
    delta_y_um = delta_y_pix / pix_per_um
    delta_z_um = delta_z_pix / pix_per_um
    yield from bps.mvr(gonio.py, -delta_y_um)
    yield from bps.sleep(0.1)
    yield from bps.mvr(gonio.pz, -delta_z_um)
    
    yield from bps.mv(gonio.o, omega_start)
    scan_uid = yield from bp.rel_scan([cam_7,gcp3.real_y,gcp3.real_z], gcp3.cam_y, -5, 5, 20)
    scan_df = db[scan_uid].table()
    b = scan_df['cam_7_stats4_max_xy_y']
    A = np.matrix([[cam_y, 1] for cam_y in scan_df['gcp_cam_y']])
    v0 = np.linalg.inv(A.transpose()*A)*A.transpose()*np.matrix(b.to_numpy()).transpose()
    best_cam_y = (p0[2] - v0[1]) / v0[0]
    print(best_cam_y)
    yield from bps.mv(gcp3.cam_y,best_cam_y)
    
    yield from bps.mv(gonio.o, omega_start + 90)
    scan_uid = yield from bp.rel_scan([cam_7,gcp3.real_y,gcp3.real_z], gcp3.cam_y, -5, 5, 20)
    scan_df = db[scan_uid].table()
    b = scan_df['cam_7_stats4_max_xy_y']
    A = np.matrix([[cam_y, 1] for cam_y in scan_df['gcp_cam_y']])
    v0 = np.linalg.inv(A.transpose()*A)*A.transpose()*np.matrix(b.to_numpy()).transpose()
    best_cam_y = (p0[2] - v0[1]) / v0[0]
    
    print(best_cam_y)
    yield from bps.mv(gcp3.cam_y,best_cam_y)
