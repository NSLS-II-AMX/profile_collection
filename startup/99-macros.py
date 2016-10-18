def simple_ascan(camera, stats, motor, start, end, steps):
    gs.DETS = [camera]
    gs.MASTER_DET = camera
 
    stats_name = "_".join((camera.name,stats)) if stats else camera.name
    gs.PLOT_Y = stats_name

    uid = RE(ascan(motor, start, end, steps))[0]
    table = get_table(db[uid])
    try:
        return table[[motor.name, stats_name]]
    except KeyError:
        return table[[motor.name+"_readback", stats_name]]
