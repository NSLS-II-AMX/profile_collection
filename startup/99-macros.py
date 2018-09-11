def scan_bpm(bpm_num, bpm_motor, start, end, step):
    detector_name = 'bpm' + str(bpm_num)
    device_name = 'mbpm' + str(bpm_num)
    motor_name = '_'.join((device_name, bpm_motor))

    detector = globals()[detector_name]
    device = globals()[device_name]
    motor = getattr(device, bpm_motor)
    egu = motor.motor_egu.get()

    plan = scan([detector], motor, start, end, step)
    result_uid = RE(plan)
    table = db.get_table(db[result_uid])
    channels = ['A', 'B', 'C', 'D']
    ydata = [table['_'.join((detector_name, ch.lower()))] for ch in channels]
    xdata = table[motor_name]

    plt.xlabel('Motor position ({})'.format(egu))
    plt.ylabel('Currents')
    plt.title('BPM {} Currents vs Motor Position'.format(bpm_num))
    plots = [plt.plot(xdata, ydata[i], label=ch) for i, ch in enumerate(channels)]
    plt.legend(channels)
    return result_uid
