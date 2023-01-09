from time import perf_counter
from lcm_interface import device_states, controller_states, commands, handle, subscribe, unsubscribe
from model.kinematics import *
import gnuplotlib as gp
import numpy as np
# TODO:
# REWRITE WITH CURSES

SAMPLING = 30
UPDATE_RATE = 1000/SAMPLING

commands_subscription = subscribe("commands")
device_subscription = subscribe("device")
controller_subscription = subscribe("controller")


ts = 0
# T = [0]
ticks = 0
i = 0
N = 1000000

ticks_data = [0]

f_data = [[0], [0], [0]]


try:
    # for i in range(50):
    #     print(50*' ')
    # print(50*'\033[A', end="\r", flush=True)

    ts = 0
    t0 = perf_counter()
    print('Printer is started...')
    while True:
        t = perf_counter() - t0

        handled = handle(blocking=False)

        if t - ts >= 1/UPDATE_RATE and handled:
            re, jacobian_m, jacobian_d, jacobian_s = full_kinematics(
                device_states.carriage_positions, device_states.motor_angles)
            cartesian_force = -jacobian_m.T @ device_states.tensions

            ticks_data.append(device_states.timestamp)
            f_data[0].append(np.array(cartesian_force[0]))
            f_data[1].append(np.array(cartesian_force[1]))
            f_data[2].append(np.array(cartesian_force[2]))
            if t <= 2:
                f_x_mean = np.mean(f_data[0])
                f_y_mean = np.mean(f_data[1])
                f_z_mean = np.mean(f_data[2])

            if t > 2:
                f_x = (np.array(ticks_data), np.array(
                    f_data[0]) - f_x_mean, dict(_with='lines lw 2', legend='f_x'))
                f_y = (np.array(ticks_data), np.array(
                    f_data[1]) - f_y_mean, dict(_with='lines lw 2', legend='f_y'))
                f_z = (np.array(ticks_data), np.array(
                    f_data[2]) - f_z_mean, dict(_with='lines lw 2', legend='f_z'))

                if len(ticks_data) > 1:
                    ttt = ticks_data[len(ticks_data)-1]
                else:
                    ttt = 1
                gp.plot(f_x, f_y, f_z,
                        _yrange=[-80, 80],
                        title='Forces',
                        _xrange=[ttt-3000, ttt],
                        xlabel="Timestamp (ms)",
                        terminal='qt 1 size 800,300 noraise',
                        # _set = 'size ratio 0.5'
                        
                        )

            ts = t

except KeyboardInterrupt:
    # print(10*'\n')
    unsubscribe(controller_subscription)
    print('[LCM] unsubscribed from controller subscription')
    unsubscribe(device_subscription)
    print('[LCM] unsubscribed from device subscription')
    unsubscribe(commands_subscription)
    print('[LCM] unsubscribed from commands subscription')
    print(100*'\033[A')
