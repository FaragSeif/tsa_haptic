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

r_data = [[0], [0], [0]]

r_d_data = [[0], [0], [0]]


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
            re, jacobian_m, jacobian_d, jacobian_s = full_kinematics(device_states.carriage_positions, device_states.motor_angles)
            cartesian_force = -jacobian_m.T @ device_states.tensions

            ticks_data.append(device_states.timestamp)
            r_data[0].append(np.array(re[0]))
            r_data[1].append(np.array(re[1]))
            r_data[2].append(np.array(re[2]))
            
            r_d_data[0].append(np.array(commands.desired_position[0]))
            r_d_data[1].append(np.array(commands.desired_position[1]))
            r_d_data[2].append(np.array(commands.desired_position[2]))
            

            r_x = (np.array(ticks_data), np.array(r_data[0]), dict(_with='lines lc rgb "red" lw 2', legend='r_x'))
            r_y = (np.array(ticks_data), np.array(r_data[1]), dict(_with='lines lc rgb "green" lw 2', legend='r_y'))
            r_z = (np.array(ticks_data), np.array(r_data[2]), dict(_with='lines lc rgb "blue" lw 2', legend='r_z'))

            r_x_d = (np.array(ticks_data), np.array(r_d_data[0]), dict(_with='lines lc rgb "grey" lw 2 dashtype 4', legend='r_x'))
            r_y_d = (np.array(ticks_data), np.array(r_d_data[1]), dict(_with='lines lc rgb "grey" lw 2 dashtype 4', legend='r_y'))
            r_z_d = (np.array(ticks_data), np.array(r_d_data[2]), dict(_with='lines lc rgb "grey" lw 2 dashtype 4', legend='r_z'))


            if len(ticks_data) > 1:
                ttt = ticks_data[len(ticks_data)-1]
            else:
                ttt = 1
            gp.plot(r_x, r_y, r_z,
                    r_x_d, r_y_d, r_z_d,
                    _yrange=[-50, 120],
                    title='Cartesian position',
                    _xrange=[ttt-6000, ttt],
                    xlabel="Timestamp (ms)",
                    terminal='qt 1 size 800,400 noraise',
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
