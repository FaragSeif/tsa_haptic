from time import perf_counter, sleep
from lcm_interface import device_states, controller_states, commands, handle, subscribe, unsubscribe
import numpy as np
# TODO:
# REWRITE WITH CURSES

SAMPLING = 10
UPDATE_RATE = 1000/SAMPLING

commands_subscription = subscribe("commands")
device_subscription = subscribe("device")
controller_subscription = subscribe("controller")
N = 10000000
n = 1 + 4 + 4 + 4 + 4 + 3 + 4
data_array = np.zeros((N,n))

label = 'cartesian_points_v2'
# label = 'cartesian_points_v2'
# label = 'cartesian_circle_v4'
# label = 'z_motion_slow_motors_7kg'
# label = 'calibration'
# label = 'cartesian_points_v3'
label = 'cartesian_points_v3'


try:
    # for i in range(50):
    #     print(50*' ')
    # print(50*'\033[A', end="\r", flush=True)
        
    ts = 0
    t0 = perf_counter()
    print('[LOGGER] Logger is started...')
    i = 0
    while i<=N:
        t = perf_counter() - t0 
        
        handled = handle(blocking = False)
            
        if t - ts >= 1/UPDATE_RATE and handled:    
            data_array[i, 0] = t#device_states.timestamp
            data_array[i, 1:5] = device_states.motor_angles
            data_array[i, 5:9] = device_states.motor_speeds
            data_array[i, 9:13] = device_states.motor_torques
            data_array[i, 13:16] = device_states.carriage_positions
            data_array[i, 16:20] = device_states.tensions
            data_array[i, 20:] = commands.desired_position
            i+=1
        
            ts = t

except KeyboardInterrupt:
    # print(10*'\n')
    unsubscribe(controller_subscription)
    print('[LCM] unsubscribed from controller subscription')
    unsubscribe(device_subscription)
    print('[LCM] unsubscribed from device subscription')
    unsubscribe(commands_subscription)
    print('[LCM] unsubscribed from commands subscription')

    print('[LOGGER] Saving the data file...')
    data_to_save = data_array[:i-1, :]
    np.savetxt(label + '.csv', data_to_save, delimiter=',')
    print(f'[LOGGER] File is saved to {label}.csv')
    # print(data_to_save)
    # save data