from time import perf_counter, sleep
from lcm_interface import commands_message, publish
import numpy as np

commands_message.mode = 0

try:
    t_upd = 0
    t0 = perf_counter()
    while True:
        t = perf_counter() - t0
        if t - t_upd >= 1E-2:
            A, omega = 40, 2.0
            commands_message.arm = True
            commands_message.timestamp = int(t*1000)
            commands_message.mode = 21
            commands_message.desired_position[:3] = [0,
                                                     20 + A*np.cos(omega*t),
                                                     80]

            commands_message.desired_speed[:3] = [0,
                                                  -A*omega*np.sin(omega*t),
                                                  0]
            publish("commands")
            t_upd = t


except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    print('Controller is turning off...')
    commands_message.arm = True
    

    commands_message.desired_position[:3] = [0, 0, 80]
    publish("commands")
    sleep(0.5)
    
    commands_message.arm = True
    commands_message.mode = 10
    # commands_message.error_flag = -1
    publish("commands")

