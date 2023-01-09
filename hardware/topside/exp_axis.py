from time import perf_counter
from lcm_interface import commands_message, publish
import numpy as np
commands_message.mode = 0


T = 0.5

try:
    t_upd = 0
    t0 = perf_counter()
    while True:
        t = perf_counter() - t0
        if t - t_upd >= 1E-2:
            point_ind = int((t//T) % 5)
            commands_message.arm = True
            commands_message.mode = 12
            initial_point, final_point = 35, 210
            A = (final_point - initial_point)/2
            middle_point = (final_point + initial_point)/2
            omega = 0.5
            position = middle_point + A*np.sin(omega*t)
            speed = A*omega*np.cos(omega*t)

            commands_message.desired_position[:3] = [position-10, position+15, position+20]
            commands_message.desired_position[3] = 0
            
            commands_message.desired_speed[:3] = [speed, speed, speed]
            commands_message.desired_speed[3] = 0
            publish("commands")
            t_upd = t


except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    print('Controller is turning off...')
    position = 0
    commands_message.desired_position[:3] = [position, position+10, position+10]
    # commands_message.desired_position[:3] = [position, position, position]
    commands_message.desired_speed = np.zeros(4)
    commands_message.mode = 12
    commands_message.arm = True
    publish("commands")
    # lc.publish("commands", commands_message.encode())
