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
        # print(t)
        if t - t_upd >= 1E-2:
            point_ind = int((t//T) % 5)
            commands_message.arm = True
            commands_message.mode = 12
            # commands_message.desired_position[:3] = [200, 200, 200]
            # commands_message.desired_position[:3] = [200, 200, 200]
            
            final_point = np.array([275, 280, 270])
            initial_point  = np.array([150, 150, 150])
            phases = np.array([0, 0, 0])
            # initial_point  = -final_point
            A = (final_point - initial_point)/2
            middle_point = (final_point + initial_point)/2
            omega = 1.0
            position = middle_point + A*np.sin(omega*t + phases)
            speed = A*omega*np.cos(omega*t)

            commands_message.desired_position[:3] = position
            commands_message.desired_speed[:3] = speed
            
            # commands_message.desired_position[:3] = [180, 200, 200]
            # commands_message.desired_speed[:3] = 3*[0]
            publish("commands")
            t_upd = t


except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    print('Controller is turning off...')
    commands_message.arm = True    
    commands_message.mode = 12
    commands_message.desired_position = 4*[0]
    commands_message.desired_speed = 4*[0]
    publish("commands")

