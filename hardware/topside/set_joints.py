from time import perf_counter
from lcm_interface import commands_message, publish

commands_message.mode = 0

set_points = [[220, 220, 220],
              [140, 180, 180], 
              [190, 220, 220],
              [210, 170, 220], 
              [210, 220, 170]]

T = 2

try:
    t_upd = 0
    t0 = perf_counter()
    while True:
        t = perf_counter() - t0
        if t - t_upd >= 1E-2:
            point_ind = int((t//T) % 5)
            commands_message.arm = True
            commands_message.mode = 11
            commands_message.desired_position[:3] = set_points[point_ind] 
            publish("commands")
            t_upd = t


except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    print('Controller is turning off...')
    commands_message.arm = True
    commands_message.mode = 10
    publish("commands")
    # lc.publish("commands", commands_message.encode())
