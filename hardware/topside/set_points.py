from time import perf_counter, sleep
from lcm_interface import commands_message, publish

commands_message.mode = 0 

set_points = [[0, 0, 70], 
              [0, 45, 80], 
              [35, 0, 90], 
              [0, -15, 80],
              [-35, 0, 90]]

T = 1.5
        
for i in range(200):
    commands_message.arm = True
    commands_message.mode = 11
    commands_message.desired_position[:3] = [200, 200, 200]
    commands_message.desired_speed[:3] = 3*[0]
    publish("commands")
    sleep(0.01)

try:
    t_upd = 0 
    t0 = perf_counter()
    while True:
        t = perf_counter() - t0
        # for point in set_points:
        if t - t_upd >= 1E-2:
            point_ind = int((t//T) % 5)
            commands_message.timestamp = int(t*1000)
            commands_message.arm = True
            # print(control)
            commands_message.mode = 21
            commands_message.desired_position[:3] = set_points[point_ind]
            # commands_message.desired_position[:3] = 3*set_points[point_ind]
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

