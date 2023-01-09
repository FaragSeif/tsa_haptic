from time import perf_counter
from lcm_interface import commands_message, publish

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
            # commands_message.desired_position[:3] = [200, 200, 200]
            commands_message.desired_position = 4*[0,0,0,0]
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
