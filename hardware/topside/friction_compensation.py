from time import perf_counter, sleep
from lcm_interface import commands_message, publish

commands_message.mode = 0 

try:
    t_upd = 0 
    t0 = perf_counter()
    while True:
        t = perf_counter() - t0
        # for point in set_points:

        if t - t_upd >= 1E-2:
            commands_message.timestamp = int(t*1000)
            commands_message.arm = True
            
            # print(control)
            commands_message.mode = 2
            publish("commands")
            t_upd = t


except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    print('Controller is turning off...')
