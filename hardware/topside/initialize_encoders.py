from time import perf_counter
from lcm_interface import lc, commands_message, publish

commands_message.mode = 0 
# commands_message.desired_position = np.array([0, 0, 0, 0])

try:
    t_upd = 0 
    t0 = perf_counter()
    while True:
        t = perf_counter() - t0

        if t - t_upd >= 1E-2:
            # print(control)
            commands_message.arm = True
            commands_message.timestamp = int(t*1000)
            commands_message.mode = -1
            
            publish("commands")
            t_upd = t


except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    print('Controller is turning off...')
    commands_message.arm = False
    commands_message.mode = 1 
    # commands_message.error_flag = -1
    publish("commands")
    # lc.publish("commands", commands_message.encode())
