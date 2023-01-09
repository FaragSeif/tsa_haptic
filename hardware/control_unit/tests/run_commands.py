from time import perf_counter
import lcm
from protocol import device_states_t, controller_states_t, commands_t
import numpy as np
from model.kinematics import *
from control_unit.tests.qp_controller import get_torques
from types import SimpleNamespace
import select as select

# address = "udpm://239.255.76.67:7667?ttl=1"
address = ""
lc = lcm.LCM(address)


commands_message = commands_t()
lc.publish("commands", commands_message.encode())

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
            commands_message.mode = 11
            

            lc.publish("commands", commands_message.encode())
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
    lc.publish("commands", commands_message.encode())
