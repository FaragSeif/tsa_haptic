from interface import RobotInterface
from time import sleep, perf_counter
import numpy as np
from model.kinematics import *

robot = RobotInterface()

#
control = np.zeros(4)

kp = 4.0
kd = 0.3


t0 = perf_counter()
# theta = np.zeros(3)
# theta_prev = np.zeros(3)
min_position = np.inf
linear_offsets = []
try:
    t0 = perf_counter()
    module_id = 0
    while True:
        t = perf_counter() - t0
        control = np.zeros(4)
        control[3] = 45
        

        control[module_id] = 5*np.sign(np.sin(0.5*np.pi*t))
        
        robot.set_torques(torques = control)
        robot.update_state()
        position = robot.state.carriages.position[module_id] 
        
        if position <= min_position:
            min_position = position     
            
        if t>=5:
            print(f'Module {module_id + 1} is initialized')
            linear_offsets.append(min_position)
            min_position = np.inf
            module_id+=1
            t0 = perf_counter()
        
        
        if module_id == 3:
            print(f'Linear encoders are initialized, \noffsets are {linear_offsets}')
            break
        
        # print(position, min_position)
        # theta = robot.state.modules.angles[2]
        # dtheta = robot.state.modules.speeds[2]
        # x = robot.state.modules.speeds[]

        # # print()
        # A, omega = 250, 1
        # # A, omega = 0, 0
        
        # theta_d = -100+A*np.sin(omega*t) 
        # dtheta_d = A*omega*np.cos(omega*t) 
        
        # control = kp*(theta_d - theta) + kd*(dtheta_d - dtheta)
        

        # print(robot.state.modules.forces)
        # print(robot.state.carriages.position)
        # print('\n')
        

        
except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    robot.set_torques(torques = [0, 0, 0, 0])
    pass

