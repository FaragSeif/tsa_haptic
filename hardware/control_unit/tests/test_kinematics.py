from interface import RobotInterface
from time import sleep, perf_counter
import numpy as np
from model.kinematics import *

robot = RobotInterface()

control = np.zeros(4)
try:
    t0 = perf_counter()

    while True:
        t = perf_counter() - t0
        # control = [10,10,10,50]
        robot.set_torques(torques = control)
        robot.update_state()
        position = robot.state.carriages.position
        theta = robot.state.modules.angles
        # end_effector_position = forward_kinematics(position)
        end_effector_position, jacobian_m, jacobian_d, jacobian_s = full_kinematics(position, theta)
        # print(jacobian_m.T @ robot.state.modules.forces)
        # print(jacobian_d, jacobian_s)
        
        # print(position)
        # print(end_effector_position)
        print(position, end_effector_position)
        

        
except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    robot.set_torques(torques = [0, 0, 0, 0])
    
    pass

