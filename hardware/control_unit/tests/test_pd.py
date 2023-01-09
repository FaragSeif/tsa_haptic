from interface import RobotInterface
from time import sleep, perf_counter
import numpy as np
# from model.kinematics import *

robot = RobotInterface()

#
control = np.zeros(4)
control[3] = 60

kp = 6.0*np.eye(3)
kd = 0.3*np.eye(3)


t0 = perf_counter()

try:
    while True:
        t = perf_counter() - t0
        robot.set_torques(torques = [control[0], 
                                     control[1], 
                                     control[2], 
                                     control[3]])
        
        # print(theta)
        robot.update_state()
        theta = robot.state.modules.angles
        position = robot.state.carriages.position
        dtheta = robot.state.modules.speeds

        # print()
        A, omega = 50, 3.0
        # A, omega = 0, 0
        # theta_d = 
        theta_d_1 = 140 + A*np.sin(omega*t) 
        dtheta_d_1 = A*omega*np.cos(omega*t)
        theta_d =  np.array([theta_d_1, 180, 180])#№+A*np.sin(omega*t) 
        dtheta_d = np.array([dtheta_d_1, 0, 0])

        
        theta_d = 180
        dtheta_d = 0
        
        # theta_d = 180
        # dtheta_d = 0
        
        
        control_vec = kp@(theta_d - theta[:3]) + kd@(dtheta_d - dtheta[:3]) #+ 3*np.ones(3)
        control[:3] = control_vec
        # control = np.zeros(4)
        
        print(theta, position)
        # print(robot.state.modules.forces) 
        # print(theta, robot.state.carriages.position - position_0)
        # print('\n')
        

        
except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    robot.set_torques(torques = [0, 0, 0, 0])
    pass

