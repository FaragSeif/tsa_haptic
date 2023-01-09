from interface import RobotInterface
from time import sleep, perf_counter
import numpy as np
from model.kinematics import *
from control_unit.tests.qp_controller import get_torques

robot = RobotInterface()

#
control = np.zeros(4)
control[3] = 50

kp = 6.0*np.eye(3)
kd = 0.3*np.eye(3)


t0 = perf_counter()


cartesian_stiffnes = np.diag([3, 
                              3.,
                              3.])

cartesian_damping =  np.diag([0.2,
                              0.2,
                              0.2])

try:
    while True:
        t = perf_counter() - t0
        robot.set_torques(torques = control)
        
        # print(theta)
        robot.update_state()
        theta = robot.state.modules.angles
        position = robot.state.carriages.position
        dtheta = robot.state.modules.speeds
        tensions = robot.state.modules.forces
        cartesian_position, jacobian_m, jacobian_d, jacobian_s = full_kinematics(position, theta)
        cartesian_speed = np.linalg.pinv(jacobian_d) @ dtheta
        cartesian_force = jacobian_d.T @ tensions
        

        # print()
        A, omega = 30, 5.0
        # A, omega = 0, 0
        # theta_d = 
        x_d = A*np.sin(omega*t) 
        dx_d = A*omega*np.cos(omega*t)
        y_d = 10 + A*np.cos(omega*t) 
        dy_d = -A*omega*np.sin(omega*t)
        # dtheta_d_1 = A*omega*np.cos(omega*t)
        # theta_d =  np.array([theta_d_1, 180, 180])#№+A*np.sin(omega*t) 
        # dtheta_d = np.array([dtheta_d_1, 0, 0])
        
        
        # desired_pos = np.array([x_d, y_d, 80]) 
        # desired_speed = np.array([dx_d, dy_d, 0]) 
        
        desired_pos = np.array([0, 0, 60]) 
        desired_speed = np.array([0, 0, 0]) 
        
        cartesian_pd = cartesian_stiffnes@(desired_pos - cartesian_position) + cartesian_damping @ (desired_speed - cartesian_speed)
        
        
        qp_control = get_torques(jacobian_d, jacobian_s, cartesian_pd, pretension=1)

        control = np.clip(qp_control, -80, 80)
        
        
        
        
        # control = np.zeros(4)
        print(cartesian_position)
        # print(qp_control)
        # print(qp_control)
        # print(cartesian_speed)
        # print(cartesian_force)
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

