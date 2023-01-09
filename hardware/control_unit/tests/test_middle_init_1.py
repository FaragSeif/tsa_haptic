from interface import RobotInterface
from time import sleep, perf_counter
import numpy as np


robot = RobotInterface()

#
control = np.zeros(4)

kp = 4.0
kd = 0.3


t0 = perf_counter()
# theta = np.zeros(3)
# theta_prev = np.zeros(3)
min_position = np.inf
max_angle = -np.inf
min_angle = np.inf

linear_offsets = []
angle_offsets = [] 
mid_angles = []
force = 100
try:
    t0 = perf_counter()
    module_id = 0
    while True:
        t = perf_counter() - t0
        control = np.zeros(4)
        control[0] = 30
        control[1] = 20
        control[2] = 20
        # control[3] = 20

        control[3] = 10*np.sign(np.sin(0.5*np.pi*t))
        

        
        # print(force)
        if force <= 1 and  1 <= t < 5:
            mid_angles.append(theta)
            
        if t>=5:
            theta_d = np.mean(mid_angles)
            kp = 6.0
            kd = 0.3
            control[3] = kp*(theta_d - theta) + kd*( - dtheta)

        if t>=8:
            break 
        
        theta = robot.state.modules.angles[3] 
        dtheta = robot.state.modules.speeds[3] 
        robot.set_torques(torques = control)
        robot.update_state()
        position = robot.state.carriages.position[0]
        # angle = robot.state.carriages.position[3] 
        force = robot.state.modules.forces[3]
        torque = robot.state.modules.torques[3]
        
except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    robot.set_torques(torques = [0, 0, 0, 0])
    pass

