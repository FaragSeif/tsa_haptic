from interface import RobotInterface
from time import sleep, perf_counter
import numpy as np


robot = RobotInterface()

#
control = np.zeros(4)
control[3] = 20

kp = 6.0*np.eye(4)
kd = 0.3*np.eye(4)


t0 = perf_counter()

# theta_prev = np.zeros(3)
try:
    while True:
        t = perf_counter() - t0
        robot.set_torques(torques = control)
        
        robot.update_state()
        theta = robot.state.modules.angles
        dtheta = robot.state.modules.speeds

        theta_d = 0
        dtheta_d = 0
        
        control = kp@(theta_d - theta) + kd@(dtheta_d - dtheta)

        
except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    robot.set_torques(torques = [0, 0, 0, 0])
    pass

