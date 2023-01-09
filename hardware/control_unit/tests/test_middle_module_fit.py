from interface import RobotInterface
from time import sleep, perf_counter
import numpy as np


np.savetxt('angle_calibration.conf', np.zeros(4))
np.savetxt('position_calibration.conf', np.zeros(3))


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
done = False
try:
    module_id = 0
    print('Initialization begin')
    angles = []
    forces = []
    t = 0 
    
    force_bias = np.zeros(4)
    
    for i in range(200):
        sleep(0.002)
        robot.set_torques(torques = control)
        robot.update_state()

        force_bias += robot.state.modules.forces/200
        
    # print(force_bias)
    
    t0 = perf_counter()
    while True:
        t = perf_counter() - t0
    #     # if done:
    #     #     break
        
    #     # control = np.zeros(4)
        robot.set_torques(torques = control)
        robot.update_state()
        
        angle = robot.state.modules.angles 
        speed = robot.state.modules.speeds 
        force = robot.state.modules.forces - force_bias

            
        control[:3] = 10*np.ones(3)
        control[3] = -0.5*speed[3] - 50*np.sign(np.sin(0.2*np.pi*t))
        

        print(force[3])
        if (t <= 15):
            angles.append(angle[3])
            forces.append(force[3])
        if t>15:
            break


        sleep(0.002)
        
    # # position_offset = np.min(positions)
    forces = np.array(forces)
    angles = np.array(angles)
    indeces = (forces >= 5)
    theta = angles[indeces]
    tension = forces[indeces]
    A = np.array([theta**2, theta, np.ones(len(theta))]).T
    y = tension
    
    coefficients = np.linalg.pinv(A)@y
    a, b, c = coefficients
    theta_0 = -b/(2*a)
    # print(theta_0)
    kp, kd = 2, 0.3
    
    while True:
        robot.set_torques(torques = control)
        robot.update_state()
        
        angle = robot.state.modules.angles[3] 
        speed = robot.state.modules.speeds[3] 
        force = robot.state.modules.forces[3]
        
        control = kp*(theta_0 - angle) - kd*speed
            
        print(angle, theta_0)


        
except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:


    robot.set_torques(torques = [0, 0, 0, 0])
    pass

