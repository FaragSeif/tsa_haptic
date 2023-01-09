from interface import RobotInterface
from time import sleep, perf_counter
import numpy as np
from misc import fit_quadratic

np.savetxt('calibration/angle_calibration.conf', np.zeros(4))
np.savetxt('calibration/position_calibration.conf', np.zeros(3))
np.savetxt('calibration/force_calibration.conf', np.zeros(4)) 

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
    t0 = perf_counter()
    t = perf_counter() - t0
    module_id = -1
    print('Initialization begin')
    
    print('Pushing to zero position...')
    while t<=5:
        t = perf_counter() - t0
        
        # control = np.zeros(4)
        robot.set_torques(torques = control)
        robot.update_state()
        
        angle = robot.state.modules.angles
        speed = robot.state.modules.speeds
        force = robot.state.modules.forces
        position = robot.state.carriages.position

        # 1.MOVE TO INIT POSITION
            
        # if module_id == -1:
        control[0] = -0.0*speed[0] + 3#*np.sign(np.sin(0.5*np.pi*t))
        control[1] = -0.0*speed[1] + 3#*np.sign(np.sin(0.5*np.pi*t))
        control[2] = -0.0*speed[2] + 3#*np.sign(np.sin(0.5*np.pi*t))
        control[3] = 70 - 0.3*speed[3] 
            
        if t>=4.9:
            control[:3] = np.zeros(3)
            
    
    # //////////////////////////////
    # INITIALIZE THREE FIRST MODULES 
    # //////////////////////////////
    print('Initializing modules....')
    for module_id in range(3):
        print(f'Initialization of module №{module_id+1}')
        angles = []
        positions = []
        t0 = perf_counter()
        t = perf_counter() - t0
        control = np.zeros(4)
        while t<=10:
            t = perf_counter() - t0
            robot.set_torques(torques = control)
            robot.update_state()
    
            angle = robot.state.modules.angles
            speed = robot.state.modules.speeds
            force = robot.state.modules.forces
            position = robot.state.carriages.position
            
            control = np.zeros(4)
            if module_id == 0:
                control[1] = -0.02*speed[1] + 3
                control[2] = -0.02*speed[2] + 3
            
            if module_id == 1:
                control[2] = -0.02*speed[2] + 5
                
            if module_id == 2:
                control[1] = -0.02*speed[2] + 5

            control[module_id] = -0.02*speed[module_id] - 10*np.sign(np.sin(0.5*np.pi*t))
            control[3] = 70 - 0.2*speed[3] 
       
            positions.append(position[module_id])
            angles.append(angle[module_id])
            sleep(0.002)
            
        position_offset = np.min(positions)
        positions = np.array(positions)
        angles = np.array(angles)
        indeces = positions >= position_offset + 0.1
        theta = angles[indeces]
        contraction = positions[indeces]
        A = np.array([theta**2, theta, np.ones(len(theta))]).T
        y = contraction - position_offset
        coefficients = np.linalg.pinv(A)@y
        a, b, c = coefficients
        angle_offset = -b/(2*a)
        
        angle_offsets.append(angle_offset)
        linear_offsets.append(position_offset)
        t0 = perf_counter()
        print(f'Initialization module №{module_id+1} is initialized')
        print(angle_offset, position_offset)
    # /////////////////////////////////////////////////////////////////////////////////
    
    # /////////////////////////
    # INITIALIZE MIDDLE MODULE
    # /////////////////////////
    
    print(f'Initialization of pulling module')
    t0 = perf_counter()
    t = perf_counter() - t0
    angles = []
    forces = []
    while t<=20:
        t = perf_counter() - t0
        robot.set_torques(torques = control)
        robot.update_state()
        
        angle = robot.state.modules.angles 
        speed = robot.state.modules.speeds 
        force = robot.state.modules.forces

            
        control[:3] = 10*np.ones(3)
        control[3] = -0.5*speed[3] - 70*np.sign(np.sin(0.25*np.pi*t))
        if t>5:
            angles.append(angle[3])
            forces.append(force[3])
        sleep(0.002)
        
    # # position_offset = np.min(positions)
    force_offset = np.min(forces)
    forces = np.array(forces) - force_offset
    angles = np.array(angles)
    indeces = (forces >= 10)
    theta = angles[indeces]
    tension = forces[indeces] 
    A = np.array([theta**2, theta, np.ones(len(theta))]).T
    y = tension
    
    coefficients = np.linalg.pinv(A)@y
    a, b, c = coefficients
    angle_offset = -b/(2*a)
    
    print(f'Pulling module is initialized')
    # print(angle_offset, position_offset)
    angle_offsets.append(angle_offset)
    
    
    # print('Over')
    # print(angle_offsets, linear_offsets)
    np.savetxt('calibration/angle_calibration.conf', angle_offsets)
    np.savetxt('calibration/position_calibration.conf', linear_offsets)
    print('Initialization is over!')
    print(f'Linear encoders offsets are {linear_offsets}')
    print(f'Motor encoders offsets are {angle_offsets}')
    robot.set_torques(torques = [0, 0, 0, 0])
    # robot.__del__()
    # TODO:    # 
        
        
    # //////////////////////////////////
    theta_0 = angle_offsets
    control = np.zeros(4)
    kp = 6.0*np.eye(4)
    kd = 0.3*np.eye(4)

    t0 = perf_counter()
    t = perf_counter() - t0
    while t < 3:
        t = perf_counter() - t0
        robot.set_torques(torques = control)
        robot.update_state()
        
        theta = robot.state.modules.angles - theta_0
        dtheta = robot.state.modules.speeds

        theta_d = 0
        dtheta_d = 0
        control = kp@(theta_d - theta) + kd@(dtheta_d - dtheta)

    # ///////////////////////////////////////

    control = np.zeros(4)
    N = 500
    force_bias = np.zeros(4)
    robot.set_torques(torques = control)
    robot.update_state()
    for i in range(N):
        force_bias += robot.state.modules.forces/N
    np.savetxt('calibration/force_calibration.conf', force_bias) 
    print(force_bias)
    # ///////////////////////////////////////
    
    
    # # //////////////////////////////////

    # control = np.zeros(4)
    # kp = 6.0*np.eye(4)
    # kd = 0.3*np.eye(4)
    
    # t0 = perf_counter()
    # t = perf_counter() - t0
    # while t < 3:
    #     t = perf_counter() - t0
    #     robot.set_torques(torques = control)
        
    #     robot.update_state()
    #     theta = robot.state.modules.angles
    #     dtheta = robot.state.modules.speeds

    #     theta_d = 100
    #     dtheta_d = 0
    #     control = kp@(theta_d - theta) + kd@(dtheta_d - dtheta)
    #     control[3] = 40 - 0.1*dtheta[3]

    # # ///////////////////////////////////////
    
    # robot.set_torques(torques = [0, 0, 0, 0])
    # # exit()
    
        
except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    robot.set_torques(torques = [0, 0, 0, 0])
    robot.__del__()
    pass

