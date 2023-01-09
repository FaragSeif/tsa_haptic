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
try:
    t0 = perf_counter()
    module_id = -1
    while True:
        t = perf_counter() - t0
        control = np.zeros(4)
        if module_id == 0:
            control = 6*np.ones(4)
        
        control[3] = 50
        
        control[module_id] = 8*np.sign(np.sin(0.5*np.pi*t))
        
        robot.set_torques(torques = control)
        robot.update_state()
        position = robot.state.carriages.position[module_id] 

        # if :
        if (t < 5) and (position <= min_position):
            min_position = position     
        
        if (t>=5) and (position >= min_position+1):
            angle = robot.state.modules.angles[module_id]
            if angle >= max_angle:
                max_angle = angle
                max_angle_pos = position-min_position
            if angle <= min_angle:
                min_angle = angle 
                min_angle_pos = position-min_position
            
            # print(min_angle, max_angle,  position-min_position, angle)
        if t>=10:
            print(f'Module {module_id + 1} is initialized')
            linear_offsets.append(min_position)
            angle_offset = (min_angle*min_angle_pos + max_angle*max_angle_pos)/(max_angle_pos + min_angle_pos)
            angle_offsets.append(angle_offset)
            lin_offset = min_position
            
            min_position = np.inf
            max_angle = -np.inf
            min_angle = np.inf
            
            module_id+=1
            t0 = perf_counter()
            # print(angle_offset)
            # break
            
        #     break
        
        if module_id == 3:
            print(f'Linear encoders are initialized, \noffsets are {linear_offsets}')
            print(f'Motor encoders are initialized, \noffsets are {angle_offsets}')
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

