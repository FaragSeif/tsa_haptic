from time import perf_counter,sleep
from lcm_interface import commands_message, publish
from model.kinematics import inverse_kinematics, x2theta, L, r
import numpy as np 

commands_message.mode = 0


T = 0.5

try:
    t_upd = 0
    t0 = perf_counter()
    while True:
        t = perf_counter() - t0
        # print(t)
        if t - t_upd >= 1E-2:
            point_ind = int((t//T) % 5)
          
            # # ////////////////
            # # CIRCULAR MOTION
            # # ////////////////
            # radius = 20
            # omega = 2
            # A = radius
            # cartesian_pos = [-5+A*np.sin(omega*t),
            #                  A*np.cos(omega*t),
            #                  10]
            
            # # ////////////////
            # # SPIRAL MOTION
            # # ////////////////
            # omega = 2
            # A = 0.5*radius*(1 + np.sin(omega*t/6.5))+2
            # cartesian_pos = [-5+A*np.sin(omega*t),
            #                  A*np.cos(omega*t),
            #                  10]

            
            # # ////////////////
            # # ONE AXIS MOTION
            # # ////////////////
            # # 
            # # X axis
            # final_point = np.array([-35, -10, 15])
            # initial_point = np.array([28, -10, 15])
            # # # Y axis
            # # initial_point = np.array([0, -30, 15])
            # # final_point = np.array([0, 30, 15])
            # Z axis
            initial_point = np.array([0, -2, 0])
            final_point = np.array([8, -20, 70])
            
            phases = np.array([0, 0, 0])
            A = (final_point - initial_point)/2
            middle_point = (final_point + initial_point)/2
            omega = 2.0
            cartesian_pos = middle_point + A*np.sin(omega*t + phases)

            # //////////////////
            # INVERSE KINEMATICS 
            contraction = inverse_kinematics(cartesian_pos)
            motor_angle = x2theta(contraction, np.array(L), np.array(r))
            
            # /////////////
            # PD CONTROLLER
            commands_message.arm = True
            commands_message.mode = 12
            commands_message.desired_position[:3] = motor_angle[:3]
            commands_message.desired_speed[:3] = np.zeros(3)
            
            publish("commands")
            
            t_upd = t


except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    print('Controller is turning off...')
    commands_message.arm = True    
    commands_message.mode = 12
    commands_message.desired_position = 4*[0]
    commands_message.desired_speed = 4*[0]
    publish("commands")

