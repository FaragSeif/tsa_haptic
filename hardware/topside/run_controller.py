from time import perf_counter
import select as select
import numpy as np
from model.kinematics import *
from controllers.qp_controller import get_torques, qp_posture_torques
from lcm_interface import handle, device_states, commands, controller_message, publish, subscribe, unsubscribe
# import lcm


commands_subscription = subscribe("commands")
device_subscription = subscribe("device")

# TODO:
# ADD GAINS AND FEEDFORWARD TORQUE IN COMMANDS
#

max_torque = 80

control = np.zeros(4)

commands.arm = False
commands.mode = 0
commands.desired_position = np.array([0, 0, 0, 0])
commands.desired_speed = np.zeros(4)
commands.desired_accel = np.zeros(3)

device_states.carriage_positions = np.zeros(3)
device_states.cartesian_position = np.array([0, 0, 60])
device_states.cartesian_velocity = np.zeros(3)
device_states.motor_angles = 20*np.ones(4)
device_states.motor_speeds = np.zeros(4)


try:
    t_upd = 0
    t0 = perf_counter()
    while True:
        t = perf_counter() - t0

        handle()
        # rfds, _, _ = select.select([lc.fileno()], [], [], 0.)
        # if rfds:
        # # t1 = perf_counter()
        #     lc.handle()

        theta = np.array(device_states.motor_angles)
        dtheta = np.array(device_states.motor_speeds)
        x = np.array(device_states.carriage_positions)

        if commands.arm:

            # ///////// DESIRED TRAJECTORY /////////////

            desired_position = commands.desired_position
            desired_speed = commands.desired_speed
            desired_accel = commands.desired_accel
            # desired_torque =

            # //////////// CONTROLLERS ///////////////

            # --- MODE 10: OPEN LOOP TORQUE ---

            controller_message.armed = True

            if commands.mode == 2:
                control = 0.015*dtheta

            if commands.mode == 10:
                control[0] = 7
                control[1] = 7
                control[2] = 7
                control[3] = -20

            # --- MODE 11: JOINT SPACE PD + TENSION ON LAST MODULE ---

            if commands.mode == 11:

                theta_d = np.array(desired_position)
                dtheta_d = np.array(desired_speed)

                kp = 6.0*np.eye(3)
                kd = 0.25*np.eye(3)

                joint_pd = kp@(theta_d[:3] - theta[:3]) + \
                    kd@(dtheta_d[:3] - dtheta[:3])  # + 3*np.ones(3)
                control[:3] = joint_pd
                control[3] = -20

            # --- MODE 12: JOINT SPACE PD  ---

            if commands.mode == 12:
                theta_d = desired_position
                dtheta_d = desired_speed

                kp = 6*np.eye(4)
                # kp[3,3] = 0
                kd = 0.25*np.eye(4)
                # kd[3,3] = 0.1

                joint_pd = kp@(theta_d - theta) + \
                    kd@(dtheta_d - dtheta)  # + 3*np.ones(3)

                motor_damping = np.array([0.015, 0.015, 0.015, 0.02])
                # motor_damping[3] = 0.02
                motor_friction = motor_damping*dtheta

                control = joint_pd + 0*motor_friction

            # --- MODE 21: QP BASED CARTESIAN SPACE PD + TENSION PRESERVING ---

            if commands.mode == 21:

                # ////////////////////////////////////////
                re_d = np.array(commands.desired_position[:3])
                dre_d = np.array(commands.desired_speed[:3])

                # ///////////////////////////////////////////
                cartesian_stiffness = np.diag([20.,
                                               20.,
                                               20.])

                cartesian_damping = np.diag([0.4,
                                             0.4,
                                             0.4])

                motor_damping = 0.015*np.eye(4)
                motor_damping[3] = 0.02

                motor_friction = motor_damping @ dtheta

                # ///////////////////////////////////////////

                re, jacobian_m, jacobian_d, jacobian_s = full_kinematics(
                    x, theta)
                dre = np.linalg.pinv(jacobian_d) @ dtheta

                # ///////////////////////////////////////////
                cartesian_pd = cartesian_stiffness @ (re_d - re) + \
                    cartesian_damping @ (dre_d - dre) + \
                    jacobian_d.T @ motor_friction

                # pretension = np.array([0.5, 0.5, 0.5, 0.5])

                # min_tension = jacobian_s @ motor_friction + pretension

                # print(jacobian_s)
                # weights = np.diag([1,1,1,1])
                torque_max = np.array([35, 35, 35, 60])
                torque_min = -torque_max
                tension_min = np.array([2, 2, 2, 2]) + \
                    jacobian_s @ motor_friction
                tension_max = np.array(
                    [80, 80, 80, 150]) + jacobian_s @ motor_friction

                control = qp_posture_torques(jacobian_d,
                                             jacobian_s,
                                             force=cartesian_pd,
                                             tension_bounds=[
                                                 tension_min, tension_max],
                                             torque_bounds=[torque_min, torque_max])

                # print(control)

        else:
            controller_message.armed = False
            control = np.zeros(4)

        clipped_control = np.clip(control, -max_torque, max_torque)

        if t - t_upd >= 1E-3:
            # print(control)
            controller_message.timestamp = int(t*1000)
            controller_message.mode = commands.mode
            controller_message.control_torques = clipped_control
            controller_message.error_flag = 0

            # device_states_message.cartesian_force

            publish("controller")
            # print(device_states_message.motor_angles)
            t_upd = t

except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    print('Controller is turning off...')
    controller_message.control_torques = np.zeros(4)
    controller_message.error_flag = -1
    publish("controller")
    print('Torques are setted to zero...')
    unsubscribe(device_subscription)
    print('[LCM] unsubscribed from device subscription')
    unsubscribe(commands_subscription)
    print('[LCM] unsubscribed from commands subscription')

    pass
