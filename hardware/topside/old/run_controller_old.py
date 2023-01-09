from time import perf_counter
import lcm
from protocol import device_states_t, controller_states_t, commands_t
import numpy as np
from model.kinematics import *
from controllers.qp_controller import get_torques
from types import SimpleNamespace
import select as select

address = "udpm://239.255.76.67:7667?ttl=1"
# address = ""
lc = lcm.LCM(address)

device_states = SimpleNamespace()
commands = SimpleNamespace()


# TODO:
# MOVE LCM RELATED STUFF TO SEPARATE FILE
# IMPLEMENT DESIRED TRAJECTORY IN COMMANDS

def update_device(channel, data):
    device_data = device_states_t.decode(data)
    device_states.mode = device_data.mode
    device_states.timestamp = device_data.timestamp
    device_states.cartesian_position = device_data.cartesian_position
    device_states.cartesian_velocity = device_data.cartesian_velocity
    device_states.carriage_positions = device_data.carriage_positions
    device_states.motor_angles = device_data.motor_angles
    device_states.motor_speeds = device_data.motor_speeds
    device_states.motor_torques = device_data.motor_torques
    device_states.tensions = device_data.tensions
    device_states.cartesian_force = device_data.cartesian_force
    device_states.error_flags = device_data.error_flags
    device_states.armed = device_data.armed


def update_commands(channel, data):
    commands_data = commands_t.decode(data)
    commands.timestamp = commands_data.timestamp
    commands.mode = commands_data.mode
    commands.desired_position = commands_data.desired_position
    commands.desired_speed = commands_data.desired_speed
    commands.desired_accel = commands_data.desired_accel
    commands.arm = commands_data.arm
# channel = "measurements"


commands_subscription = lc.subscribe("commands", update_commands)
device_subscription = lc.subscribe("device", update_device)

controller_message = controller_states_t()
lc.publish("controller", controller_message.encode())

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

        rfds, _, _ = select.select([lc.fileno()], [], [], 0.)
        if rfds:
            lc.handle()

        if commands.arm:

            # ////////////////
            # OPEN LOOP TORQUE
            # ////////////////

            controller_message.armed = True
            if commands.mode == 10:
                control[0] = 7
                control[1] = 7
                control[2] = 7
                control[3] = 20

            # ////////////////////////////////////////
            # JOINT SPACE PD + TENSION ON LAST MODULE
            # ///////////////////////////////////////

            if commands.mode == 11:
                kp = 6.0*np.eye(3)
                kd = 0.25*np.eye(3)
                theta = np.array(device_states.motor_angles[:3])
                dtheta = np.array(device_states.motor_speeds[:3])
                theta_d = np.array([180, 200, 200])
                joint_pd = kp@(theta_d - theta) + \
                    kd@(- dtheta)  # + 3*np.ones(3)
                control[:3] = joint_pd
                control[3] = 50

            # /////////////////////////////////////////////////
            # QP BASED CARTESIAN SPACE PD + TENSION PRESERVING
            # ////////////////////////////////////////////////

            if commands.mode == 12:
                cartesian_stiffnes = np.diag([4,
                                              4,
                                              4.])

                cartesian_damping = np.diag([0.2,
                                            0.2,
                                            0.2])

                cartesian_position, jacobian_m, jacobian_d, jacobian_s = full_kinematics(
                    device_states.carriage_positions, device_states.motor_angles)
                cartesian_velocity = np.linalg.pinv(
                    jacobian_d) @ device_states.motor_speeds

                desired_position = np.array([0, 0, 90])
                desired_speed = np.array([0, 0, 0])

                cartesian_pd = cartesian_stiffnes@(desired_position - cartesian_position) + \
                    cartesian_damping @ (desired_speed - cartesian_velocity)

                # cartesian_pd = cartesian_stiffnes@(commands.desired_position[:3] - cartesian_position) + \
                #                cartesian_damping @ (commands.desired_speed[:3] - cartesian_velocity)
                control = get_torques(
                    jacobian_d, jacobian_s, cartesian_pd, pretension=0.1)
                # print(control)

            # ///////////////
            # CLIP CONTROLLER
            # ///////////////
            # control = [5, 5, 5, 20]

        else:
            controller_message.armed = False
            control = np.zeros(4)

            # clipped_control = control

        clipped_control = np.clip(control, -80, 80)
        if t - t_upd >= 1E-3:
            # print(control)
            controller_message.timestamp = int(t*1000)
            controller_message.mode = commands.mode
            controller_message.control_torques = clipped_control
            controller_message.error_flag = 0

            # device_states_message.cartesian_force

            lc.publish("controller", controller_message.encode())
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
    lc.publish("controller", controller_message.encode())
    print('Torques are setted to zero...')
    lc.unsubscribe(device_subscription)
    print('[LCM] unsubscribed from device subscription')
    lc.unsubscribe(commands_subscription)
    print('[LCM] unsubscribed from commands subscription')

    pass
