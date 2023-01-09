from interface import RobotInterface
from types import SimpleNamespace
import numpy as np
from time import perf_counter
import select as select
from model.kinematics import *
import lcm
from protocol import device_states_t, controller_states_t

address = ""
lc = lcm.LCM(address)
robot = RobotInterface()

device_states = SimpleNamespace()
controller_states = SimpleNamespace()


def update_controller(channel, data):
    controller_data = controller_states_t.decode(data)
    controller_states.timestamp = controller_data.timestamp
    controller_states.mode = controller_data.mode
    controller_states.control_torques = controller_data.control_torques
    controller_states.error_flag = controller_data.error_flag
    controller_states.armed = controller_data.armed

controller_subscription = lc.subscribe("controller", update_controller)
device_states_message = device_states_t()
lc.publish("device", device_states_message.encode())

mode = 0
control_torques = np.zeros(4)
cartesian_position = np.zeros(3)
cartesian_velocity = np.zeros(3)

error_flag = 0
t_st_upd = 0
try:
    t0 = perf_counter()
    while True:
        t = perf_counter() - t0

        rfds, _, _ = select.select([lc.fileno()], [], [], 0.)
        if rfds:
            lc.handle()
            mode = controller_states.mode
            control_torques = controller_states.control_torques
            # ind =

        # control =
        # control_torques = [0, 0, 0, 0]


        if t - t_st_upd >= 1.0E-3:
            robot.set_torques(torques=control_torques)
            robot.update_state()
            # cartesian_position, jacobian_m, jacobian_d, jacobian_s = full_kinematics(
            #     robot.state.carriages.position, robot.state.modules.angles)
            # cartesian_velocity = np.linalg.pinv(
            #     jacobian_d) @ robot.state.modules.speeds
            device_states_message.mode
            device_states_message.timestamp = int(1000*t)
            device_states_message.carriage_positions = robot.state.carriages.position
            device_states_message.motor_angles = robot.state.modules.angles
            device_states_message.motor_speeds = robot.state.modules.speeds
            device_states_message.motor_torques = robot.state.modules.torques
            device_states_message.tensions = robot.state.modules.forces

            device_states_message.cartesian_position = cartesian_position
            device_states_message.cartesian_velocity = cartesian_velocity
            # print(cartesian_position)
            # device_states_message.cartesian_force

            device_states_message.error_flags = error_flag
            device_states_message.armed = True

            lc.publish("device", device_states_message.encode())
            # print(device_states_message.motor_angles)
            t_st_upd = t
  


except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    lc.unsubscribe(controller_subscription)
    print('[LCM] unsubscribed from controller subscription')
    robot.set_torques(torques=[0, 0, 0, 0])
    # pass
