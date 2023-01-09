from time import perf_counter
from lcm_interface import device_states, controller_states, commands, handle, subscribe, unsubscribe
from model.kinematics import *

# TODO:
# REWRITE WITH CURSES

SAMPLING = 30
UPDATE_RATE = 1000/SAMPLING

commands_subscription = subscribe("commands")
device_subscription = subscribe("device")
controller_subscription = subscribe("controller")


try:
    # for i in range(50):
    #     print(50*' ')
    # print(50*'\033[A', end="\r", flush=True)

    ts = 0
    t0 = perf_counter()
    print('Printer is started...')
    while True:
        t = perf_counter() - t0

        handled = handle(blocking=False)

        if t - ts >= 1/UPDATE_RATE and handled:
            re, jacobian_m, jacobian_d, jacobian_s = full_kinematics(
                device_states.carriage_positions, device_states.motor_angles)
            cartesian_force = -jacobian_m.T @ device_states.tensions

            print(f'\n{15*"/"} DEVICE STATE {15*"/"}')
            print(f'MODULES: ' + 20*' ',
                  #   f'\n  positions: {robot.state.modules.angles}'+ 20*' ',

                  f'\n  ticks (ms): {device_states.timestamp}' + 20*' ',
                  f'\n  angles: {np.round(device_states.motor_angles,3)}' + 20*' ',
                  f'\n  speeds: {np.round(device_states.motor_speeds, 3)}' + 20*' ',
                  f'\n  torques: {np.round(device_states.motor_torques,3)}' + 20*' ',
                  f'\n  position: {np.round(device_states.carriage_positions,3)}' + 20*' ',
                  f'\n  end effector pos: {np.round(re,3)}' + 20*' ',
                  f'\n  tensions: {np.round(device_states.tensions,3)}' + 20*' ',
                  f'\n  end effector force: {np.round(cartesian_force,3)}' + \
                  20*' ',
                  end=11*" " + "\n", flush=True)
            print(f'{52*"/"}\n')
            print(13*'\033[A', end="\r", flush=True)

            ts = t

except KeyboardInterrupt:
    # print(10*'\n')
    unsubscribe(controller_subscription)
    print('[LCM] unsubscribed from controller subscription')
    unsubscribe(device_subscription)
    print('[LCM] unsubscribed from device subscription')
    unsubscribe(commands_subscription)
    print('[LCM] unsubscribed from commands subscription')
    print(100*'\033[A')
