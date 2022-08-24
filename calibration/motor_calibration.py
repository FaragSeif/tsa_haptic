from cProfile import label
from xml.dom.minidom import ReadOnlySequentialNamedNodeMap
from click import style
from libs.can import CANSocket
from libs.myactuator import MyActuator
from time import perf_counter, sleep
import numpy as np
from math import pi, sin, cos
import matplotlib.pyplot as plt

# the serial port of device
# you may find one by examing /dev/ folder,
# this is usually devices ttyACM
serial_device = "ttyACM2"

# Initiate the can bus socket
can_bus = CANSocket(serial_port=serial_device)

# Initiate motor
pendulum = MyActuator(can_bus=can_bus)
pendulum.torque_constant = 1e-3


# Set the control loop timings
frequency = 500
sampling_time = 1 / frequency

kp = 0.5
kd = 0.08

m = 0.145
l = 0.075
g = 9.8


desired = [pi / 2, -pi / 2]
torque_trials = []
TT_average = []
N = 1

print("\n\nPress 's': to set zero point\nPress 'c': to start calibration")
ui = input()

if ui == "s":
    pendulum.set_zero(persistant=True)
else:
    try:
        for i in range(5):
            for angle in desired:
                thetad = angle
                dthetad = 0

                pos = []
                curr = []
                tim = []
                last_execution = 0
                control = 0
                count = 0
                # find the global time before intering control loop
                initial_time = perf_counter()
                while True:
                    time = perf_counter() - initial_time  # get actual time in secs
                    # /////////////////////////
                    # Get and parse motor state
                    # /////////////////////////
                    state = pendulum.state
                    theta = state["angle"]
                    dtheta = state["speed"]
                    torque = state["torque"]

                    # ///////////////////////////////////////////
                    # Update the control only on specific timings
                    # ///////////////////////////////////////////
                    if (time - last_execution) >= sampling_time:
                        last_execution = time
                        # YOUR CONTROLLER GOES HERE
                        e, de = thetad - theta, dthetad - dtheta
                        control = kp * e + kd * de
                        # print(
                        #     f"Motor angle data: {round(theta, 5)}\nDesired angle: {round(thetad, 5)}\n\n",
                        #     end="    \r",
                        #     flush=True,
                        # )
                        count += 1

                    pos.append(theta)
                    tim.append(time)
                    curr.append(torque)
                    pendulum.set_torque(control)

                    if count >= 2000:
                        break

                theta_samples = pos[-500:-1]
                current_samples = curr[-500:-1]
                torque_trials.append(
                    np.mean(m * g * l * np.sin(theta_samples))
                    / np.mean(current_samples)
                )

                print(f"Trial {N} constant = {torque_trials[-1]}\n")
                N += 1

            TT_average.append((torque_trials[-1] + torque_trials[-2]) / 2)

        for i in range(100):
            pendulum.set_torque(0)
        print(f"Average values from each trial {TT_average}\n\n")
        print(f"Final Torque Average = {np.mean(TT_average)}")

    except KeyboardInterrupt:
        for i in range(100):
            pendulum.set_torque(0)
