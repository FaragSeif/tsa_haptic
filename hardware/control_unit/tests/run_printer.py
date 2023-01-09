import lcm
from protocol import device_states_t, controller_states_t, commands_t
from types import SimpleNamespace
from time import perf_counter, sleep
import select as select 
import numpy as np 


# address = "udpm://239.255.76.67:7667?ttl=1"
address = ""
lc = lcm.LCM(address)

SAMPLING = 5
UPDATE_RATE = 1000/SAMPLING


device_states = SimpleNamespace()
controller_states = SimpleNamespace()
commands = SimpleNamespace()

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

def update_controller(channel, data):
    controller_data = controller_states_t.decode(data)
    controller_states.timestamp = controller_data.timestamp
    controller_states.mode = controller_data.mode
    controller_states.control_torques = controller_data.control_torques
    controller_states.error_flag = controller_data.error_flag
    controller_states.armed = controller_data.armed


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
controller_subscription = lc.subscribe("controller", update_controller)


ts = 0
# T = [0]
ticks = 0 
i = 0 
N = 1000000 
# velocity_data = np.zeros((N, 3))

lc.handle()
sleep(0.1)
try:
    t0 = perf_counter()
    print('Printer is started...')
    while i<=N:
        t = perf_counter() - t0 

        rfds, _, _ = select.select([lc.fileno()], [], [], 0.)
        if rfds:
            lc.handle()
            
            if t - ts >= 1/UPDATE_RATE:
                i+=1

                print(device_states.motor_angles )
                # print(device_states.cartesian_position )
                print(device_states.carriage_positions )
                print(controller_states.control_torques)
                print(commands.mode, commands.arm)
            
                ts = t

except KeyboardInterrupt:
    lc.unsubscribe(device_subscription)
    lc.unsubscribe(controller_subscription)
    lc.unsubscribe(commands_subscription)