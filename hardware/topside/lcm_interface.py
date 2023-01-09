

import lcm
from types import SimpleNamespace
from protocol import device_states_t, controller_states_t, commands_t
import select as select

# TODO: MOVE EVERYTHING TO CLASS

address = "udpm://239.255.76.67:7667?ttl=1"
# address = ""
lc = lcm.LCM(address)

device_states = SimpleNamespace()
controller_states = SimpleNamespace()
commands = SimpleNamespace()


def update_device(channel, data):
    device_data = device_states_t.decode(data)
    slots = device_states_t.__slots__
    for slot in slots:
        vars(device_states)[slot] = getattr(device_data, slot)


def update_controller(channel, data):
    controller_data = controller_states_t.decode(data)
    slots = controller_states_t.__slots__
    for slot in slots:
        vars(controller_states)[slot] = getattr(controller_data, slot)


def update_commands(channel, data):
    commands_data = commands_t.decode(data)
    slots = commands_t.__slots__
    for slot in slots:
        vars(commands)[slot] = getattr(commands_data, slot)


update_dict = {"commands": update_commands,
               "device": update_device,
               "controller": update_controller}


def subscribe(channel):

    subscription = lc.subscribe(channel, update_dict[channel])
    print(f'[LCM] Subscribed to "{channel}" channel')
    return subscription


def unsubscribe(subscription):
    lc.unsubscribe(subscription)


commands_message = commands_t()
device_message = device_states_t()
controller_message = controller_states_t()

message_dict = {"commands": commands_message,
                "device": device_message,
                "controller": controller_message}


def publish(channel):
    lc.publish(channel, message_dict[channel].encode())


def handle(blocking=False):
    handled = False
    if not blocking:
        rfds, _, _ = select.select([lc.fileno()], [], [], 0.)
        if rfds:
            lc.handle()
            handled = True
    else:
        lc.handle()
        handled = True
    return handled


# def update_device(channel, data):
#     device_data = device_states_t.decode(data)
#     device_states.mode = device_data.mode
#     device_states.timestamp = device_data.timestamp
#     device_states.cartesian_position = device_data.cartesian_position
#     device_states.cartesian_velocity = device_data.cartesian_velocity
#     device_states.carriage_positions = device_data.carriage_positions
#     device_states.motor_angles = device_data.motor_angles
#     device_states.motor_speeds = device_data.motor_speeds
#     device_states.motor_torques = device_data.motor_torques
#     device_states.tensions = device_data.tensions
#     device_states.cartesian_force = device_data.cartesian_force
#     device_states.error_flags = device_data.error_flags
#     device_states.armed = device_data.armed


# def update_commands(channel, data):
#     commands_data = commands_t.decode(data)
#     commands.timestamp = commands_data.timestamp
#     commands.mode = commands_data.mode
#     commands.desired_position = commands_data.desired_position
#     commands.desired_speed = commands_data.desired_speed
#     commands.desired_accel = commands_data.desired_accel
#     commands.arm = commands_data.arm

# channel = "measurements"


# commands_subscription = lc.subscribe("commands", update_commands)
# device_subscription = lc.subscribe("device", update_device)
# controller_subscription = lc.subscribe("controller", update_controller)

# commands_subscription = lc.subscribe("commands", update_commands)
