from pyb import CAN  # type: ignore
from time import ticks_ms, ticks_us, sleep_us
from hardware.myactuator import MyActuator


can = CAN(1, CAN.NORMAL, extframe=False, prescaler=2, sjw=1, bs1=14, bs2=6)
can.setfilter(0, CAN.LIST16, 0, (320, 321, 322, 323))

act1 = MyActuator(device_id=321, can=can)
# act2 = MyActuator(device_id=322, can=can)

te = 0
tp = 0
t0 = ticks_us()
while True:
    t = ticks_us() - t0

    if t - te > 500:
        te = t
        # can.send(command, 0x141)   # seWnd a message with id 123
        t1 = ticks_us()
        state_1 = act1.set_torque(0)
        # state_2 = act2.set_torque(0)
        t2 = ticks_us()

    if t - tp > 20000:
        tp = t
        # print(t2 - t1, state_1, state_2)
        print(t2 - t1, state_1)
