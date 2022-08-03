from pyb import CAN  # type: ignore
from time import ticks_ms, ticks_us
from struct import unpack

print("started")
can = CAN(1, CAN.NORMAL, extframe=False, prescaler=2, sjw=1, bs1=14, bs2=6)

# can.init(CAN.NORMAL)
print("initialized")

can.setfilter(0, CAN.LIST16, 0, (320, 321, 322, 323))
current = 0
command = b"\xA1" + 3 * b"\x00" + current.to_bytes(2, "little", True) + 2 * b"\x00"
# command = b'\x01\x02\x03\x04\x05\x06\x07\x08'


# command = b"\xA1" + 7 * b"\x00"
# buf = bytearray(8)
# lst = [0, 0, 0, memoryview(buf)]
# No heap memory is allocated in the following call
print("Loop is begining")

msg_frmt = "<hhh"
counts, speed, torque, temp, cmd = 0, 0, 0, 0, 0
t0 = ticks_us()
te = 0
tp = 0
while True:
    t = ticks_us() - t0
    if t - te > 500:
        te = t
        # can.send(command, 0x141)   # seWnd a message with id 123
        t1 = ticks_us()
        can.send(command, 321)  # seWnd a message with id 123
        # sleep_us(100)

        if can.any(0):
            dev_id, _, _, msg = can.recv(0)
            torque, speed, counts = unpack(msg_frmt, msg[2:])
            t2 = ticks_us()

    if t - tp > 20000:
        tp = t
        print(t2 - t1, torque, speed, counts)
