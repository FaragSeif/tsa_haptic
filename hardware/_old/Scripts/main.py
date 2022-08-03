from pyb import CAN, ADC  # type: ignore
from time import ticks_ms, ticks_us, sleep_ms, ticks_diff
from struct import unpack, pack
from Encoder.hardware.quad_encoder import QuadEncoder  # type: ignore

print("started")
can = CAN(1, CAN.NORMAL, extframe=False, prescaler=2, sjw=1, bs1=14, bs2=6)
can.setfilter(0, CAN.LIST16, 0, (320, 321, 322, 323))

amp1 = ADC("A1")  # create an analog object from a pin
amp2 = ADC("A3")  # create an analog object from a pin

encoder1 = QuadEncoder(timer=1)  # channelA = "A8" , channelB = "A9"
encoder2 = QuadEncoder(timer=8)  # channelA = "C6" , channelB = "C7"


print("initialized")
current1 = 0
current2 = 0
command1 = b"\xA1" + 3 * b"\x00" + current1.to_bytes(2, "little", True) + 2 * b"\x00"
command2 = b"\xA1" + 3 * b"\x00" + current2.to_bytes(2, "little", True) + 2 * b"\x00"
stop = b"\x81" + 7 * b"\x00"

counts1, turns1, counts_mt1 = 0, 0, 0
counts2, turns2, counts_mt2 = 0, 0, 0

msg_frmt = "<hhh"
counts, speed, torque, temp, cmd = 0, 0, 0, 0, 0
t0 = ticks_us()
te = 0
tp = 0
dat = {}

sleep_ms(5000)
print("Loop is begining")

try:
    while True:
        t = ticks_us() - t0
        if t - te > 500:
            te = t
            # ============================================
            t1 = ticks_us()
            # ============================================
            can.send(command1, 321)
            if can.any(0):
                dev_id, _, _, msg = can.recv(0)
                torque, speed, counts = unpack(msg_frmt, msg[2:])

            can.send(command2, 322)
            if can.any(0):
                dev_id, _, _, msg = can.recv(0)
                torque, speed, counts = unpack(msg_frmt, msg[2:])

            force1 = amp1.read()
            force2 = amp2.read()

            counts_mt1 = encoder1.get_counter(overflow=True)
            counts_mt2 = encoder2.get_counter(overflow=True)
            # ============================================
            # t2 = ticks_us()
            # ============================================
            tim_diff = ticks_diff(ticks_us(), t1)
            counts1 = encoder1.counts
            counts2 = encoder2.counts

        if t - tp > 20000:
            tp = t
            # print(tim_diff)
            print(counts1, "   ", counts2)
except KeyboardInterrupt:
    can.send(stop, 321)
    can.send(stop, 322)
