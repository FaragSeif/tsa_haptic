from Encoder.hardware.quad_encoder import QuadEncoder  # type: ignore
import time

encoder1 = QuadEncoder(timer=1)  # channelA = "A8" , channelB = "A9"
encoder2 = QuadEncoder(timer=8)  # channelA = "C6" , channelB = "C7"

time_it = time.ticks_us

counts1, turns1, counts_mt1 = 0, 0, 0
counts2, turns2, counts_mt2 = 0, 0, 0

t1, t2 = 0, 0

t0 = time_it()
te = 0

time.sleep_ms(5000)  # wait for init message

while True:
    t = time_it() - t0
    if t - te > 1000:
        t1 = time_it()
        counts_mt1 = encoder1.get_counter(overflow=True)
        counts_mt2 = encoder2.get_counter(overflow=True)
        t2 = time_it()
        counts1 = encoder1.counts
        counts2 = encoder2.counts
        print(counts1, "   ", counts2)
