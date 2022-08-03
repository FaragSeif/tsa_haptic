from hardware.quad_encoder import QuadEncoder
import time

encoder = QuadEncoder(timer=5, pins=["A0", "A1"])

time_it = time.ticks_us
# get_counts = encoder.get_counter

counts, turns, counts_mt = 0, 0, 0

t1, t2 = 0, 0
# try:
t0 = time_it()
te = 0

while True:
    t = time_it() - t0
    if t - te > 1000:
        t1 = time_it()
        counts_mt = encoder.get_counter(overflow=True)
        t2 = time_it()
        counts = encoder.counts
        print(t2 - t1, counts, counts_mt)
