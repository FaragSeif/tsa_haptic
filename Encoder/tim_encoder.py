import pyb  # type: ignore
import time

# def init_encoder(pin_a, pin_b, timer):
#     pyb.Pin(pin_a, pyb.Pin.AF_PP,
#             pull=pyb.Pin.PULL_NONE, af=pyb.Pin.AF1_TIM1)

#     pyb.Pin(pin_b, pyb.Pin.AF_PP,
#             pull=pyb.Pin.PULL_NONE, af=pyb.Pin.AF1_TIM1)

#     enc_timer = pyb.Timer(timer, prescaler=0, period=65535)
#     enc_timer.channel(1, pyb.Timer.ENC_AB)

#     return enc_timer

# input()

pin_a = pyb.Pin("PA8", pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE, af=pyb.Pin.AF1_TIM1)
pin_b = pyb.Pin("PA9", pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE, af=pyb.Pin.AF1_TIM1)

enc_1_timer = pyb.Timer(1, prescaler=0, period=65535)
enc_1_timer.channel(1, pyb.Timer.ENC_AB)


pyb.Pin("PA6", pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE, af=pyb.Pin.AF2_TIM3)
pyb.Pin("PA7", pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE, af=pyb.Pin.AF2_TIM3)

enc_2_timer = pyb.Timer(3, prescaler=0, period=65535)
enc_2_timer.channel(1, pyb.Timer.ENC_AB)


while True:
    time.sleep_us(100)
    counts_1 = enc_1_timer.counter()
    time.sleep_us(100)
    counts_2 = enc_2_timer.counter()
    # print(counts_1)
    print(counts_1, counts_2)
