from pyb import Pin, Timer  # type: ignore

# TODO:
# Maximaze speed

# Test different timers
# tim1
# tim2 -     ['A0', 'A1']
# tim4 - ok  ['B7', 'B6']
# tim5 - ok ['A0', 'A1']
# tim8 - ok ['C6', 'C7']
# remove timer definition from class
# remove timer labels

# create pyboard tim encoders class


class QuadEncoder:
    def __init__(self, timer=1):

        self.af_table = {
            1: [Pin.AF1_TIM1, "A8", "A9"],  # Advanced Control timer (Double speed)
            2: [Pin.AF1_TIM2, "A0", "A1"],  # Cannot work with Tim5
            3: [Pin.AF2_TIM3, "A6", "A7"],
            4: [Pin.AF2_TIM4, "B6", "B7"],
            5: [Pin.AF2_TIM5, "A0", "A1"],  # Cannot work with Tim2
            8: [Pin.AF3_TIM8, "C6", "C7"],  # Advanced Control timer (Double speed)
        }

        if timer not in self.af_table.keys():
            print(f"Timer TIM{timer} is not found...")
            self.timer = 1
        else:
            self.timer = timer

        print(f"Timer is set to TIM{self.timer}")

        self.af = self.af_table[self.timer][0]
        self.ch_a = self.af_table[self.timer][1]
        self.ch_b = self.af_table[self.timer][2]

        self.period = 65535

        self.pin_a = Pin(self.ch_a, Pin.AF_PP, pull=Pin.PULL_NONE, af=self.af)
        self.pin_b = Pin(self.ch_b, Pin.AF_PP, pull=Pin.PULL_NONE, af=self.af)

        self.enc_timer = Timer(self.timer, prescaler=0, period=self.period)

        self.enc_timer.channel(1, Timer.ENC_AB)
        self.counts = 0
        self.__prev_counts = 0
        self.__turns = 0

    def __del__(self):
        print(f"Encoder was deleted from timer TIM{self.timer}")
        # DEINIT TIMER

    def update_counter(self):
        self.__prev_counts = self.counts
        self.counts = self.enc_timer.counter()

    def get_counter(self, overflow=False):
        self.update_counter()
        if overflow:
            counts = self.__multiturn_counter()
        else:
            counts = self.counts
        return counts

    def __multiturn_counter(self, threshold=30000):
        # self.velocity_estimate = self.encoder_prev
        if self.__prev_counts - self.counts >= threshold:
            self.__turns += 1
        elif self.__prev_counts - self.counts <= -threshold:
            self.__turns += -1
        # self.prev_counts = counts
        return self.counts + self.period * self.__turns
