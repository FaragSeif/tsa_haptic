from pyb import Pin

class ForceSensor:

    def __init__(self,
                 pin,
                 channel,
                 adc):

        self.pin = Pin(pin, Pin.ANALOG)
        self.adc = adc
        self.adc_channel = channel
        #
        self.force_u16 = 0
        self.force = 0

    # def __del__(self):
    #     self.adc.deinit()

    def get_force(self):
        self.force = self.adc.read_channel(self.adc_channel)
        return self.force

    def set_zero(self):
        pass
