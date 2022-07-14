
import pyb  # type: ignore
import time

adc = pyb.ADC("A1")  # create an analog object from a pin
while True:

    t = time.ticks_us()
    data = adc.read()
    diff = time.ticks_diff(time.ticks_us(), t)
    print(data, diff)
    # print(adc.read())  # read an analog value
    pyb.delay(1)
