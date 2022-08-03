import spidev
from struct import pack, unpack
import time 
import RPi.GPIO as GPIO

CLOCK_FREQ = 168000000/16
SPI_PRESCALE = 1
CMD_SIZE = 12
REPLY_SIZE = 30

spi_speed = int(CLOCK_FREQ/SPI_PRESCALE)
print(spi_speed)
spi = spidev.SpiDev()
spi.open(0, 0)

spi.max_speed_hz = spi_speed
spi.lsbfirst = False
# POLARITY/PHASE
spi.mode = 0b10
spi.bits_per_word = 8
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(7,GPIO.OUT, initial=True)



torque_1 = 0
torque_2 = 0


CMD = 200


STATE_FORMAT = '<BBIhhihhihhhh'
CMD_FORMAT = '<BBhhhhh'

ticks_0 = 0
ts = 0 
FREQ = 0.00001

data_to_send = pack(CMD_FORMAT, 
                    CMD, 
                    0,
                    0, 
                    0, 
                    0, 
                    0, 
                    0)

# print(data_to_send)
spi.writebytes(data_to_send)
rcvd_bytes = spi.readbytes(REPLY_SIZE)
data = unpack(STATE_FORMAT, bytearray(rcvd_bytes))
ticks = data[2]

try:
    ts = time.perf_counter() 
    while True:
        
        if time.perf_counter() - ts >= FREQ:
            spi.writebytes(data_to_send)
            rcvd_bytes = spi.readbytes(REPLY_SIZE)
            data = unpack(STATE_FORMAT, bytearray(rcvd_bytes))
            
            if rcvd_bytes[0] == CMD:
                ticks = data[2]
                # print(data, ticks - ticks_0)
                print(ticks - ticks_0)
                # print(ticks - ticks_0)
                ticks_0 = ticks
            ts = time.perf_counter() 

        
except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    spi.close()
    print("SPI closed")
