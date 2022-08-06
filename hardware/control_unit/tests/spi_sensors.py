import spidev
from struct import pack, unpack
import time 
import RPi.GPIO as GPIO

CLOCK_FREQ = 168000000
SPI_PRESCALE = 1
CMD_SIZE = 32
REPLY_SIZE = 32

# spi_speed = int(CLOCK_FREQ/SPI_PRESCALE)

spi_speed = 656250#5_250_000

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = spi_speed
spi.lsbfirst = False

# POLARITY/PHASE
spi.mode = 0b01
spi.bits_per_word = 8

CS_PIN = 8
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(CS_PIN,GPIO.OUT, initial=True)

CMD = 200


STATE_FORMAT = '<BBIhhihhihhhhBB'
CMD_FORMAT = '<BB'

ticks_0 = 0
ts = 0 
FREQ = 0.00000000001

data_to_send = pack(CMD_FORMAT, 
                    CMD, 0)

data_to_send += bytearray(30)

# print(data_to_send)
rcvd_bytes = spi.xfer(list(data_to_send), spi_speed, 5, 8)
data = unpack(STATE_FORMAT, bytearray(rcvd_bytes))

# ticks = data[2]

try:
    ts = time.perf_counter() 
    while True:

        if time.perf_counter() - ts >= FREQ:
            
            rcvd_bytes = spi.xfer(list(data_to_send), spi_speed, 10, 8)
            spi.writebytes2(data_to_send)
            rcvd_bytes = spi.readbytes(REPLY_SIZE)
            data = unpack(STATE_FORMAT, bytearray(rcvd_bytes))
            print(data)#, ticks - ticks_0)
            # if rcvd_bytes[0] == CMD:
            #     ticks = data[2]
            #     # print(ticks - ticks_0)
            #     print(data, ticks - ticks_0)
            #     ticks_0 = ticks
            ts = time.perf_counter() 

        
except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    spi.close()
    print("SPI closed")
