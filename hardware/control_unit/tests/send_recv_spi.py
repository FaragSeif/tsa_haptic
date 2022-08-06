import spidev
from struct import pack, unpack
import time 
import RPi.GPIO as GPIO

CLOCK_FREQ = 168000000/16
SPI_PRESCALE = 1
CMD_SIZE = 12
REPLY_SIZE = 30

spi_speed = int(CLOCK_FREQ/SPI_PRESCALE)

spi_speed = int(168000000/16)

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = spi_speed
spi.lsbfirst = False

# POLARITY/PHASE
spi.mode = 0b11
spi.bits_per_word = 8

CS_PIN = 8
GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN,GPIO.OUT, initial=True)

CMD = 9


STATE_FORMAT = '<BBIhhihhihhhh'
CMD_FORMAT = '<BBhhhhh'

ticks_0 = 0
ts = 0 
FREQ = 0.002

data_to_send = pack(CMD_FORMAT, 
                    CMD, 
                    0,
                    0, 
                    0, 
                    0, 
                    0, 
                    0)

spi.writebytes(data_to_send)
rcvd_bytes = spi.readbytes(REPLY_SIZE)
data = unpack(STATE_FORMAT, bytearray(rcvd_bytes))
print(data)