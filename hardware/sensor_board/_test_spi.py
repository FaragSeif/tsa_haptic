from time import ticks_us
from struct import pack, unpack
from pyb import SPI
from config import *

spi = SPI(SPI_CH)
spi.init(SPI.SLAVE, 
         prescaler=SPI_CH, 
         polarity=SPI_POL,
         phase=SPI_PH, 
         bits=SPI_BITS)

cmd_buf = bytearray(CMD_SIZE)
send_data = bytearray(range(REPLY_SIZE)[::-1])

print('Press any key to start')
input()

try:
    while True:
        t0 = ticks_us()
        spi.recv(cmd_buf)
        spi.send(send_data)
        t1 = ticks_us()
        print(cmd_buf, t1 - t0)

except KeyboardInterrupt:
    spi.deinit()
