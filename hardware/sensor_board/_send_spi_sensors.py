from time import ticks_us
from struct import pack, unpack
from pyb import SPI
from config import *

spi = SPI(SPI_CH)
spi.init(SPI.SLAVE,
         prescaler=SPI_PRESCALE,
         polarity=SPI_POL,
         phase=SPI_PH,
         bits=SPI_BITS)

recv_buf = bytearray(CMD_SIZE)

print('Press any key to start')
input()


try:
    # STATE_FORMAT
    cmd = 100  # 1 byte
    error_code = 0  # 1 bytes
    tick = 112541  # 4 bytes
    enc1_counts = 2451  # 2 bytes
    enc2_counts = 1265  # 2 bytes
    mot1_counts = 1431214  # 4 bytes
    mot1_speed = 1212  # 2 bytes
    mot1_current = 415  # 2 bytes
    mot2_counts = -1235512  # 4 bytes
    mot2_speed = -341  # 2 bytes
    mot2_current = -241  # 2 bytes
    amp1_counts = 14145  # 2 bytes
    amp2_counts = 12156  # 2 bytes

    while True:

        tick = ticks_us()
        send_bytes = pack(STATE_FORMAT,
                          cmd,
                          error_code,
                          tick,
                          enc1_counts,
                          enc2_counts,
                          mot1_counts,
                          mot1_speed,
                          mot1_current,
                          mot2_counts,
                          mot2_speed,
                          mot2_current,
                          amp1_counts,
                          amp2_counts)

        spi.recv(recv_buf)
        spi.send(send_bytes)
        cmd_data = unpack(CMD_FORMAT, recv_buf)
        cmd = cmd_data[0]
        # print(cmd)




except KeyboardInterrupt:
    spi.deinit()
