from config import *
from pyb import CAN, ADC, SPI
from quad_encoder import QuadEncoder
from myactuator import MyActuator
from struct import pack, unpack
from time import ticks_us

# ////////////// HARDWARE INITIALIZATION //////////////

# amp1 = ADC("A1")  # create an analog object from a pin
# amp2 = ADC("A3")  # create an analog object from a pin

# Quadrature Encoders
enc1 = QuadEncoder(timer=ENC1_TIM, direction=1)
enc2 = QuadEncoder(timer=ENC2_TIM, direction=1)

# ////////////////
# SETTING CAN BUS
# ///////////////

can = CAN(CAN_CH, CAN.NORMAL, prescaler=2, sjw=1, bs1=14, bs2=6)
can.setfilter(0, CAN.LIST16, 0, (MOT_ID1, MOT_ID2, 322, 323))

# //////////////////////
# CAN BUS BLDC ACTUATOR
# /////////////////////
act1 = MyActuator(device_id=MOT_ID1, can=can)
act2 = MyActuator(device_id=MOT_ID2, can=can)

# ///////// SPI ////////
spi = SPI(SPI_CH)
spi.init(SPI.SLAVE,
         prescaler=SPI_CH,
         polarity=SPI_POL,
         phase=SPI_PH,
         bits=SPI_BITS)
recv_buf = bytearray(CMD_SIZE)

print('Press any key to continue...')
input()

try:
    # STATE_FORMAT
    cmd = 100  # 1 byte
    error_code = 0  # 1 bytes
    tick = 0  # 4 bytes
    enc1_counts = 0  # 2 bytes
    enc2_counts = 0  # 2 bytes
    mot1_counts = 0  # 4 bytes
    mot1_speed = 0  # 2 bytes
    mot1_current = 0  # 2 bytes
    mot2_counts = 0  # 4 bytes
    mot2_speed = 0  # 2 bytes
    mot2_current = 0  # 2 bytes
    amp1_counts = 0  # 2 bytes
    amp2_counts = 0  # 2 bytes

    while True:

        tick = ticks_us()
        # -------------------
        # STATE OF DEVICE
        # -------------------
        # amp1_counts = amp1.read()
        # amp2_counts = amp2.read()
        enc1_counts = enc1.get_counter(overflow=True)
        enc2_counts = enc2.get_counter(overflow=True)
        # TODO:
        # ACTUATOR STATE
        # GET END SWITCHES

        # PACK STATE
        state_bytes = pack(STATE_FORMAT,
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

        # SEND RCV WITH SPI:
        send_bytes = state_bytes
        spi.recv(recv_buf)
        spi.send(send_bytes)
        cmd_data = unpack(CMD_FORMAT, recv_buf)
        cmd = cmd_data[0]
        # TODO: PROTOCOL

except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    can.deinit()
    # amp1.deinit()
    # amp2.deinit()
    

