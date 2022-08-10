from config import *
from pyb import CAN, ADC, UART
from quad_encoder import QuadEncoder
from myactuator import MyActuator
from struct import pack, unpack
from time import ticks_us, sleep_us

# ///////// SPI ////////
uart = UART(UART_CH, 
            UART_BAUD, 
            bits=UART_BITS, 
            parity=UART_PARITY, 
            stop=UART_STOP, 
            timeout=UART_TIMEOUT)

# spi.init()

# ////////////// HARDWARE INITIALIZATION //////////////

amp1 = ADC(AMP1_ADC_PIN)  # create an analog object from a pin
amp2 = ADC(AMP2_ADC_PIN)  # create an analog object from a pin

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
act1 = MyActuator(device_id=MOT_ID1, can=can) #, reset=True)
act2 = MyActuator(device_id=MOT_ID2, can=can) #, reset=True)


recv_buf = bytearray(CMD_SIZE)

# print('Press any key to continue...')
# input()

try:
    # STATE_FORMAT
    cmd = 200  # 1 byte
    error_code = 0  # 1 bytes
    tick = 0  # 4 bytes

    while True:
        # sleep_us(100)
        tick = ticks_us()
        # -------------------
        # STATE OF DEVICE
        # -------------------
        amp1_counts = amp1.read()
        amp2_counts = amp2.read()
        enc1_counts = enc1.get_counter(overflow=True)
        enc2_counts = enc2.get_counter(overflow=True)

        act1.set_torque(0, send=True)
        act1.update_state()

        act2.set_torque(0, send=True)
        act2.update_state()

        while True:
            if uart.any() > 0:
                rcv = uart.read(1)
                if rcv[0] == 200:

                    state_bytes = pack(STATE_FORMAT,
                                       cmd,
                                       error_code,
                                       tick,
                                       enc1_counts,
                                       act1.counts_mt,
                                       act1.speed,
                                       act1.current,
                                       amp1_counts,
                                       0,
                                       enc2_counts,
                                       act2.counts_mt,
                                       act2.speed,
                                       act2.current,
                                       amp2_counts,
                                       0)

                    send_bytes = state_bytes
                    uart.read(CMD_SIZE - 1)
                    uart.write(send_bytes)
                    
                    break
            else:
                # act1.set_torque(0, send=True)
                # act1.set_torque(0, send=True)
                break
                # print('No command')
                # NO COMMAND RECEIVED
        # tick2 = ticks_us()
        # print(tick2 - tick)



except KeyboardInterrupt:
    print('Exit by interrupt')
# except Exception as e:
#     print(e)
finally:
    # print('Finally...')
    uart.deinit()
    can.deinit()
    # amp1.deinit()
    # amp2.deinit()
