from config import *
from pyb import CAN, ADC, UART
from quad_encoder import QuadEncoder
from myactuator import MyActuator
from struct import pack, unpack
from time import ticks_us, sleep_us

# TODO: ADD SWITCHES

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
# print('done')
# //////////////////////
# CAN BUS BLDC ACTUATOR
# /////////////////////
act1 = MyActuator(device_id=MOT_ID1, can=can) #, reset=True)
act2 = MyActuator(device_id=MOT_ID2, can=can) #, reset=True)


recv_buf = bytearray(CMD_SIZE)

    

# print('Press any key to continue...')
# input()
# cmd_cntrl = TORQUE_CMD
no_rcv_count = 0

desired_torques = [0, 0]
# STATE_FORMAT
cmd = 200  # 1 byte
error_code = 0  # 1 bytes
tick = 0  # 4 bytes



while True:
    try:
        act1.set_torque(desired_torques[0], send=True)
        act1.update_state()
        act2.set_torque(desired_torques[1], send=True)
        act2.update_state()

    except KeyboardInterrupt:
        # print('Exit by interrupt')
        uart.deinit()
        can.deinit()
        break
    
    except Exception as e:
        # print(e)
        if e.args[0] == 16:
            error_code = 2
            # print(e, error_code)
        else:
            uart.deinit()
            can.deinit()
            break
    # sleep_us(100)
    tick = ticks_us()
    # -------------------
    # STATE OF DEVICE
    # -------------------

        # print(desired_torques)
                # print('No command')
                # NO COMMAND RECEIVED
        # tick2 = ticks_us()
        # print(tick2 - tick)
    amp1_counts = amp1.read()
    amp2_counts = amp2.read()
    enc1_counts = enc1.get_counter(overflow=True)
    enc2_counts = enc2.get_counter(overflow=True)


    while True:
        if uart.any() > 0:
            no_rcv_count = 0 
            rcv = uart.read(1)
            if rcv[0] == 200:

                state_bytes = pack(STATE_FORMAT,
                                    rcv[0],
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
                rcv_bytes = uart.read(CMD_SIZE - 1)
                # print(rcv_bytes)
                rcv_data = unpack(CMD_FORMAT, rcv + rcv_bytes)
                # print(rcv_data)
                
                desired_torques[0] = rcv_data[3]
                desired_torques[1] = rcv_data[5]
                    
                uart.write(send_bytes)
                break
        else:
            no_rcv_count += 1
            if no_rcv_count >= 50:
                desired_cmd = [0, 0]
                desired_torques = [0, 0] 
                error_code = 1
                # print('No cmd')
            break

