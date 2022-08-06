import serial
from struct import pack, unpack
import time 
from config import *


board_id = 0 

ser = serial.Serial(UART_CHANNELS[board_id], 
                    baudrate = UART_BAUD, 
                    bytesize = UART_BYTESIZE, 
                    parity = UART_PARITY, 
                    stopbits = UART_STOP, 
                    timeout = UART_TIMEOUT)

CMD = 200

ticks_0 = 0
ts = 0 
FREQ = 0.000001

data_to_send = pack(CMD_FORMAT, 
                    CMD, 0,0,0,0,0,0)

rcvd_bytes = bytearray(32)

ticks_array = []

try:
    ts = time.perf_counter() 
    ser.write(data_to_send)
    while True:
        if ser.in_waiting > 0:
            rcv = ser.read(1)
            if rcv[0] == 200:
                break
            
    rcvd_bytes = ser.read(REPLY_SIZE-1)
    while True:

        if time.perf_counter() - ts >= FREQ:
            ser.write(data_to_send)

            while True:
                if ser.in_waiting > 0:
                    rcv = ser.read(1)
                    if rcv[0] == 200:
                        break

            rcvd_bytes = ser.read(REPLY_SIZE - 1)
            data = unpack(REPLY_FORMAT, bytearray(rcvd_bytes))
            ticks = data[1]
            
            print(data)
            # print(ticks - ticks_0)
            ticks_0 = ticks
            ts = time.perf_counter() 

        
except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    ser.close()
    # print("SPI closed")
