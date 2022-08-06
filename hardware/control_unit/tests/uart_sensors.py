import serial
from struct import pack, unpack
import time 
import RPi.GPIO as GPIO


CMD_SIZE = 12
REPLY_SIZE = 32

ser = serial.Serial('/dev/ttyAMA1', baudrate=921600, bytesize=8, parity='N', stopbits=1, timeout=None)



CMD = 200

STATE_FORMAT = '<BIhhihhihhhhBB'
CMD_FORMAT = '<BBhhhhh'

ticks_0 = 0
ts = 0 
FREQ = 0.1

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

            # if ser.in_waiting > 0:
            rcvd_bytes = ser.read(REPLY_SIZE - 1)
            data = unpack(STATE_FORMAT, bytearray(rcvd_bytes))
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
