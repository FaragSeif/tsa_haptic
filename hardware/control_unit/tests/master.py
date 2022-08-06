# SEND DATA FROM PC AND RECEIVE REPLY BACK

# from pyb import UART
from time import perf_counter, sleep
import serial
import struct
print('Press any key to begin:')
input()

baud = 921600
# baud = 115200
ser = serial.Serial('/dev/ttyUSB1', baud)#, timeout=0.0001)
print('UART with baud', baud)

bytes_number = 15
msg_bytes = b'\x3A' + bytearray(range(bytes_number))
ticks= 0
try:
    while True:
        t1 = perf_counter()
        ser.write(msg_bytes)
        while True:

            if ser.in_waiting > 0:
                rcv = ser.read(1)
                # print(rcv)
                if rcv == b'\x3A':
                    break
        # uart.readinto(msg, bytes_number)

        reply = ser.read(bytes_number)
        t2 = perf_counter()
        if reply:

            data = struct.unpack('<BLhbhhhb', reply)
            print(data[1] - ticks)
            ticks = data[1]
            # print((t2 - t1)*1_000_000)
            # print(reply)
            # print(data)
        # sleep(1/20)

except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    ser.close()
