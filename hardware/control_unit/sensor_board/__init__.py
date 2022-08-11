from multiprocessing import Process, Array, Value
from numpy import zeros
# from config import *
from struct import pack, unpack
import serial
from time import sleep


UART_BAUD = 921600
UART_BYTESIZE = 8
UART_PARITY = 'N'
UART_STOP = 1
UART_TIMEOUT = None
UART_CHANNELS = ['/dev/ttyAMA1', 
                 '/dev/ttyAMA2']

CMD_SIZE = 12
REPLY_SIZE = 32

REPLY_FORMAT ='<BIhihhhBhihhhB'
CMD_FORMAT = '<BBBiBi'


# SCALES 
LINEAR_ENCODER_SCALE = 1
MOTOR_ENCODER_SCALE = 1
MOTOR_SPEED_SCALE = 1
MOTOR_TORQUE_SCALE = (1, 1, 1, 1)
LOAD_CELL_SCALE = (1, 1, 1, 1)


class SensorBoardInterface:
    def __init__(self, board_id=0) -> None:
        self.board_id = board_id
        self.CMD = 200
        self.uart = serial.Serial(UART_CHANNELS[board_id],
                                  baudrate=UART_BAUD,
                                  bytesize=UART_BYTESIZE,
                                  parity=UART_PARITY,
                                  stopbits=UART_STOP,
                                  timeout=UART_TIMEOUT)

        self._zero_command = pack(CMD_FORMAT,
                                  self.CMD, 
                                  0,
                                  0xA1, 0,
                                  0XA1, 0)

        self.received_bytes = bytearray(REPLY_SIZE)

        self.__initialize_states()
        self.__initialize_shared_states()

        self.command = self._zero_command
        
        self._process_is_working = False
        self._handler_process = Process(target=self.__handler)

    def __del__(self):
        if self._process_is_working:
            self.stop(output=True)
            print(f'sensor board {self.board_id} process is over')
            
        self.uart.close()
        print(f'UART {UART_CHANNELS[self.board_id]} is closed')

    def start(self):
        print('Sensor board process is activated....')
        self._handler_process.start()
        print('Waiting for process to start...')
        sleep(0.2)

    def stop(self, output=False):
        self._handler_process.terminate()
        if output:
            print('Robot process was terminated')
        

    def __handler(self):

        try:
            
            self._process_is_working = 1
            
            # initial_time = perf_counter()
            # tick = 0
            while True:
                # print(self.__tick.value)
                # actual_time = perf_counter() - initial_time
                cmd = self.pack_command(self.__command_modes, self.__command_data)
                self.update_device(cmd)
        
        except KeyboardInterrupt:
            print(f'Exit by interrupt from sensor board {self.board_id}')
            self._process_is_working
        except Exception as e:
            print(f'Get exception in sensor board {self.board_id}')
            print(e)
        finally:
            print(f'Attempting to kill the board {self.board_id} process...')
            

    def __initialize_shared_states(self):
        self.__error_code = Value('i', 0) 
        self.__tick = Value('i', 0)    
        self.__angle_counts = Array('i', 2*[0])
        self.__speed_counts = Array('i', 2*[0])
        self.__torque_counts = Array('i', 2*[0])
        self.__force_counts = Array('i', 2*[0])
        self.__linear_counts = Array('i', 2*[0])
        self.__stop_limits = Array('i', 2*[0])   
        self.__command_modes = Array('i', 2*[0])      
        self.__command_data = Array('i', 2*[0])  


    def __initialize_states(self):
        self.error_code = 0
        self.tick = 0
        self.angle_counts = zeros(2, dtype='i')
        self.speed_counts = zeros(2, dtype='i')
        self.torque_counts = zeros(2, dtype='i')
        self.force_counts = zeros(2, dtype='i')
        self.linear_counts = zeros(2, dtype='i')
        self.stop_limits = zeros(2, dtype='b')     


    def __copy_states(self):
        self.error_code = self.__error_code.value
        self.tick = self.__tick.value 
        # print(self.tick)
        for i in range(2):
            self.angle_counts[i] = self.__angle_counts[i] 
            self.speed_counts[i] = self.__speed_counts[i] 
            self.torque_counts[i] = self.__torque_counts[i] 
            self.force_counts[i] = self.__force_counts[i] 
            self.linear_counts[i] = self.__linear_counts[i] 
            self.stop_limits[i] = self.__stop_limits[i]         
        
    def get_states(self):
        self.__copy_states()
    
    def send(self, send_message):
        self.uart.write(send_message)

    def receive(self):
        
        while True:
            if self.uart.in_waiting > 0:
                rcv = self.uart.read(1)
                if rcv[0] == self.CMD:
                    break
        self.received_bytes = self.uart.read(REPLY_SIZE-1)

        return self.received_bytes

    def send_receive(self, send_message):
        self.send(send_message)
        received_bytes = self.receive()

        return received_bytes

    def parse_reply(self, received_bytes, share = True):
        self.received_data = unpack(REPLY_FORMAT, bytearray(received_bytes))
        # TODO: STATE PARSING
        # self.measurements.cmd = self.received_data[0]
        self.__error_code.value = self.received_data[0]
        self.__tick.value = self.received_data[1]
        for i in range(2):
            self.__linear_counts[i] = self.received_data[2 + i*6]
            self.__angle_counts[i] = self.received_data[3 + i*6]
            self.__speed_counts[i] = self.received_data[4 + i*6]
            self.__torque_counts[i] = self.received_data[5 + i*6]
            self.__force_counts[i] = self.received_data[6 + i*6]
            self.__stop_limits[i] = self.received_data[7 + i*6]

        self.__copy_states()        

        return self.received_data

    # def set_command(self, command):
    #     self.command = command

    def pack_command(self, cmd_modes, cmd_data):
        
        self.command = pack(CMD_FORMAT,
                            self.CMD, 
                            0,
                            cmd_modes[0], cmd_data[0],
                            cmd_modes[1], cmd_data[1])
        return self.command 

    def set_command(self, cmd_modes, cmd_data):
        for i in range(2):
            self.__command_modes[i] = cmd_modes[i]
            self.__command_data[i] = cmd_data[i]
        
        
    def update_device(self, command):
        if command is not None:
            cmd = command
        else:
            cmd = self.command

        received_bytes = self.send_receive(cmd)
        self.parse_reply(received_bytes)
  