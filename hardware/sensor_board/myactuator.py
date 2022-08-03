from struct import unpack, pack
# from micropython import const

# SIZE OF CAN FRAME
FRAME_SIZE = 8

# PROTOCOL CONSTANTS
WRITE_PID_RAM = b"\x31"
WRITE_PID_ROM = b"\x32"
READ_ACC_LIM = b"\x33"
WRITE_ACC_LIM = b"\x34"
READ_ENC = b"\x90"
SET_ENC_OFFSET = b"\x91"
SET_ZERO_ROM = b"\x19"
READ_ENC_MULTITURN = b"\x92"
READ_ENC_ANGLE = b"\x94"
READ_STATUS_1 = b"\x9A"
READ_STATUS_2 = b"\x9C"
READ_STATUS_3 = b"\x9D"
CLEAR_ERRORS = b"\x9B"
MOTOR_OFF = b"\x80"
MOTOR_ON = b"\x81"
MOTOR_RUN = b"\x88"
SET_TORQUE = b"\xA1"
SET_SPEED = b"\xA2"
SET_POS_1 = b"\xA3"
SET_POS_2 = b"\xA4"
SET_POS_3 = b"\xA5"
SET_POS_4 = b"\xA6"

# FORMAT FOR COMMANDS

# FORMAT FOR REPLY

STATE_REPLY = '<hhh'
ENC_MULTITURN_REPLY = '<hhhh'
READ_ACC_LIM_REPLY = '<hhhh'
READ_PID_REPLY = '<hhhh'
READ_STATUS_3_REPLY = '<hhhh'
READ_STATUS_2_REPLY = '<hhhh'
READ_STATUS_1_REPLY = '<hhhh'

#
ENC_RES = 16384
TORQUE_RES = 2048
SPEED_RES = 1
SPEED_LSB = 1
TORQUE_LSB = 1


class MyActuator:

    def __init__(self, device_id=321, can=None, reset=False, cmd_as_tuple=False):
        # self.protocol =
        self.msg = bytearray(FRAME_SIZE)
        self.cmd = bytearray(FRAME_SIZE)
        self.as_tuple = cmd_as_tuple
        self.id = device_id
        # self.set_can()

        self.state = 0

        self.counts = 0
        self.counts_mt = 0
        self.speed = 0
        self.current = 0
        self.bits = ENC_RES
        self.__prev_counts = 0
        self.__init_counts = 0
        self.turns = 0

        self.reply = [0, 0, 0, memoryview(self.msg)]
        if can != None:
            self.set_can(can)
        else:
            print(
                'provide CAN object as argument for constructor, or use set_can() method')

        if reset:
            self.reset_counter()

    def set_can(self, can):
        self.can = can
        # self.can = CAN(1, CAN.NORMAL, extframe=False, prescaler=2, sjw=1, bs1=14, bs2=6)
        # self.can.setfilter(0, CAN.LIST16, 0, (320, 321, 322, 323))

    def reset_counter(self):
        # cmd = b"\xA1" + 7 * b"\x00"
        # self.send_rcv(self.id, cmd)
        for i in range(20):
            self.set_torque(0)
        self.__init_counts = self.counts
        print(self.__init_counts)

    @micropython.native
    def send_rcv(self, dvc_id, cmd):

        if self.as_tuple:
            self.can.send(tuple(cmd), dvc_id)
        else:
            self.can.send(cmd, dvc_id)   # seWnd a message with id 123
        dev_id = 0
        # time.sleep_us(100)
        if self.can.any(0):
            # if dev_id != self.id:
            dev_id, _, _, _, msg = self.can.recv(0)
            self.msg = msg

        return self.msg

    @micropython.native
    def send(self, cmd, dvc_id):
        self.can.send(cmd, dvc_id)
        # return

    @micropython.native
    def recv(self):
        dev_id, _, _, _, msg = self.can.recv(0)
        self.msg = msg

    # def
    @micropython.native
    def set_torque(self, torque):
        torque_bytes = pack('<h', torque)
        # memoryview(self.msg)
        self.msg = memoryview(SET_TORQUE + 3 * b"\x00" +
                              torque_bytes + 2 * b"\x00")
        return self.msg

    @micropython.native
    def update_state(self):

        # self.send_rcv(self.id, cmd)

        self.state = unpack(STATE_REPLY, self.msg[2:])
        # self.temp =
        self.current = self.state[0]
        self.speed = self.state[1]
        self.counts = self.state[2]

        self.counts_mt = self.__multiturn_counter() - self.__init_counts
        return self.counts, self.turns, self.counts_mt, self.speed, self.current

    @micropython.native
    def __multiturn_counter(self, threshold=8000):
        # self.velocity_estimate = self.encoder_prev
        if self.__prev_counts - self.counts >= threshold:
            self.turns += 1
        elif self.__prev_counts - self.counts <= -threshold:
            self.turns += -1

        self.__prev_counts = self.counts
        # self.prev_counts = counts
        return self.counts + self.bits * self.turns
