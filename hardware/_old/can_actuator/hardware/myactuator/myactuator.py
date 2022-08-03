from struct import unpack, pack

# TODO: Implement motor state parser
#
#

# TORQUE_CONTROL
# SPEED_CONTROL
# POSITION_CONTROL


class MyActuator:
    def __init__(self, device_id=321, can=None):
        # self.protocol =
        self.msg = bytearray(8)
        self.cmd = bytearray(8)
        self.id = device_id
        # self.set_can()
        self.cmd_trq = b"\xA1" + bytearray(7)
        self.state_frmt = "<hhh"
        self.state = 0
        self.reply = [0, 0, 0, memoryview(self.msg)]
        if can != None:
            self.set_can(can)
        else:
            print(
                "provide CAN object as argument for constructor, or use set_can() method"
            )

    def set_can(self, can):
        self.can = can
        # self.can = CAN(1, CAN.NORMAL, extframe=False, prescaler=2, sjw=1, bs1=14, bs2=6)
        # self.can.setfilter(0, CAN.LIST16, 0, (320, 321, 322, 323))

    @micropython.native  # type: ignore
    def send_rcv(self, dvc_id, cmd):
        self.can.send(cmd, dvc_id)  # seWnd a message with id 123
        dev_id = 0
        if self.can.any(0):
            while dev_id != self.id:
                dev_id, _, _, msg = self.can.recv(0)
                self.msg = msg

        return self.msg

    @micropython.native  # type: ignore
    def set_torque(self, torque):
        torque_bytes = pack("<h", torque)
        # memoryview(self.msg)
        cmd = b"\xA1" + 3 * b"\x00" + torque_bytes + 2 * b"\x00"

        self.send_rcv(self.id, cmd)
        self.state = unpack(self.state_frmt, self.msg[2:])
        return self.state
