from sensor_board import SensorBoardInterface
from misc import RecursiveNamespace
from time import sleep
from numpy import zeros, pi

LIN_ENC_SCALE = 25.4/(360*4)
MOD_ENC_SCALE = 2*pi/16384
MOD_SPD_SCALE = 1
MOD_TRQ_SCALE = 1
MOD_CUR_SCALE = 1
MOD_FRC_SCALE = 1


class RobotInterface:
    def __init__(self, update_rate = 500) -> None:
        
        print('Initializing sensor boards....')
        self.boards = [] 
        for board_id in range(2):
            self.boards.append(SensorBoardInterface(board_id = board_id))
            self.boards[board_id].start()
            
        # self.__motor_state =     
        self.__motor_offsets = zeros(4) 
        self.__linear_offsets = zeros(3)
        
        self.__state_dict = {'modules': {'angles': zeros(4), 'speeds': zeros(4), 'torques': zeros(4), 'forces': zeros(4)},
                             'carriages' : {'position': zeros(3), 'speed': zeros(3)},
                             'end_effector': {'position':zeros(3), 'speed': zeros(3), 'force': zeros(3)}
                             }
        
        self.state = RecursiveNamespace(**self.__state_dict)

        self.cmd_modes = [0xA1, 0xA1, 0xA1, 0xA1]
        self.cmd_data = zeros(4)


    def __del__(self):
        print('Interface was destroyed')

    # def update_device(self):

    def update_state(self):
        for board_id in range(2):
            self.boards[board_id].get_states()
            self.state.modules.angles[2*board_id:2*(board_id+1)] = MOD_ENC_SCALE*self.boards[board_id].angle_counts
            self.state.modules.speeds[2*board_id:2*(board_id+1)] = MOD_SPD_SCALE*self.boards[board_id].speed_counts
            self.state.modules.torques[2*board_id:2*(board_id+1)] = MOD_TRQ_SCALE*self.boards[board_id].torque_counts
            self.state.modules.forces[2*board_id:2*(board_id+1)] = MOD_FRC_SCALE*self.boards[board_id].force_counts

        # self.angle_counts = zeros(2, dtype='i')
        # self.speed_counts = zeros(2, dtype='i')
        # self.torque_counts = zeros(2, dtype='i')
        # self.force_counts = zeros(2, dtype='i')
        # self.linear_counts = zeros(2, dtype='i')
        # self.stop_limits = zeros(2, dtype='b')     
            
    def get_force(self):
        pass


    def set_torques(self, torques = zeros(4)):
        self.cmd_modes = [0xA1, 0xA1, 0xA1, 0xA1]
        self.cmd_data = torques
        for board_id in range(2):
            self.boards[board_id].set_command(self.cmd_modes[2*board_id:2*(board_id+1)], 
                                               self.cmd_data[2*board_id:2*(board_id+1)])
            # print(self.boards[board_id].cmd)

    
    def set_modes(self):
        pass
    
    def set_cart_pos(self):
        pass
    
    def set_motor_pos(self):
        pass
    
    def set_motor_speed(self):
        pass