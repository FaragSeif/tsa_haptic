from sensor_board import SensorBoardInterface
from misc import RecursiveNamespace, fit_quadratic
from time import sleep, perf_counter
from numpy import zeros, pi, array, copy, sign, inf, sin, genfromtxt, savetxt, min, ones

LIN_ENC_SCALE = 25.4/(360*4)
MOT_ENC_SCALE = 2*pi/16384.
MOT_SPD_SCALE = 2*pi/360.

MOT_TRQ_SCALES = array([0.1957, 0.1947, 0.1984, 0.3812])
LCELL_FRC_SCALES = array([0.1212, 0.1214, 0.1266, 0.1247])
LCELL_BIAS = array([66, 42, 48, 52])

CALI_ANGLE = genfromtxt('calibration/angle.conf')
CALI_POS = genfromtxt('calibration/position.conf')
CALI_FRC = genfromtxt('calibration/force.conf')


class RobotInterface:
    def __init__(self, update_rate=500) -> None:
        print('[ROBOT INTERFACE] Initialization of robot interface\n')
        
        print('[ROBOT INTERFACE] Initializing sensor boards....')
        self.boards = []
        for board_id in range(2):
            self.boards.append(SensorBoardInterface(board_id=board_id))
            self.boards[board_id].start()

        # self.__motor_state =
        self.__angle_bias = CALI_ANGLE
        self.__pos_bias = CALI_POS
        self.__force_bias = CALI_FRC

        self.__state_dict = {'modules': {'angles': zeros(4), 'speeds': zeros(4), 'torques': zeros(4), 'forces': zeros(4)},
                             'carriages': {'position': zeros(3), 'speed': zeros(3)},
                             'end_effector': {'position': zeros(3), 'speed': zeros(3), 'force': zeros(3)}
                             }
        self.__enc_buff = zeros(4)

        self.state = RecursiveNamespace(**self.__state_dict)

        self.cmd_modes = [0xA1, 0xA1, 0xA1, 0xA1]
        self.cmd_data = zeros(4)
        print('[ROBOT INTERFACE] Boards are running, device is ready\n')
        

    def __del__(self):
        for board_id in range(2):
            self.boards[board_id].stop(output=True)

        print('Interface was destroyed')

    def initialize(self, save_to_file=False, return_to=False):
        self.__angle_bias = zeros(4)
        self.__pos_bias = zeros(3)
        self.__force_bias = zeros(4)
        
        angle_bias = zeros(4)
        pos_bias = zeros(3)
        force_bias = zeros(4)
        
        # motor_bias =
        sleep(0.5)

        print('[INITIALIZATION] Initialization begin, it will take some time...')
        # print('Pushing to zero position...')
        self.__damped_push(torques=[3, 3, 3, -60], damping=[0, 0, 0, 0.3])
        sleep(0.1)
        print('--- Initializing modules...\n')
        for module_id in range(3):
            angle_bias[module_id], pos_bias[module_id] = self.__init_pushing_module(module_id)
        angle_bias[3] = self.__init_pulling_module()
        print('--- Modules are initialized ---\n')
        

        print('--- Moving to zero position ---')
        self.move_to(angles = angle_bias)

        print('--- Estimating force bias ---')
        force_bias = self.__get_force_bias()
        
        self.__angle_bias = angle_bias
        self.__pos_bias = pos_bias
        self.__force_bias = force_bias
        
        if save_to_file:
            savetxt('calibration/angle.conf', self.__angle_bias)
            savetxt('calibration/position.conf', self.__pos_bias)
            savetxt('calibration/force.conf', self.__force_bias)
            print('\n--- Files are saved to calibration folder ---')


        if return_to:
            sleep(0.2)
            print('\n--- Moving to initial position ---')
            self.move_to(angles=[180, 180, 180, 0],
                         kp=[5, 5, 5, 0],
                         kd=[0.3, 0.3, 0.3, 0.3],
                         feedforward=[0, 0, 0, 50])
            
        print('\n[INITIALIZATION] Sensor initialization is over, ready to work.')

    def __damped_push(self, torques, damping, time_limit=5):
        t0 = perf_counter()
        t = perf_counter() - t0
        control = zeros(4)

        print('--- Pushing to zero position ---')
        while t <= time_limit:
            t = perf_counter() - t0

            self.set_torques(torques=control)
            self.update_state()

            speed = self.state.modules.speeds

            # 1.MOVE TO INIT POSITION
            control[0] = -damping[0]*speed[0] + torques[0]
            control[1] = -damping[1]*speed[1] + torques[1]
            control[2] = -damping[2]*speed[2] + torques[2]
            control[3] = -damping[3]*speed[3] + torques[3]

            if t >= time_limit-0.1:
                control[:3] = zeros(3)

    def __init_pulling_module(self,  time_limit=15):
        print(f'--- Initialization of pulling module ---')
        control = zeros(4)
        t0 = perf_counter()
        t = perf_counter() - t0
        angles = []
        forces = []
        while t <= time_limit:
            t = perf_counter() - t0
            self.set_torques(torques=control)
            self.update_state()

            angle = self.state.modules.angles
            speed = self.state.modules.speeds
            force = self.state.modules.forces

            control[:3] = 10*ones(3)
            control[3] = -0.5*speed[3] - 70*sign(sin(0.25*pi*t))
            if t > 5:
                angles.append(angle[3])
                forces.append(force[3])
            sleep(0.002)

        # # position_offset = np.min(positions)
        force_offset = min(forces)
        forces = array(forces) - force_offset
        angles = array(angles)
        indeces = (forces >= 10)
        theta = angles[indeces]
        tension = forces[indeces]
        angle_offset, _, _, _, _ = fit_quadratic(theta, tension)
        print('--- Pulling module is initialized! ---')
        print(f'    Motor encoder bias: {angle_offset}\n')

        return angle_offset

    def __init_pushing_module(self, module_id, time_limit=10):

        print(f'--- Initialization of module {module_id+1} ---')
        angles = []
        positions = []
        t0 = perf_counter()
        t = perf_counter() - t0
        control = zeros(4)
        while t <= time_limit:
            t = perf_counter() - t0
            self.set_torques(torques=control)
            self.update_state()

            angle = self.state.modules.angles
            speed = self.state.modules.speeds
            # force = self.state.modules.forces
            position = self.state.carriages.position

            control = zeros(4)

            if module_id == 0:
                control[1] = -0.0*speed[1] + 8
                control[2] = -0.0*speed[2] + 8

            if module_id == 1:
                control[2] = -0.02*speed[2] + 5

            if module_id == 2:
                control[1] = -0.02*speed[2] + 5

            control[module_id] = -0.02 * \
                speed[module_id] - 10*sign(sin(0.5*pi*t))
            control[3] = -70 - 0.2*speed[3]

            positions.append(position[module_id])
            angles.append(angle[module_id])
            sleep(0.002)

        position_offset = min(positions)
        positions = array(positions)
        angles = array(angles)
        indeces = positions >= position_offset + 0.1
        theta = angles[indeces]
        contraction = positions[indeces]
        angle_offset, _, _, _, _ = fit_quadratic(theta, contraction)

        print(f'--- Pushing module {module_id+1} is initialized ---')
        print(f'    Linear encoder bias: {position_offset}')
        print(f'    Motor encoder bias: {angle_offset}\n')

        return angle_offset, position_offset


    def __get_force_bias(self, samples=500):
        print(f'--- Getting force bias, torques will be setted to zero ---')

        control = zeros(4)

        force_bias = zeros(4)
        for i in range(samples):
            self.set_torques(torques=control)
            self.update_state()
            force_bias += self.state.modules.forces/samples
            sleep(0.001)
        print(f'--- Force bias estimated ---')
        print(f'    Force bias: {force_bias}\n')
        return force_bias

    def move_to(self,
                angles=[0, 0, 0, 0],
                kp=[5, 5, 5, 5],
                kd=[0.3, 0.3, 0.3, 0.3],
                feedforward=[0, 0, 0, 0],
                time_limit=3):
        
        control = zeros(4)
        theta_d = array(angles)
        kp = array(kp)
        kd = array(kd)
        tau_0 = array(feedforward)

        t0 = perf_counter()
        t = perf_counter() - t0
        while t < time_limit:
            t = perf_counter() - t0
            self.set_torques(torques=control)
            self.update_state()

            theta = self.state.modules.angles
            dtheta = self.state.modules.speeds

            control = kp*(theta_d - theta) + kd*(- dtheta) + tau_0

    def update_state(self):
        for board_id in range(2):
            self.boards[board_id].get_states()
            self.state.modules.angles[2*board_id:2*(board_id+1)] = copy(
                MOT_ENC_SCALE*self.boards[board_id].angle_counts) - self.__angle_bias[2*board_id:2*(board_id+1)]
            self.state.modules.speeds[2*board_id:2*(board_id+1)] = copy(
                MOT_SPD_SCALE*self.boards[board_id].speed_counts)
            self.state.modules.torques[2*board_id:2*(board_id+1)] = copy(
                MOT_TRQ_SCALES[2*board_id:2*(board_id+1)]*self.boards[board_id].torque_counts)
            self.state.modules.forces[2*board_id:2*(board_id+1)] = copy(LCELL_FRC_SCALES[2*board_id:2*(
                board_id+1)]*(self.boards[board_id].force_counts)) - self.__force_bias[2*board_id:2*(board_id+1)]
            self.__enc_buff[2*board_id:2*(board_id+1)] = copy(
                LIN_ENC_SCALE*self.boards[board_id].linear_counts)
        self.state.carriages.position = copy(
            self.__enc_buff[:3]) - self.__pos_bias

    def set_torques(self, torques=zeros(4)):
        self.cmd_modes = [0xA1, 0xA1, 0xA1, 0xA1]
        self.cmd_data = (array(torques) / MOT_TRQ_SCALES).astype(int)

        for board_id in range(2):
            self.boards[board_id].set_command(self.cmd_modes[2*board_id:2*(board_id+1)],
                                              self.cmd_data[2*board_id:2*(board_id+1)])
            # print(self.boards[board_id].cmd)

    # def set_modes(self):
    #     pass

    # def set_cart_pos(self):
    #     pass

    # def set_motor_pos(self):
    #     pass

    # def set_motor_speed(self):
    #     pass

    # def get_force(self):
    #     pass