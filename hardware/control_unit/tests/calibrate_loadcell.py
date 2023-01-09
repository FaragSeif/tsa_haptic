from interface import RobotInterface
from time import sleep


robot = RobotInterface()
weight = 2.503*9.81

try:
    while True:
        N = 2000
        for load_cell_id in range(4):
            print(f'Calibration of the load cell {load_cell_id + 1}')
            input('Press any key to continue....')
            load_cell_id = 3
        
        

            bias = 0
            print(f'Measuring....')
            for j in range(N):
                robot.set_torques(torques = [0, 0, 0, 0])
                robot.update_state()
                load_cell_reading = robot.state.modules.forces[load_cell_id]
                bias += load_cell_reading/N
                sleep(0.002)
            print(f'Bias of the load cell {load_cell_id + 1}: {bias}')
            input('Out the weight and press any key to continue....')
            print(f'Measuring....')
            mean_data = 0
            for j in range(N):
                robot.set_torques(torques = [0, 0, 0, 0])
                robot.update_state()
                load_cell_reading = robot.state.modules.forces[load_cell_id] - bias 
                mean_data += load_cell_reading/N
                sleep(0.002)
            print(f'Scale of the load cell {load_cell_id + 1}: {weight/mean_data}')
        
except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    robot.set_torques(torques = [0, 0, 0, 0])
    pass

