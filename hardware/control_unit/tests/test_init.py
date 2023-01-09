from interface import RobotInterface
import numpy as np


robot = RobotInterface()
robot.initialize(save_to_file = True, return_to = True)
# robot.move_to()

try:

    while True:
    
        robot.set_torques(torques = np.zeros(4))
        robot.update_state()
        
        angle = robot.state.modules.angles
        speed = robot.state.modules.speeds
        force = robot.state.modules.forces
        position = robot.state.carriages.position
        print(angle)

     
except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    robot.set_torques(torques = [0, 0, 0, 0])
    robot.__del__()
    pass

