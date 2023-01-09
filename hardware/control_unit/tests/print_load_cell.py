from interface import RobotInterface
from time import sleep


robot = RobotInterface()

#
try:
    while True:
        robot.set_torques(torques = [0, 0, 0, 0])
        robot.update_state()
        
        print(robot.state.modules.forces)
        sleep(0.1)
    
        
except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    robot.set_torques(torques = [0, 0, 0, 0])
    pass

