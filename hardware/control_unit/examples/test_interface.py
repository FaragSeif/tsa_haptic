from interface import RobotInterface
from time import sleep


robot = RobotInterface()

#
try:
    while True:
        robot.set_torques(torques = [30, 30, 30, 40])
        robot.update_state()
        
        print(f'\n{15*"/"} DEVICE STATE {15*"/"}')
        print(f'MODULES:',
            #   f'\n  positions: {robot.state.modules.angles}'+ 20*' ', 
              f'\n  angles: {robot.state.modules.angles}'+ 20*' ', 
              f'\n  speeds: {robot.state.modules.speeds}'+ 20*' ', 
              f'\n  torques: {robot.state.modules.torques}' + 20*' ', 
              f'\n  forces: {robot.state.modules.forces}'+ 20*' ', 
              f'\n  position: {robot.state.carriages.position}'+ 20*' ',
              end=10*" " + "\n", flush=True)
        print(f'{52*"/"}\n')
        print(10*'\033[A', end="\r", flush=True)
    
        
except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    robot.set_torques(torques = [0, 0, 0, 0])
    pass

