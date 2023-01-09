from sensor_board import SensorBoardInterface
from time import sleep

# 

board = SensorBoardInterface(board_id = 1)
board.start()

try:
    while True:
        for i in range(2):
            board.get_states()
        print(board.angle_counts)
        print(board.force_counts)

        
except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    pass

