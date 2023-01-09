from sensor_board import SensorBoardInterface
from time import sleep

# 

boards = [] 
for board_id in range(2):
    boards.append(SensorBoardInterface(board_id = board_id))
    boards[board_id].start()

try:
    while True:
        print(boards[0].shared.angle_counts, boards[1].shared.angle_counts)
        print(boards[0].shared.angle_counts, boards[1].shared.angle_counts)

        
except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    print('close')

