from sensor_board import SensorBoardInterface
from time import sleep

# 

boards = [] 
for board_id in range(2):
    boards.append(SensorBoardInterface(board_id = board_id))
    boards[board_id].start()

try:
    while True:
        for i in range(2):
            boards[i].get_states()

        # print(boards[0].angle_counts, boards[1].angle_counts)
        # print(boards[0].linear_counts, boards[1].linear_counts)
        # print(boards[0].torque_counts, boards[1].torque_counts)
        print(boards[0].force_counts, boards[1].force_counts)

        
except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    pass

