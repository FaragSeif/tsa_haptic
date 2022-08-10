from sensor_board import SensorBoardInterface
from time import sleep

# 

boards = [] 
for board_id in range(2):
    boards.append(SensorBoardInterface(board_id = board_id))
    boards[board_id].start()

try:
    while True:
        # pass
        # print()
        for i in range(2):
            boards[i].get_states()

        # print(boards[0].angle_counts[0], boards[1].angle_counts[1])
        # print(boards[0].angle_counts, boards[1].angle_counts)
        # print(boards[0].angle_counts, boards[1].angle_counts)
        # print(boards[0].angle_counts, boards[1].angle_counts)

        
except KeyboardInterrupt:
    print('Exit by interrupt')
except Exception as e:
    print(e)
finally:
    pass

