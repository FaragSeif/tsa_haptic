import numpy as np


init_angles = np.array([-92.05823059758514, 90.53455437859897, -153.1917866614793, -298.6547389724083])
np.savetxt('angle_calibration.conf', init_angles)
init_pos = np.array([-15.945555555555554, -1.8344444444444443, -0.2998611111111111])
np.savetxt('position_calibration.conf', init_pos)

angle_load = np.genfromtxt('angle_calibration.conf')
pos_load = np.genfromtxt('position_calibration.conf')
print(angle_load)
print(pos_load)