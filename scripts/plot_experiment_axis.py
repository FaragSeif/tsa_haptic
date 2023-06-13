from time import perf_counter
import numpy as np
from models import *
from calculations import *
from scipy.interpolate import griddata
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# import seaborn as sns

plt.style.use('plots/test_style.mplstyle')
samples = 1000
t_fin = 18

axis = 'y'
file_label = axis + '_motion_slow_motors_5kg'

axis_ind = {'x':0,'y':1,'z':2}

data_array = np.genfromtxt(f'data/{file_label}.csv', delimiter=',')
# print(data_array)
t = data_array[:, 0] - data_array[0, 0]
motor_angles = data_array[:, 1:5]
motor_speeds = data_array[:, 5:9]
friction = 5*np.tanh(motor_speeds*0.2) + 0.02*motor_speeds
motor_torques = data_array[:, 9:13] - friction
carriage_positions = data_array[:, 13:16]
tension_scale = np.array([ 0.87836188,0.67126076,0.66536704,1])
tensions = tension_scale*data_array[:, 16:20] + np.array([13.58203349, 12.62714948, 5.81913085, 0])
desired_pos = data_array[:, 20:]
end_effector_pos = np.zeros((len(t), 3))
tensions_model = np.zeros((len(t), 3))
torques_model = np.zeros((len(t), 3))
force = np.array([0,0,5*9.81])

# Y_T = np.zeros((len(t), 2, 3))
for i in range(len(t)):
    end_effector_pos[i, :], jacobian_m, jacobian_d, jac_tsa = full_kinematics(carriage_positions[i, :], motor_angles[i, :])
    # print(jacobian_m)
    tensions_model[i, :] = np.linalg.inv(jacobian_m[:3,:].T) @ force
    torques_model[i,:] = np.linalg.inv(jacobian_d[:3,:].T) @ force

torques_norm = np.linalg.norm(motor_torques, axis=1)
torque_model_norm = np.linalg.norm(torques_model, axis=1)

tension_norm = np.linalg.norm(tensions, axis=1)
tension_model_norm = np.linalg.norm(tensions_model, axis=1)

plt.figure(figsize=(7., 2.5))
plt.scatter(2.2*end_effector_pos[:,axis_ind[axis]], tension_norm/np.min(tension_model_norm), s = 2.5, color = 'r', alpha = 0.2)
plt.plot(2.2*end_effector_pos[:,axis_ind[axis]], tension_model_norm/np.min(tension_model_norm), 'r-', label='1')
plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
plt.grid(True)
# plt.legend()
plt.ylim([0.5,1.5])
plt.ylabel(r'Normalized Mechanism Ratio')
plt.xlabel(fr'Cartesian axis ${axis}$ (mm)' )
# plt.xlim([-50,50])
plt.xlim([-50,75])
plt.tight_layout()
plt.savefig(f'plots/tensions_ratio_{axis}.png')
plt.show()

plt.figure(figsize=(7., 2.5))
plt.scatter(2.2*end_effector_pos[:,axis_ind[axis]], (2-(torques_norm)/np.min(torque_model_norm)), s = 4.5, color = 'b', alpha = 0.1)
plt.plot(2.2*end_effector_pos[:,axis_ind[axis]], (2-(torque_model_norm)/np.min(torque_model_norm)), 'b-', label='1')
plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
plt.grid(True)
# plt.xlim([-50,50])
plt.xlim([-50,75])
plt.ylim([0.5,1.5])
plt.ylabel(r'Normalized Device Ratio')
plt.xlabel(fr'Cartesian axis ${axis}$ (mm)' )
plt.tight_layout()
plt.savefig(f'plots/torque_ratio_{axis}.png')
plt.show()

torques_norm = np.linalg.norm(motor_torques, axis=1)
torque_model_norm = np.linalg.norm(torques_model, axis=1)


