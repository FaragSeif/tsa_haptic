from time import perf_counter
import numpy as np
from models import *
from calculations import *
from scipy.interpolate import griddata
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

plt.style.use('plots/test_style.mplstyle')
samples = 1000
t_fin = 18

file_label = 'circle_motion_slow_motors_5kg'
# file_label = 'var_spiral_motion_slow_motors_5kg'
file_label = 'var_spiral'


data_array = np.genfromtxt(f'data/{file_label}.csv', delimiter=',')
# print(data_array)
t = data_array[:, 0] - data_array[0, 0]
motor_angles = data_array[:, 1:5]
motor_speeds = data_array[:, 5:9]
# friction = 5*np.tanh(motor_speeds*0.2) + 0.02*motor_speeds
motor_torques = data_array[:, 9:13]
carriage_positions = data_array[:, 13:16]
tensions = data_array[:, 16:20]
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


# plt.plot(end_effector_pos[:,0],end_effector_pos[:,1])
# plt.show()

t0, te = 17, 40

colors = ['r-', 'b--', 'g-.']
limits = [0,0,0]
fig = plt.figure(figsize=(7., 5.))
gs = fig.add_gridspec(3, hspace=0.08)
axes = gs.subplots(sharex=True, sharey=False)
# fig.add_subplot(111, frameon=False)
for j in range(3):
    axes[j].plot(t-t0, tensions_model[:, j], colors[j], label=str(j))
    axes[j].scatter(t-t0, tensions[:, j], s = 2.5, color = colors[j][0], alpha = 0.1)
    axes[j].grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
    axes[j].grid(True)
    minim = np.min(tensions_model[:, j])
    maxim = np.max(tensions_model[:, j])
    axes[j].set_ylim([minim-3, maxim+3])
fig.supylabel(r'Tensions $T$ (N)')
fig.supxlabel(r'Time  $t$')
plt.xlim(t0, te)
plt.tight_layout()
plt.savefig(f'plots/tensions_time_{file_label}.png')
plt.show()


fig = plt.figure(figsize=(7., 5.))
gs = fig.add_gridspec(3, hspace=0.08)
axes = gs.subplots(sharex=True, sharey=False)
# fig.add_subplot(111, frameon=False)
for j in range(3):
    axes[j].plot(t-t0, torques_model[:, j], colors[j], label=str(j))
    axes[j].scatter(t-t0, motor_torques[:, j], s = 2.5, color = colors[j][0], alpha = 0.1)
    axes[j].grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
    axes[j].grid(True)
    minim = np.min(torques_model[:, j])
    maxim = np.max(torques_model[:, j])
    axes[j].set_ylim([minim-3, maxim+3])
fig.supylabel(r'Torques $\tau$ (mNm)')
fig.supxlabel(r'Time  $t$')
plt.xlim(t0, te)
plt.tight_layout()
plt.savefig(f'plots/torques_time_{file_label}.png')
plt.show()


