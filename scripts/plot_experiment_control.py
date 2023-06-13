from time import perf_counter
import numpy as np
from models import *
from calculations import *
from scipy.interpolate import griddata
import matplotlib.pyplot as plt


plt.style.use('plots/test_style.mplstyle')
samples = 1000
t_fin = 15
file_label = 'z_motion_slow_motors_7kg'
# file_label = 'cartesian_circle_4kg'
file_label = 'cartesian_circle_v4'
# file_label = 'cartesian_points_v3'
file_label = 'cartesian_points_v1'
file_label = 'cartesian_points_v2'

data_array = np.genfromtxt(f'data/{file_label}.csv', delimiter=',')
# print(data_array)
t = data_array[:, 0] - data_array[0, 0]
motor_angles = data_array[:, 1:5]
motor_speeds = data_array[:, 5:9]
friction = 5*np.tanh(motor_speeds*0.25) + 0.02*motor_speeds
motor_torques = data_array[:, 9:13] - friction
carriage_positions = data_array[:, 13:16]
tension_scale = np.array([ 0.87836188,0.67126076,0.66536704,0.4])
tensions = tension_scale*data_array[:, 16:20] + np.array([13.58203349, 12.62714948, 5.81913085, 0])
desired_pos = data_array[:, 20:]
end_effector_pos = np.zeros((len(t), 3))
tensions_model = np.zeros((len(t), 3))
torques_model = np.zeros((len(t), 3))
force = np.array([0,0,5*9.81])

for i in range(len(t)):

    end_effector_pos[i, :], _, _, _ = full_kinematics(
        carriage_positions[i, :], motor_angles[i, :])
# re, jacobian_m, jacobian_d, jacobian_s = full_kinematics(device_states.carriage_positions, device_states.motor_angles)


plt.figure(figsize=(7., 2.6))
plt.plot(t, end_effector_pos[:, 0], 'r-', label='x')
plt.plot(t, end_effector_pos[:, 1], 'b--', label='y')
plt.plot(t, end_effector_pos[:, 2], 'g-.', label='z')
plt.step(t, desired_pos[:, 0], linestyle='--',
         lw=1.6, color='black', alpha=0.6, zorder=-1)
plt.step(t, desired_pos[:, 1], linestyle='--',
         lw=1.6, color='black', alpha=0.6, zorder=-1)
plt.step(t, desired_pos[:, 2], linestyle='--',
         lw=1.6, color='black', alpha=0.6, zorder=-1)
# plt.grid()
plt.grid(True)
plt.legend()
plt.xlim(0, t_fin)
plt.ylabel(r'End effector $\mathbf{r}^e$ (mm)')
plt.xlabel(r'Time  $t$')
plt.tight_layout()
plt.savefig(f'plots/end_effector_{file_label}.png')
plt.show()



#
colors = ['b', 'r', 'g', 'purple']
plt.figure(figsize=(7., 2.6))
for j in range(4):
    plt.plot(t, tensions[:, j], color = colors[j], lw = 1.4, label=str(j+1), alpha = 0.8)

# plt.plot(t, tensions[:, 0], 'r-',lw=1.6, label='1')
# plt.plot(t, tensions[:, 1], 'b',lw=1.6, label='2')
# plt.plot(t, tensions[:, 2], color = 'green', 'g',lw=1.6, label='3')
# plt.plot(t, tensions[:, 3], color='purple', label='4')

plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
plt.hlines(75, 0, t_fin, color='black', linestyle='--', linewidth=2.0)
plt.hlines(5, 0, t_fin, color='black', linestyle='--', linewidth=2.0)
plt.grid(True)
plt.legend()
plt.xlim(0, t_fin)
plt.ylabel(r'Tensions $T$ (N)')
plt.xlabel(r'Time  $t$')
plt.tight_layout()
plt.savefig(f'plots/tension_{file_label}.png')
plt.show()


# plt.figure(figsize=(9, 3))
# plt.plot(t, motor_speeds[:,:4])
# plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
# plt.grid(True)
# # plt.xlim(axes['spiral']['z'][0], axes['spiral']['z'][-1])
# plt.xlim(0, t_fin)
# plt.ylabel(r'Motor Speeds $\dot{\theta}$ (rad/s)')
# plt.xlabel(r'Time  $t$')
# plt.tight_layout()
# plt.savefig(f'plots/motor_speeds_{file_label}.png')
# plt.show()

# plt.figure(figsize=(7., 2.5))
# # plt.plot(t, motor_torques[:,:4])
# plt.plot(t, motor_torques[:, 0], 'r-', label='1')
# plt.plot(t, motor_torques[:, 1], 'b--', label='2')
# plt.plot(t, motor_torques[:, 2], 'g-.', label='3')
# plt.legend()
# plt.plot(t, -motor_torques[:, 3], color='orange', label='4')
# plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
# plt.grid(True)
# plt.xlim(0, t_fin)
# # plt.xlim(axes['spiral']['z'][0], axes['spiral']['z'][-1])
# plt.ylabel(r'Torques $\tau$ (mNm)')
# plt.xlabel(r'Time  $t$')
# plt.tight_layout()
# plt.savefig(f'plots/motor_torques_{file_label}.png')
# plt.show()



# plt.figure(figsize=(5, 5))
# plt.plot(end_effector_pos[:,0], end_effector_pos[:,1])
# plt.plot(desired_pos[:,0], desired_pos[:,1])
# plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
# plt.grid(True)
# # plt.xlim(axes['spiral']['z'][0], axes['spiral']['z'][-1])
# # plt.ylabel(r'Torques $\boldsymbol{\tau}$ (mNm)')
# plt.xlabel(r'Time  $t$')
# plt.tight_layout()
# plt.savefig(f'plots/torques_{file_label}.png')
# plt.show()


# plt.figure(figsize=(7., 2.5))
# plt.plot(t, carriage_positions[:, 0], 'r-', label='1')
# plt.plot(t, carriage_positions[:, 1], 'b--', label='2')
# plt.plot(t, carriage_positions[:, 2], 'g-.', label='3')
# plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
# plt.grid(True)
# plt.legend()
# plt.xlim(0, t_fin)
# plt.ylabel(r'Carriages $X$ (mm)')
# plt.xlabel(r'Time  $t$')
# plt.tight_layout()
# plt.savefig(f'plots/carriage_pos_{file_label}.png')
# plt.show()


# plt.figure(figsize=(7., 2.5))
# plt.plot(t, motor_angles[:, 0], 'r-', label='1')
# plt.plot(t, motor_angles[:, 1], 'b--', label='2')
# plt.plot(t, motor_angles[:, 2], 'g-.', label='3')
# plt.plot(t, -motor_angles[:, 3], color='orange', label='4')
# plt.legend()
# plt.grid(True)
# plt.xlim(0, t_fin)
# # plt.xlim(axes['spiral']['z'][0], axes['spiral']['z'][-1])
# plt.ylabel(r'Motor Angles $\theta$ (rad)')
# plt.xlabel(r'Time  $t$')
# plt.tight_layout()
# plt.savefig(f'plots/motor_angles_{file_label}.png')
# plt.show()
