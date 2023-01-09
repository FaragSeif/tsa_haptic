from time import perf_counter
import numpy as np
from models import *
from haptic_interface.code.tsa_haptic.scripts.sort.calculations import *

import matplotlib.pyplot as plt



plt.style.use('plots/test_style.mplstyle')
samples = 1000
t_fin = 25
file_label = 'cartesian_points_v9'


data_array  = np.genfromtxt(f'data/{file_label}.csv', delimiter=',')
# print(data_array)
t = data_array[:, 0] - data_array[0, 0]
motor_angles = data_array[:, 1:5]
motor_speeds = data_array[:, 5:9] 
motor_torques = data_array[:, 9:13] 
carriage_positions = data_array[:, 13:16]
tensions = data_array[:, 16:20] + np.array([5,5,5,0])
desired_pos = data_array[:, 20:]
end_effector_pos = np.zeros((len(t), 3))

for i in range(len(t)):

    end_effector_pos[i,:],_,_,_ = full_kinematics(carriage_positions[i,:], motor_angles[i,:])
# re, jacobian_m, jacobian_d, jacobian_s = full_kinematics(device_states.carriage_positions, device_states.motor_angles)

ind = 0

contraction_model = theta2x(motor_angles[:,ind], 295, 0.7)

plt.figure(figsize=(9, 3))
plt.plot(motor_angles[:,ind], contraction_model, 'b')
plt.plot(motor_angles[:,ind], carriage_positions[:,ind], 'r')
plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
plt.grid(True)
# plt.xlim(0, t_fin)
plt.ylabel(r'End effector $\mathbf{r}^e$ (mm)')
plt.xlabel(r'Time  $t$')
plt.tight_layout()
# plt.savefig(f'plots/end_effector_{file_label}.png')
plt.show()


plt.figure(figsize=(9, 3))
plt.plot(t, contraction_model, 'b')
plt.plot(t, carriage_positions[:,ind], 'r')
plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
plt.grid(True)
plt.xlim(0, t_fin)
plt.ylabel(r'End effector $\mathbf{r}^e$ (mm)')
plt.xlabel(r'Time  $t$')
plt.tight_layout()
# plt.savefig(f'plots/end_effector_{file_label}.png')
plt.show()

# plt.figure(figsize=(9, 3))
# plt.plot(t, motor_speeds)
# plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
# plt.grid(True)
# plt.xlim(0, t_fin)
# plt.ylabel(r'End effector $\mathbf{r}^e$ (mm)')
# plt.xlabel(r'Time  $t$')
# plt.tight_layout()
# # plt.savefig(f'plots/end_effector_{file_label}.png')
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


# plt.figure(figsize=(9, 3))
# plt.plot(t, carriage_positions)
# plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
# plt.grid(True)
# # plt.xlim(axes['spiral']['z'][0], axes['spiral']['z'][-1])
# plt.xlim(0, t_fin)
# plt.ylabel(r'Carriages $X$ (mm)')
# plt.xlabel(r'Time  $t$')
# plt.tight_layout()
# plt.savefig(f'plots/carriage_pos_{file_label}.png')
# plt.show()


# plt.figure(figsize=(9, 3))
# plt.plot(t, motor_angles[:,:4])
# plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
# plt.grid(True)
# plt.xlim(0, t_fin)
# # plt.xlim(axes['spiral']['z'][0], axes['spiral']['z'][-1])
# plt.ylabel(r'Motor Angles $\theta$ (rad)')
# plt.xlabel(r'Time  $t$')
# plt.tight_layout()
# plt.savefig(f'plots/torques_{file_label}.png')
# plt.show()

# plt.figure(figsize=(9, 3))
# plt.plot(t, motor_torques[:,:4])
# plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
# plt.grid(True)
# plt.xlim(0, t_fin)
# # plt.xlim(axes['spiral']['z'][0], axes['spiral']['z'][-1])
# plt.ylabel(r'Torques $\tau$ (mNm)')
# plt.xlabel(r'Time  $t$')
# plt.tight_layout()
# plt.savefig(f'plots/motor_torques_{file_label}.png')
# plt.show()

# # # 
# plt.figure(figsize=(9, 3))
# plt.plot(t, tensions[:,:4])
# plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
# plt.grid(True)
# plt.xlim(0, t_fin)
# # plt.xlim(axes['spiral']['z'][0], axes['spiral']['z'][-1])
# plt.ylabel(r'Tensions $T$ (N)')
# plt.xlabel(r'Time  $t$')
# plt.tight_layout()
# # plt.savefig(f'plots/tension_{file_label}.png')
# plt.show()

