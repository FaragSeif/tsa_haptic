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

traj_type = 'var_spiral'

file_label = f'{traj_type}_motion_slow_motors_5kg'
# file_label = 'var_spiral_motion_slow_motors_5kg'


data_array = np.genfromtxt(f'data/{file_label}.csv', delimiter=',')
# print(data_array)
t = data_array[:, 0] - data_array[0, 0]
motor_angles = data_array[:, 1:5]
motor_speeds = data_array[:, 5:9]
friction = 5*np.tanh(motor_speeds*0.2) + 0.02*motor_speeds
motor_torques = data_array[:, 9:13] - friction
carriage_positions = data_array[:, 13:16]
tension_scale = np.array([0.87836188, 0.67126076, 0.66536704, 1])
tensions = tension_scale*data_array[:, 16:20] + \
    np.array([13.58203349, 12.62714948, 5.81913085, 0])
desired_pos = data_array[:, 20:]
end_effector_pos = np.zeros((len(t), 3))
tensions_model = np.zeros((len(t), 3))
torques_model = np.zeros((len(t), 3))
# carriage_positions = np.zeros((len(t), 4))
force = np.array([0, 0, 5*9.81])

for i in range(len(t)):
    end_effector_pos[i, :], jacobian_m, jacobian_d, jac_tsa = full_kinematics(carriage_positions[i, :], motor_angles[i, :])
    # print(jacobian_m)
    tensions_model[i, :] = np.linalg.inv(jacobian_m[:3,:].T) @ force
    torques_model[i,:] = np.linalg.inv(jacobian_d[:3,:].T) @ force


plt.plot(tensions_model)
plt.plot(tensions[:,:3])
plt.show()

tensions_noise = tensions_model - tensions[:,:3]
torques_noise = torques_model - motor_torques[:,:3]
carriage_positions = np.zeros((len(t), 4))


radius = 35
omega = 2
A = radius
z = 60

if traj_type == 'var_spiral':
    A = 0.5*radius*(1 + np.sin(omega*t/6.5))+2
if traj_type == 'circle':
    A = radius 
    
cartesian_pos = np.asarray([A*np.sin(omega*t),
                            A*np.cos(omega*t),
                            z*np.ones(len(t))]).T

# print()

t0, te = 17, 40

for i in range(len(t)):
    # print(cartesian_pos[i,:])
    # print(carriage_positions[i,:3])
    carriage_positions[i,:] = inverse_kinematics(cartesian_pos[i,:])
    # carriage_positions[i,3] = 0
    # print(carriage_positions[i,:])
    motor_angles[i, :] = x2theta(carriage_positions[i, :], L=L[0], r=r[0])
    # print(motor_angles[i, :])
    
    _, jacobian_m, jacobian_d, jac_tsa = full_kinematics(carriage_positions[i, :], motor_angles[i, :])
    end_effector_pos[i, :] = cartesian_pos[i, :]
    # print(jacobian_m)
    tensions_model[i, :] = np.linalg.inv(jacobian_m[:3,:].T) @ force
    torques_model[i,:] = np.linalg.inv(jacobian_d[:3,:].T) @ force


# plt.plot(cartesian_pos[:,0], cartesian_pos[:,1])
# plt.show()

tensions = tensions_model+ tensions_noise
motor_torques = torques_model+ torques_noise/1.3
# plt.plot(t, tensions_model)
# plt.plot(t, )
# plt.show()

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

data_array[:, 0] = t-t0#device_states.timestamp
data_array[:, 1:5] = motor_angles
# data_array[:, 5:9] = device_states.motor_speeds
data_array[:, 9:12] = motor_torques
data_array[:, 13:16] = carriage_positions[:,:3]
data_array[:, 16:19] = tensions
# data_array[:, 20:] = commands.desired_position

np.savetxt(traj_type + '.csv', data_array, delimiter=',')