from scipy.signal import butter, filtfilt
from time import perf_counter
import numpy as np
from models import *
from calculations import *
from scipy.interpolate import griddata
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.interpolate import griddata


plt.style.use('plots/test_style.mplstyle')
samples = 1000
t_fin = 18

file_label = 'spiral_3d'

data_array = np.genfromtxt(f'data/{file_label}.csv', delimiter=',')
# print(data_array)
t = data_array[:, 0] - data_array[0, 0]
print(np.mean(np.diff(t)))
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
force = np.array([0, 0, 5*9.81])

# Y_T = np.zeros((len(t), 2, 3))
for i in range(len(t)):
    end_effector_pos[i, :], jacobian_m, jacobian_d, jac_tsa = full_kinematics(
        carriage_positions[i, :], motor_angles[i, :])
    # print(jacobian_m)
    tensions_model[i, :] = np.linalg.inv(jacobian_m[:3, :].T) @ force
    torques_model[i, :] = np.linalg.inv(jacobian_d[:3, :].T) @ force


def butter_lowpass_filter(data, cutoff, fs, order):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y


torques_norm = np.linalg.norm(motor_torques, axis=1)
torque_norm_filt = butter_lowpass_filter(torques_norm, 0.1, 100, 5)
torque_model_norm = np.linalg.norm(torques_model, axis=1)


tension_norm = np.linalg.norm(tensions, axis=1)
tension_norm_filt = butter_lowpass_filter(tension_norm, 0.1, 100, 5)
tension_model_norm = np.linalg.norm(tensions_model, axis=1)

plt.plot(t, torque_model_norm)
plt.show()
t0, te = 15, 56
plt.plot(torques_norm[int(t0*100):int(te*100)])
plt.plot(torque_norm_filt[int(t0*100):int(te*100)])
plt.plot(torque_model_norm[int(t0*100):int(te*100)])
plt.show()

# plt.plot(tension_norm[1000:-1000])
# plt.plot(tension_norm_filt[1000:-1000])
# plt.plot(tension_model_norm[1000:-1000])
# plt.show()

# fig = plt.figure()
# ax = plt.axes(projection='3d')
# ax.plot3D(end_effector_pos[1000:-1000,0], end_effector_pos[1000:-1000,1], torque_norm_filt[1000:-1000], 'gray')

# # ax.scatter3D(end_effector_pos[:,0], end_effector_pos[:,1], torque_model_norm_filt, s = 0.1, c=torque_model_norm_filt, cmap='Greens')
# plt.show()


# fig = plt.figure()
# ax = plt.axes(projection='3d')
# ax.plot3D(end_effector_pos[1000:-1000,0], end_effector_pos[1000:-1000,1], tension_norm_filt[1000:-1000], 'gray')

# # ax.scatter3D(end_effector_pos[:,0], end_effector_pos[:,1], torque_model_norm_filt, s = 0.1, c=torque_model_norm_filt, cmap='Greens')
# plt.show()

plt.style.use('plots/test_style.mplstyle')
force = np.array([0, 0, 5*9.81])

show = False
planes = ['xy']
# planes = ['xy']
data_dict = {}
for plane in planes:
    data_dict[plane] = {}
    npzfile = np.load('plane_' + plane + '.npz')
    for data in npzfile.files:
        data_dict[plane][data] = npzfile[data]

    plane_x = data_dict[plane]['r_e'][:, 0]
    plane_y = data_dict[plane]['r_e'][:, 1]
    plane_z = data_dict[plane]['r_e'][:, 2]
    plane_points = len(plane_x)

    if plane == 'xy':
        x_interp = np.linspace(plane_x.min(), plane_x.max(), 300)
        y_interp = np.linspace(plane_y.min(), plane_y.max(), 300)

    x_grid, y_grid = np.meshgrid(x_interp, y_interp)
    # z_grid = griddata(points=np.vstack((plane_x, plane_y)).T,
    #                   values=criteria/np.max(criteria),
    #                   xi=(x_grid, y_grid),
    #                   method='cubic')

tensions_model  = np.zeros((plane_points, 3))
torques_model = np.zeros((plane_points, 3))
for j in range(plane_points):
    cartesian_pos = np.array([plane_x[j], plane_y[j], plane_z[j]])
    carriage_positions = inverse_kinematics(cartesian_pos)
    motor_angles = x2theta(carriage_positions, L=L[0], r=r[0])
    # print(motor_angles[i, :])
    
    _, jacobian_m, jacobian_d, jac_tsa = full_kinematics(carriage_positions, motor_angles)
    tensions_model[j, :] = np.linalg.inv(jacobian_m[:3,:].T) @ force
    torques_model[j,:] = np.linalg.inv(jacobian_d[:3,:].T) @ force

torque_model_norm_3d = np.linalg.norm(torques_model, axis=1)
tension_model_norm_3d = np.linalg.norm(tensions_model, axis=1)

x_grid, y_grid = np.meshgrid(x_interp, y_interp)
# # Plot the surface.
plot_grid = griddata(points=np.vstack((plane_x, plane_y)).T,
                        values=tension_model_norm_3d,
                        xi=(x_grid, y_grid),
                        method='cubic')

from matplotlib import cm
from matplotlib.ticker import LinearLocator

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

periods = 0.5
N0 = int(t0*100)
Ne = int(N0+20.5*periods*100)
surf = ax.plot_surface(x_grid, y_grid, plot_grid, cmap=cm.coolwarm, linewidth=0, antialiased=False, alpha = 0.2)
ax.plot3D(end_effector_pos[N0:Ne,0], end_effector_pos[N0:Ne,1], tension_norm_filt[N0:Ne], 'red')

plt.show()
# # Customize the z axis.
# ax.set_zlim(-1.01, 1.01)
# ax.zaxis.set_major_locator(LinearLocator(10))
# # A StrMethodFormatter is used automatically
# # ax.zaxis.set_major_formatter('{x:.02f}')

# # Add a color bar which maps values to colors.
# fig.colorbar(surf, shrink=0.5, aspect=5)

# plt.show()




# plt.figure(figsize=limit[2])
# # plt.title(r'Norm of the motor torques $\|\tau\|$')
# plt.title(label[0])
# plt.contourf(coordinate_1_grid, coordinate_2_grid,
#                 plot_grid, 5, alpha=0.4)
# plt.colorbar()
# contours = plt.contour(coordinate_1_grid, coordinate_2_grid, plot_grid, 5)

# plt.clabel(contours, inline=True, fontsize=8)
# # if plane == 'xy':
# plt.xlim(limit[0])
# plt.ylim(limit[1])
# plt.tight_layout()
# plt.savefig('plots/wspace_' + str(plane) + '_' + label[1], dpi = 300)
# ax = plt.gca()
# ax.set_aspect('equal')
# plt.show()


# import matplotlib.pyplot as plt
# from matplotlib import cm
# from matplotlib.ticker import LinearLocator
# import numpy as np

# fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

# # Make data.
# X = np.arange(-30, 60, 1)
# Y = np.arange(-45, 45, 1)
# X, Y = np.meshgrid(X, Y)
# # R = np.sqrt(X**2 + Y**2)
# Z = X**2 + Y**2

