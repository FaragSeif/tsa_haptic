import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

# plt.style.use('plots/test_style.mplstyle')

planes = ['xy', 'zy']
data_dict = {}
for plane in planes:
    data_dict[plane] = {}
    npzfile = np.load('plane_' + plane + '.npz')
    for data in npzfile.files:
        data_dict[plane][data] = npzfile[data]

    r_e = data_dict[plane]['r_e']
    X = data_dict[plane]['contraction']
    # print(r_e)
# print(data_dict['xy']['jac_m_cond'])


xy_x = data_dict['xy']['r_e'][:, 0]
xy_y = data_dict['xy']['r_e'][:, 1]
xy_points = len(xy_x)

zy_y = data_dict['zy']['r_e'][:, 1]
zy_z = data_dict['zy']['r_e'][:, 2]
zy_points = len(zy_y)


x_e_interp = np.linspace(xy_x.min(), xy_x.max(), 500)
y_e_interp = np.linspace(xy_y.min(), xy_y.max(), 500)


X_xy_e, Y_xy_e = np.meshgrid(x_e_interp, y_e_interp)
torques_norm_xy = np.linalg.norm(data_dict['xy']['torques'], axis = 1)
torques_norm_zy = np.linalg.norm(data_dict['zy']['torques'], axis = 1)

tensions_norm_xy = np.linalg.norm(data_dict['xy']['tensions'], axis = 1)
tensions_norm_zy = np.linalg.norm(data_dict['zy']['tensions'], axis = 1)


#data_dict['xy']['jac_d_cond']

print(torques_norm_xy)
print(torques_norm_zy)

print(tensions_norm_xy)
print(tensions_norm_zy)

xy_jac_m_grid = griddata(points=np.vstack((xy_x, xy_y)).T,
                         values=torques_norm_xy, xi=(
                             X_xy_e, Y_xy_e),
                         method='cubic')

xy_jac_d_grid = griddata(points=np.vstack((xy_x, xy_y)).T,
                         values=tensions_norm_xy, xi=(
                             X_xy_e, Y_xy_e),
                         method='cubic')



plt.figure(figsize=(7.5, 6))
plt.contourf(X_xy_e, Y_xy_e, xy_jac_m_grid, 8, alpha=0.5)
plt.colorbar()
contours = plt.contour(X_xy_e, Y_xy_e, xy_jac_m_grid, 8)
plt.clabel(contours, inline=True, fontsize=8)
plt.xlabel(r'$X$ (mm)')
plt.ylabel(r'$Y$ (mm)')
plt.axis('square')
plt.ylim([-105+15,105+15])
plt.xlim([-105,105])
plt.tight_layout()
plt.savefig('plots/wspace_xy_ind_mech')
plt.show()

# # /// Contour Plot ////

plt.figure(figsize=(7.5, 6))

plt.contourf(X_xy_e, Y_xy_e, xy_jac_d_grid, 8, alpha=0.5)
plt.colorbar()
contours = plt.contour(X_xy_e, Y_xy_e, xy_jac_d_grid, 8)
plt.clabel(contours, inline=True, fontsize=8)
plt.xlabel(r'$X$ (mm)')
plt.ylabel(r'$Y$ (mm)')
plt.ylim([-105+15,105+15])
plt.xlim([-105,105])
plt.tight_layout()
plt.savefig('plots/wspace_xy_ind_dev')
plt.show()
