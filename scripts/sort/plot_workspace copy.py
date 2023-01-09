import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

plt.style.use('plots/test_style.mplstyle')

show = False

planes = ['xy', 'zy']
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
        coordinates_1 = plane_x
        coordinates_2 = plane_y
        coordinates_1_interp = np.linspace(plane_x.min(), plane_x.max(), 500)
        coordinates_2_interp = np.linspace(plane_y.min(), plane_y.max(), 500)
    if plane == 'zy':
        coordinates_1 = plane_z
        coordinates_2 = plane_y
        coordinates_1_interp = np.linspace(plane_z.min(), plane_z.max(), 500)
        coordinates_2_interp = np.linspace(plane_y.min(), plane_y.max(), 500)

    coordinate_1_grid, coordinate_2_grid = np.meshgrid(coordinates_1_interp,
                                                       coordinates_2_interp)

    torques_norm = np.linalg.norm(data_dict[plane]['torques'], axis=1)
    tensions_norm = np.linalg.norm(data_dict[plane]['tensions'], axis=1)
    cond_m = data_dict[plane]['jac_m_cond']
    cond_d = data_dict[plane]['jac_d_cond']

    criterias = [torques_norm,
                 tensions_norm,
                 cond_m,
                 cond_d]

    labels = [[r'Norm of the motor torques $\|\tau\|$', 'torques_norm'],
              [r'Norm of the string tensions $\|T\|$', 'tensions_norm'],
              [r'Condition number of $\mathcal{J}_m$', 'cond_m'],
              [r'Condition number of $\mathcal{J}_d$', 'cond_d']]

    if plane == 'xy':
        limit = [[-105, 105], [-105+15, 105+15], (10, 8)]
    if plane == 'zy':
        limit = [[10,105], [-80,110], (6, 8)]

    for criteria, label in zip(criterias, labels):
        plot_grid = griddata(points=np.vstack((coordinates_1, coordinates_2)).T,
                             values=criteria,
                             xi=(coordinate_1_grid, coordinate_2_grid),
                             method='cubic')

        plt.figure(figsize=limit[2])
        # plt.title(r'Norm of the motor torques $\|\tau\|$')
        plt.title(label[0])
        plt.contourf(coordinate_1_grid, coordinate_2_grid,
                     plot_grid, 8, alpha=0.5)
        plt.colorbar()
        contours = plt.contour(
            coordinate_1_grid, coordinate_2_grid, plot_grid, 8)
        plt.clabel(contours, inline=True, fontsize=8)
        # if plane == 'xy':
        plt.xlim(limit[0])
        plt.ylim(limit[1])
        plt.tight_layout()
        plt.savefig('plots/wspace_' + str(plane) + '_' + label[1])
        plt.show()

        # if plane == 'xy':
        #     plt.xlabel(r'$X$ (mm)')
        #     plt.ylabel(r'$Y$ (mm)')
        #     plt.axis('square')
        # else:
