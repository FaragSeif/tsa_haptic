import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

plt.style.use('plots/test_style.mplstyle')

show = False

planes = ['xy', 'zy']
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
        coordinates_1 = plane_x
        coordinates_2 = plane_y
        coordinates_1_interp = np.linspace(plane_x.min(), plane_x.max(), 300)
        coordinates_2_interp = np.linspace(plane_y.min(), plane_y.max(), 300)
    if plane == 'zy':
        coordinates_1 = plane_z
        coordinates_2 = plane_y
        coordinates_1_interp = np.linspace(plane_z.min(), plane_z.max(), 300)
        coordinates_2_interp = np.linspace(plane_y.min(), plane_y.max(), 400)

    coordinate_1_grid, coordinate_2_grid = np.meshgrid(coordinates_1_interp,
                                                       coordinates_2_interp)

    # torques_norm = np.linalg.norm(data_dict[plane]['torques'], axis=1)
    # tensions_norm = np.linalg.norm(data_dict[plane]['tensions'], axis=1)
    
    volume_m = data_dict[plane]['jac_m_vol']
    volume_d = data_dict[plane]['jac_d_vol']
    cond_m = data_dict[plane]['jac_m_cond']
    cond_d = data_dict[plane]['jac_d_cond']

    criterias = [volume_m,
                 volume_d,
                 cond_m,
                 cond_d]

    labels = [[r'Volume of the mechanism  hull', 'volume_m'],
              [r'Volume of the device hull', 'volume_d'],
              [r'Condition number of $\mathcal{J}_m$', 'cond_m'],
              [r'Condition number of $\mathcal{J}_d$', 'cond_d']]

    if plane == 'xy':
        limit = [[-75, 75], [-65, 85], (6, 5)]
    if plane == 'zy':
        limit = [[40,115], [-65, 85], (3.5, 5)]

    for criteria, label in zip(criterias, labels):
        plot_grid = griddata(points=np.vstack((coordinates_1, coordinates_2)).T,
                             values=criteria/np.max(criteria),
                             xi=(coordinate_1_grid, coordinate_2_grid),
                             method='cubic')

        plt.figure(figsize=limit[2])
        # plt.title(r'Norm of the motor torques $\|\tau\|$')
        plt.title(label[0])
        plt.contourf(coordinate_1_grid, coordinate_2_grid,
                     plot_grid, 5, alpha=0.4)
        plt.colorbar()
        contours = plt.contour(coordinate_1_grid, coordinate_2_grid, plot_grid, 5)

        plt.clabel(contours, inline=True, fontsize=8)
        # if plane == 'xy':
        plt.xlim(limit[0])
        plt.ylim(limit[1])
        plt.tight_layout()
        plt.savefig('plots/wspace_' + str(plane) + '_' + label[1], dpi = 300)
        ax = plt.gca()
        ax.set_aspect('equal')
        plt.show()



        # if plane == 'xy':
        #     plt.xlabel(r'$X$ (mm)')
        #     plt.ylabel(r'$Y$ (mm)')
        #     plt.axis('square')
        # else:
