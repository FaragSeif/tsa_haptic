import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

plt.style.use('plots/test_style.mplstyle')

# https://matplotlib.org/stable/api/_as_gen/mpl_toolkits.mplot3d.axes3d.Axes3D.html#mpl_toolkits.mplot3d.axes3d.Axes3D.plot_trisurf

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
        data_dict[plane]['coordinates'] = [plane_x, plane_y]
        data_dict[plane]['interpolated'] = [np.linspace(plane_x.min(), plane_x.max(), 80),
                                            np.linspace(plane_y.min(), plane_y.max(), 80)]

    if plane == 'zy':
        data_dict[plane]['coordinates'] = [plane_z, plane_y]
        data_dict[plane]['interpolated'] = [np.linspace(plane_z.min(), plane_z.max(), 50),
                                            np.linspace(plane_y.min(), plane_y.max(), 80)]

    data_dict[plane]['grid'] = np.meshgrid(*data_dict[plane]['interpolated'])

    if plane == 'xy':
        limit = [[-75, 75], [-65, 85]]
        size = (6, 5)
    if plane == 'zy':
        limit = [[40, 115], [-65, 85]]
        size = (3.5, 5)

    data_dict[plane]['limits'] = limit
    data_dict[plane]['size'] = size


# fig, axs = plt.subplots(1, 2, constrained_layout=True)
# print(axs)
# plt.title(r'Volume of hull over device jacobian')
# print(len(axs))

criteria_labels = ['jac_d_vol',
                   'jac_m_vol',
                   'jac_d_cond',
                   'jac_m_cond']

for cind, criteria_label in enumerate(criteria_labels):
    fig = plt.figure(figsize=(5.2, 3))
    for ind, plane in enumerate(['xy', 'zy']):
        plt.subplot(1, 2, ind+1)

        criteria = data_dict[plane][criteria_label]
        plot_grid = griddata(points=np.vstack(data_dict[plane]['coordinates']).T,
                             values=criteria,
                             xi=(data_dict[plane]['grid'][0],
                                 data_dict[plane]['grid'][1]),
                             method='cubic')

        # plt.title(label[0])
        plt.contourf(data_dict[plane]['grid'][0],
                     data_dict[plane]['grid'][1], plot_grid, 4, alpha=0.3)

        ax = plt.gca()
        ax.set_aspect('equal')

        if plane == 'xy':
            plt.xlabel('x')
            plt.ylabel('y')
            # plt.colorbar(location='left')
        
        if plane == 'zy':
            plt.xlabel('z')
            
            ax.axes.yaxis.set_ticklabels([])
            
            # plt.ylabel('x')
            # plt.tick_params(axis='y',          # changes apply to the x-axis
            #                 which='both',      # both major and minor ticks are affected
            #                 bottom=False,      # ticks along the bottom edge are off
            #                 top=False,         # ticks along the top edge are off
            #                 labelbottom=False)  # labels along the bottom edge are off
            # plt.yticks([], [])
            
            
            # ax.set_yticks([])
            # for minor ticks
            # ax.set_yticks([], minor=True)

            # plt.colorbar(location='right')

        contours = plt.contour(
            data_dict[plane]['grid'][0], data_dict[plane]['grid'][1], plot_grid, 4, linewidths=1.5)
        plt.clabel(contours, inline=True, fontsize=9)

        plt.xlim(data_dict[plane]['limits'][0])
        plt.ylim(data_dict[plane]['limits'][1])

        plt.grid(zorder=-1, alpha=0.4)

    plt.tight_layout()
    plt.savefig('plots/' + criteria_label, dpi=300)
    plt.show()
