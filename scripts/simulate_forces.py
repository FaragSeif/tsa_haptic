from matplotlib import cm
import numpy as np
import scipy as sc
from models import *
from calculations import *
import matplotlib.pyplot as plt
from misc import generate_sphere_points, generate_rectangle_points
from scipy.spatial import ConvexHull, convex_hull_plot_2d, Delaunay


def _centroid_poly(poly):

    T = Delaunay(poly).simplices
    n = T.shape[0]
    W = np.zeros(n)
    C = 0

    for m in range(n):
        sp = poly[T[m, :], :]
        W[m] = ConvexHull(sp).volume
        C += W[m] * np.mean(sp, axis=0)

    return C / np.sum(W)

# TODO:
# write the script to store all the criterias in the dictionary


plt.style.use('plots/test_style.mplstyle')


samples = 10000
torque_pos = generate_sphere_points(4, samples=samples, positive_sector=True)
torque_all = generate_sphere_points(4, samples=samples)

# torque_bounds = np.array([1,1,1,1])
# torque_pos = generate_rectangle_points(np.zeros(4), torque_bounds, samples = samples)
# torque_all = generate_rectangle_points(-torque_bounds, torque_bounds, samples = samples)

r_e_calc = np.array([0, 0, 70])
X_i = inverse_kinematics(r_e_calc)
scale = np.array([1, 1, 1, 2])
jacobian_m, jacobian_d, jac_tsa = jacobians(X_i)
forces_pos = jacobian_d.T @ np.diag(scale) @ torque_pos.T
forces_all = jacobian_d.T @ np.diag(scale) @ torque_all.T

forces_xy_pos = -forces_pos[:2, :].T
forces_zy_pos = -np.array([forces_pos[2], forces_pos[1]]).T

forces_xy_all = -forces_all[:2, :].T
forces_zy_all = -np.array([forces_all[2], forces_all[1]]).T

xy_pos_hull = ConvexHull(forces_xy_pos)
zy_pos_hull = ConvexHull(forces_zy_pos)

xy_all_hull = ConvexHull(forces_xy_all)
zy_all_hull = ConvexHull(forces_zy_all)

pos_hull = ConvexHull(-forces_pos.T)
all_hull = ConvexHull(-forces_all.T)


# we can check that points is within hull
# print(pos_hull.equations)
# A = pos_hull.equations[:,:3]
# b = pos_hull.equations[:,3]
# hull_defect = ((A@forces_pos).T - b)>=-0.1
# print
# plt.plot(hull_defect[0])
# plt.show()


force_all_xy_vertices = np.array(
    [forces_xy_all[xy_all_hull.vertices, 0], forces_xy_all[xy_all_hull.vertices, 1]])
force_all_xy_norms = np.linalg.norm(force_all_xy_vertices, axis=0)
force_all_xy_cond = np.max(force_all_xy_norms)/np.min(force_all_xy_norms)


force_all_zy_vertices = np.array(
    [forces_zy_all[zy_all_hull.vertices, 0], forces_zy_all[zy_all_hull.vertices, 1]])
force_all_zy_norms = np.linalg.norm(force_all_zy_vertices, axis=0)
force_all_zy_cond = np.max(force_all_zy_norms)/np.min(force_all_zy_norms)

# print(force_all_xy_cond)

force_all_vert = forces_all.T[all_hull.vertices]
force_all_norms = np.linalg.norm(force_all_vert, axis=1)
# print(force_all_norms)
force_all_cond = np.max(force_all_norms)/np.min(force_all_norms)
# print(np.max(force_all_norms), np.min(force_all_norms))
U, S, V = np.linalg.svd(jacobian_d.T @ np.diag([1, 1, 1, 2]))
# print(4*np.pi*np.prod(S)/3)
# print(S[0]/S[-1])
# print(force_all_cond)

print(f'Condition number of regular hull {force_all_cond}')
print(f'Volume of regular hull {all_hull.volume}')

force_pos_xy_vertices = np.array(
    [forces_xy_pos[xy_pos_hull.vertices, 0], forces_xy_pos[xy_pos_hull.vertices, 1]])
force_pos_xy_norms = np.linalg.norm(force_pos_xy_vertices, axis=0)

max_point_ind_xy = np.argmax(force_pos_xy_norms)
min_point_ind_xy = np.argmin(force_pos_xy_norms)
max_point_xy = force_pos_xy_vertices[:, max_point_ind_xy]
min_point_xy = force_pos_xy_vertices[:, min_point_ind_xy]

print(max_point_xy, min_point_xy)
force_pos_xy_cond = np.max(force_pos_xy_norms)/np.min(force_pos_xy_norms)

force_pos_zy_vertices = np.array(
    [forces_zy_pos[zy_pos_hull.vertices, 0], forces_zy_pos[zy_pos_hull.vertices, 1]])
force_pos_zy_norms = np.linalg.norm(force_pos_zy_vertices, axis=0)
force_pos_zy_cond = np.max(force_pos_zy_norms)/np.min(force_pos_zy_norms)

# plr.plot()

print(np.max(force_pos_zy_norms))
force_pos_vert = forces_pos.T[pos_hull.vertices]
force_pos_norms = np.linalg.norm(force_pos_vert, axis=1)
force_pos_cond = np.max(force_pos_norms)/np.min(force_pos_norms)
print(np.max(force_pos_norms), np.min(force_pos_norms))
print(f'Condition number of "positive" hull {force_pos_cond}')
print(f'Volume of "positive" hull {pos_hull.volume}')
print(f'Cube root of volume {np.cbrt(pos_hull.volume)}')
print(f'Fraction of volume {all_hull.volume/pos_hull.volume}')
print(np.mean(forces_zy_pos[zy_pos_hull.vertices], axis=0))
# xy_centroid_pos = _centroid_poly(forces_xy_pos)
# print(xy_centroid_pos)
# print(np.linalg.norm(xy_centroid_pos))
xy_centroid_pos = np.zeros(2)
# zy_centroid_pos = _centroid_poly(forces_zy_pos)
# print(zy_centroid_pos)
# print(np.linalg.norm(zy_centroid_pos))
# /////////////////////////////////

plt.figure(figsize=(3, 3), dpi=300)

plt.fill(force_all_xy_vertices[0],
         force_all_xy_vertices[1],
         'red', alpha=0.1)

plt.fill(force_pos_xy_vertices[0],
         force_pos_xy_vertices[1],
         'white')
plt.scatter(xy_centroid_pos[0], xy_centroid_pos[1], color='blue')

plt.plot([0, max_point_xy[0]], [0, max_point_xy[1]],
         linestyle='--', marker='o', color='blue')
plt.plot([0, min_point_xy[0]], [0, min_point_xy[1]],
         linestyle='--', marker='o', color='blue')

#  linestyle='dashed',
#      linewidth=2, markersize=12)

for simplex in xy_pos_hull.simplices:
    plt.plot(forces_xy_pos[simplex, 0],
             forces_xy_pos[simplex, 1], 'b-', linewidth=2)
plt.fill(forces_xy_pos[xy_pos_hull.vertices, 0],
         forces_xy_pos[xy_pos_hull.vertices, 1], 'blue', alpha=0.1)
for simplex in xy_all_hull.simplices:
    plt.plot(forces_xy_all[simplex, 0],
             forces_xy_all[simplex, 1], 'r-', linewidth=2)
ax = plt.gca()
ax.set_aspect('equal')
plt.savefig('plots/reach_forces_xy.png')
plt.show()


# import matplotlib.pyplot as plt
# from matplotlib import cm
# plt.style.use('_mpl-gallery')


# Plot
# fig, ax = plt.subplots(subplot_kw={'projection': '3d'})
fig, ax = plt.subplots(subplot_kw={'projection': '3d'})
ax.plot_trisurf(-forces_pos.T[:,0],
                -forces_pos.T[:,2],
                -forces_pos.T[:,1], triangles = pos_hull.simplices,alpha = 0.7, cmap=plt.cm.Spectral)# vmin=z.min() * 2, cmap=cm.Blues)

# ax.set(xticklabels=[],
#        yticklabels=[],
#        zticklabels=[])

ax.set_box_aspect([1,2,1])
ax.view_init(elev=20., azim=20)
plt.show()
