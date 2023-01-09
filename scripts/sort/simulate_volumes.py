import numpy as np
import scipy as sc
from models import *
from calculations import *
import matplotlib.pyplot as plt

plt.style.use('plots/test_style.mplstyle')


planes = {'xy': {}, 'zy': {}}

end_eff_list = []
jac_m_cond = []
jac_d_cond = []
xy_norm = []
yz_norm = []
# X_list =         print(X_i)[]
torques_max = np.array([30, 30, 30, 30])
torques_min = np.array([0, 0, 0, 0])
# tensions_min = -torques_max
torques_samples = 20000
torques_pos = torques_min + (torques_max-torques_min)*np.random.rand(torques_samples, 4)
torque_all = -torques_max + 2*torques_max*np.random.rand(torques_samples, 4)


unit_vectors = np.random.randn(torques_samples, 4)
sphere = np.diag(1/np.linalg.norm(unit_vectors, axis = 1)) @ unit_vectors
sector_sphere = np.abs(sphere)
# sector_sphere = sphere
torques_pos = sector_sphere
torque_all = sphere

r_e_calc = np.array([0, 0, 60])
X_i = inverse_kinematics(r_e_calc)
print(X_i)
jacobian_m, jacobian_d, jac_tsa = jacobians(X_i)
# tensions = np.linalg.inv(jac_tsa) @ torques.T
forces_pos = jacobian_d.T @ np.diag([1,1,1,2]) @ torques_pos.T
forces_all = jacobian_d.T @ np.diag([1,1,1,2]) @torque_all.T
U, S, VT = sc.linalg.svd(jacobian_d.T, full_matrices=False)
print(S)
# print(U @ )

# # force_svd = (U @ np.diag(S)).T @ torques.T
# # # plt.show()
# plt.scatter(forces_all[0], forces_all[1])
# plt.scatter(forces_pos[0], forces_pos[1])
# torques = sector_sphere
# # plt.scatter(force_svd[0], force_svd[1])
# plt.show()
# plt.scatter(forces_all[2], forces_all[1])
# plt.scatter(forces_pos[2], forces_pos[1])
# plt.show()

from scipy.spatial import ConvexHull, convex_hull_plot_2d
# rng = np.random.default_rng()
# points = rng.random((30, 2))   # 30 random points in 2-D
forces_xy_pos = forces_pos[:2, :].T
forces_zy_pos = np.array([forces_pos[2], forces_pos[1]]).T

forces_xy_all = forces_all[:2, :].T
forces_zy_all = np.array([forces_all[2], forces_all[1]]).T

# print(forces_all[:2, :])
xy_pos_hull = ConvexHull(forces_xy_pos)
zy_pos_hull = ConvexHull(forces_zy_pos)

xy_all_hull = ConvexHull(forces_xy_all)
zy_all_hull = ConvexHull(forces_zy_all)

pos_hull = ConvexHull(forces_pos.T)
all_hull = ConvexHull(forces_all.T)

print(pos_hull.volume, all_hull.volume)
print(np.mean(forces_pos.T[pos_hull.vertices], axis = 0))
print(forces_pos.T[pos_hull.vertices])


plt.fill(forces_xy_all[xy_all_hull.vertices,0], forces_xy_all[xy_all_hull.vertices,1], 'red', alpha=0.1)
plt.fill(forces_xy_pos[xy_pos_hull.vertices,0], forces_xy_pos[xy_pos_hull.vertices,1], 'white')    

for simplex in xy_pos_hull.simplices:
    plt.plot(forces_xy_pos[simplex, 0], forces_xy_pos[simplex, 1], 'b-')
plt.fill(forces_xy_pos[xy_pos_hull.vertices,0], forces_xy_pos[xy_pos_hull.vertices,1], 'blue', alpha=0.1)
for simplex in xy_all_hull.simplices:
    plt.plot(forces_xy_all[simplex, 0], forces_xy_all[simplex, 1], 'r-')    
plt.show()



torques_samples = 5000
torques_pos = torques_min + (torques_max-torques_min)*np.random.rand(torques_samples, 4)
torque_all = -torques_max + 2*torques_max*np.random.rand(torques_samples, 4)



torques_samples = 5000
unit_vectors = np.random.randn(torques_samples, 4)
sphere = np.diag(1/np.linalg.norm(unit_vectors, axis = 1)) @ unit_vectors
sector_sphere = np.abs(sphere)
# sector_sphere = sphere
torques_pos = sector_sphere
torque_all = sphere



r_z = np.linspace(-40, 40, 100)

z_volumes = []
x_volumes = []
y_volumes = [] 

for rz_i in r_z:
    r_e_calc = np.array([rz_i, 0, 60])
    X_i = inverse_kinematics(r_e_calc)
    # print(X_i)
    jacobian_m, jacobian_d, jac_tsa = jacobians(X_i)
    forces_pos = -jacobian_d.T @ np.diag([1,1,1,2]) @ torques_pos.T
    forces_all = -jacobian_d.T @ np.diag([1,1,1,2]) @torque_all.T
    pos_hull = ConvexHull(forces_pos.T)
    all_hull = ConvexHull(forces_all.T)

    # print(pos_hull.volume, all_hull.volume)
    z_volumes.append(pos_hull.volume,)

plt.plot(r_z, z_volumes)   
plt.show()





# print(r_z)

# plt.fill(forces_xy_all[xy_all_hull.vertices,0], forces_xy_all[xy_all_hull.vertices,1], 'red', alpha=0.1)
# plt.fill(forces_xy_pos[xy_pos_hull.vertices,0], forces_xy_pos[xy_pos_hull.vertices,1], 'white')    

# for simplex in xy_pos_hull.simplices:
#     plt.plot(forces_xy_pos[simplex, 0], forces_xy_pos[simplex, 1], 'b-')
# plt.fill(forces_xy_pos[xy_pos_hull.vertices,0], forces_xy_pos[xy_pos_hull.vertices,1], 'blue', alpha=0.1)
# for simplex in xy_all_hull.simplices:
#     plt.plot(forces_xy_all[simplex, 0], forces_xy_all[simplex, 1], 'r-')    
# plt.show()





# from shapely.geometry import Polygon
# import matplotlib.pyplot as plt
# plt.rcParams["figure.figsize"] = [7.00, 3.50]
# plt.rcParams["figure.autolayout"] = True
# polygon1 = Polygon(forces_all[:2, :].T)
# x, y = polygon1.exterior.xy
# plt.plot(x, y, c="red")
# plt.show()


# print(np.linalg.)



# print(S)
# print(U)
# print(VT)
# print(U @ np.diag(S) @ VT)
# print(U @ np.diag(S) @ VT @ torques)

# for plane in ['xy', 'zy']:

#     if plane == 'xy':
#         re_x_rand = -80 + 160*np.random.rand(samples)
#         re_y_rand = -80 + 190*np.random.rand(samples)
#         re_z_rand = 50*np.ones(samples)

#     if plane == 'zy':
#         re_x_rand = np.zeros(samples)
#         re_y_rand = -80 + 190*np.random.rand(samples)
#         re_z_rand = 160*np.random.rand(samples)

#     planes[plane]['r_e'] = []
#     planes[plane]['contraction'] = []
#     planes[plane]['jac_m_cond'] = []
#     planes[plane]['jac_d_cond'] = []
#     planes[plane]['tensions'] = []
#     planes[plane]['torques'] = []

#     for i in range(samples):
#         r_e_calc = np.array([re_x_rand[i], re_y_rand[i], re_z_rand[i]])
#         X_i = inverse_kinematics(r_e_calc)

#         cons_1 = np.all(np.vstack([np.cos(phi_angles), np.sin(
#             phi_angles)]).T @ r_e_calc[:2] <= 90*np.ones(3))
#         cons_2 = np.all(-np.vstack([np.cos(phi_angles),
#                         np.sin(phi_angles)]).T @ r_e_calc[:2] <= 58*np.ones(3))
#         cons_3 = np.all(np.zeros(4) <= X_i)
#         cons_4 = np.all(X_i <= np.array(L)*0.3)

#         if cons_1 and cons_2 and cons_3 and cons_4:
#             jacobian_m, jacobian_d = jacobians(X_i)

#             tensions = get_tensions(jacobian_m,
#                                     force,
#                                     pretension=1)

#             jacobians_s = np.zeros(4)
#             for j in range(4):
#                 jacobians_s[j] = tsa_jacobian_x(X_i[j], L[j], r[j])
#             torques = get_torques(jacobian_d,
#                                   jacobians_s,
#                                   force,
#                                   pretension=1)

#             # print(jacobian_s)

#             planes[plane]['r_e'].append(r_e_calc)
#             planes[plane]['jac_m_cond'].append(np.linalg.cond(jacobian_m))
#             planes[plane]['jac_d_cond'].append(np.linalg.cond(jacobian_d))
#             planes[plane]['contraction'].append(X_i)
#             planes[plane]['tensions'].append(tensions)
#             planes[plane]['torques'].append(torques)

#     planes[plane]['r_e'] = np.array(planes[plane]['r_e'])
#     planes[plane]['contraction'] = np.array(planes[plane]['contraction'])

#     np.save('plane_' + plane,
#             r_e=planes[plane]['r_e'],
#             contraction=planes[plane]['contraction'],
#             jac_m_cond=planes[plane]['jac_m_cond'],
#             jac_d_cond=planes[plane]['jac_d_cond'],
#             tensions=planes[plane]['tensions'],
#             torques=planes[plane]['torques'])
