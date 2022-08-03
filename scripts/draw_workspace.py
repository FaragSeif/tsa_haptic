import numpy as np
from models import *
from scipy.interpolate import griddata
import matplotlib.pyplot as plt
plt.style.use('plots/test_style.mplstyle')

samples = 10000
# X_rand = 0.3*np.diag(L) @ np.random.rand(4, samples)
# np.random.rand(samples)
# Z_fix = 60
planes = {'xy': {}, 'zy': {}}

# end_effector_poses = np.zeros((3, samples))

end_eff_list = []
jac_m_cond = []
jac_d_cond = []
xy_norm = []
yz_norm = []
# X_list = []
for plane in ['xy', 'zy']:

    if plane == 'xy':
        re_x_rand = -80 + 160*np.random.rand(samples)
        re_y_rand = -80 + 190*np.random.rand(samples)
        re_z_rand = 80*np.ones(samples)

    if plane == 'zy':
        re_x_rand = np.zeros(samples)
        re_y_rand = -80 + 190*np.random.rand(samples)
        re_z_rand = 160*np.random.rand(samples)

    planes[plane]['r_e'] = []
    planes[plane]['jac_m_cond'] = []
    planes[plane]['jac_d_cond'] = []

    for i in range(samples):
        r_e_calc = np.array([re_x_rand[i], re_y_rand[i], re_z_rand[i]])
        X_i = inverse_kinematics(r_e_calc)

        cons_1 = np.all(np.vstack([np.cos(phi_angles), np.sin(
            phi_angles)]).T @ r_e_calc[:2] <= 90*np.ones(3))
        cons_2 = np.all(-np.vstack([np.cos(phi_angles),
                        np.sin(phi_angles)]).T @ r_e_calc[:2] <= 58*np.ones(3))
        cons_3 = np.all(np.zeros(4) <= X_i)
        cons_4 = np.all(X_i <= np.array(L)*0.3)

        if cons_1 and cons_2 and cons_3 and cons_4:
            jacobian_m, jacobian_d = jacobians(X_i)
            planes[plane]['r_e'].append(r_e_calc)
            planes[plane]['jac_m_cond'].append(np.linalg.cond(jacobian_m))
            planes[plane]['jac_d_cond'].append(np.linalg.cond(jacobian_d))

    planes[plane]['r_e'] = np.array(planes[plane]['r_e'])


xy_x = planes['xy']['r_e'][:, 0]
xy_y = planes['xy']['r_e'][:, 1]
xy_points = len(xy_x)

zy_y = planes['zy']['r_e'][:, 1]
zy_z = planes['zy']['r_e'][:, 2]
zy_points = len(zy_y)


x_e_interp = np.linspace(xy_x.min(), xy_x.max(), 500)
y_e_interp = np.linspace(xy_y.min(), xy_y.max(), 500)


X_xy_e, Y_xy_e = np.meshgrid(x_e_interp, y_e_interp)


xy_jac_m_grid = griddata(points=np.vstack((xy_x, xy_y)).T,
                         values=planes['xy']['jac_m_cond'], xi=(
                             X_xy_e, Y_xy_e),
                         method='cubic')

xy_jac_d_grid = griddata(points=np.vstack((xy_x, xy_y)).T,
                         values=planes['xy']['jac_d_cond'], xi=(
                             X_xy_e, Y_xy_e),
                         method='cubic')



plt.figure(figsize=(7.5, 6))
plt.contourf(X_xy_e, Y_xy_e, xy_jac_m_grid, 8, alpha=0.5)
plt.colorbar()
contours = plt.contour(X_xy_e, Y_xy_e, xy_jac_m_grid, 8)
plt.clabel(contours, inline=True, fontsize=8)
plt.xlabel(r'$x$ axis (mm)')
plt.ylabel(r'$y$ axis (mm)')
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


# z_e_interp = np.linspace(zy_z.min(), zy_z.max(), 500)
# y_e_interp = np.linspace(zy_y.min(), zy_y.max(), 500)

# Z_zy_e, Y_zy_e = np.meshgrid(z_e_interp, y_e_interp)

# zy_jac_m_grid = griddata(points=np.vstack((zy_z, zy_y)).T,
#                 values=planes['zy']['jac_m_cond'], xi=(Z_zy_e, Y_zy_e),
#                 method='cubic')

# zy_jac_d_grid = griddata(points=np.vstack((zy_z, zy_y)).T,
#                 values=planes['zy']['jac_d_cond'], xi=(Z_zy_e, Y_zy_e),
#                 method='cubic')

# # /// Contour Plot ////

# plt.figure(figsize=(6, 8))
# plt.contourf(Z_zy_e, Y_zy_e, zy_jac_m_grid, 8, alpha=0.5)
# plt.colorbar()
# contours = plt.contour(Z_zy_e, Y_zy_e, zy_jac_m_grid, 8)
# plt.clabel(contours, inline=True, fontsize=8)
# plt.xlabel(r'$Z$ (mm)')
# plt.ylabel(r'$Y$ (mm)')
# plt.ylim([-80,110])
# plt.xlim([10,105])
# plt.tight_layout()
# plt.savefig('plots/wspace_zy_ind_mech')
# plt.show()


# plt.figure(figsize=(6, 8))
# plt.contourf(Z_zy_e, Y_zy_e, zy_jac_d_grid, 10, alpha=0.5)
# plt.colorbar()
# contours = plt.contour(Z_zy_e, Y_zy_e, zy_jac_d_grid, 10)
# plt.clabel(contours, inline=True, fontsize=8)
# plt.xlabel(r'$Z$ (mm)')
# plt.ylabel(r'$Y$ (mm)')
# plt.ylim([-80,110])
# plt.xlim([10,105])
# plt.tight_layout()
# plt.savefig('plots/wspace_zy_ind_dev')
# plt.show()


# ///////////
# SIMULATIONS 
# ///////////
# 
# 1. COMPARISON BETWEEN DEVICE (TSA +MECH) AND MECH JACOBIANS 
# 
# 2. 3 CONTOUR PLOTS
# MAXIMAL FORCES GIVEN BOUNDS ON TORQUES ALONG X, Y, Z

# 
# EXPERIMENTS 
# F_Z = 20 N
# HELIX MOTION 
# TEMPORAL PLOT:
# X axis - (x, y, z, or t) 
# Y axis - TORQUES (1,2, 3 or scalar norm)

# FORCE ESTIMATION