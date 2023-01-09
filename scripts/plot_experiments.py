import numpy as np
from models import *
from calculations import *
from misc import generate_rectangle_points, generate_sphere_points
from scipy.spatial import ConvexHull
torque_samples = 8000
torque_pos = generate_sphere_points(4, samples=torque_samples, positive_sector=True)
# torque_all = generate_sphere_points(4, samples=torque_samples)
# pretension = 0

planes = {'xy': {}, 'zy': {}}

end_eff_list = []
jac_m_cond = []
jac_d_cond = []

xy_norm = []
yz_norm = []


samples = 10000
# X_list =         print(X_i)[]

for plane in ['xy', 'zy']:

    if plane == 'xy':
        re_x_rand = -70 + 140*np.random.rand(samples)
        re_y_rand = -70 + 180*np.random.rand(samples)
        re_z_rand = 75*np.ones(samples)
        # torques = force
    if plane == 'zy':
        re_x_rand = np.zeros(samples)
        re_y_rand = -70 + 180*np.random.rand(samples)
        re_z_rand = 50+100*np.random.rand(samples)
        # forces = 

    planes[plane]['r_e'] = []
    planes[plane]['contraction'] = []
    planes[plane]['jac_m_cond'] = []
    planes[plane]['jac_d_cond'] = []
    planes[plane]['jac_d_vol'] = []
    planes[plane]['jac_m_vol'] = []
    # planes[plane]['tensions'] = []
    # planes[plane]['torques'] = []

    for i in range(samples):
        r_e_calc = np.array([re_x_rand[i], re_y_rand[i], re_z_rand[i]])
        X_i = inverse_kinematics(r_e_calc)

        cons_1 = np.all(np.vstack([np.cos(phi_angles), np.sin(phi_angles)]).T @ r_e_calc[:2] <= 75*np.ones(3))
        cons_2 = np.all(-np.vstack([np.cos(phi_angles),
                        np.sin(phi_angles)]).T @ r_e_calc[:2] <= 58*np.ones(3))
        cons_3 = np.all(np.zeros(4) <= X_i)
        cons_4 = np.all(X_i <= np.array(L)*0.4)
        # cons_4 = True

        if cons_1 and cons_2 and cons_3 and cons_4:
            print(round(100*i/samples, 2))
            jacobian_m, jacobian_d, jacobians_s = jacobians(X_i)
            jacobians_dict = {}
            jacobians_dict['m'] = jacobian_m
            jacobians_dict['d'] = jacobian_d
            scale = np.array([1, 1, 1, 1])

            for jac_label in ['m','d']:
                jac = jacobians_dict[jac_label] 
                forces_pos = jac.T @ np.diag(scale) @ torque_pos.T

                if plane == 'xy':
                    forces_plane = -forces_pos[:2, :].T
                if plane == 'zy':
                    forces_plane = -np.array([forces_pos[2], forces_pos[1]]).T
                forces = -forces_pos.T

                # hull = ConvexHull(forces_plane)
                # vertices = forces_plane[hull.vertices, :]
                # norms = np.linalg.norm(vertices, axis=0)
                
                all_hull = ConvexHull(forces)
                volume = all_hull.volume
                vertices = forces_plane[all_hull.vertices, :]
                norms = np.linalg.norm(vertices, axis=0)
                cond = np.max(norms)/np.min(norms)

                planes[plane][f'jac_{jac_label}_cond'].append(cond)
                planes[plane][f'jac_{jac_label}_vol'].append(volume)
            
            planes[plane]['r_e'].append(r_e_calc)
            planes[plane]['contraction'].append(X_i)
            # planes[plane]['tensions'].append(tensions)
            # planes[plane]['torques'].append(torques)

    planes[plane]['r_e'] = np.array(planes[plane]['r_e'])
    planes[plane]['contraction'] = np.array(planes[plane]['contraction'])

    np.savez('plane_' + plane,
             r_e=planes[plane]['r_e'],
             contraction=planes[plane]['contraction'],
             jac_m_cond=planes[plane]['jac_m_cond'],
             jac_d_cond=planes[plane]['jac_d_cond'],
             jac_m_vol=planes[plane]['jac_m_vol'],
             jac_d_vol=planes[plane]['jac_d_vol'],
            #  tensions=planes[plane]['tensions'],
            #  torques=planes[plane]['force']
             )
