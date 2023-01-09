import numpy as np
from models import *
from calculations import *


def quadprog_solve_qp(P, q, G, h, A=None, b=None):
    qp_G = .5 * (P + P.T)   # make sure P is symmetric
    qp_a = -q
    if A is not None:
        qp_C = -np.vstack([A, G]).T
        qp_b = -np.hstack([b, h])
        meq = A.shape[0]
    else:  # no equality constraint
        qp_C = -G.T
        qp_b = -h
        meq = 0
    return quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, meq)[0]


samples = 20000
pretension = 0
force = np.array([0, 0, 20])


planes = {'xy': {}, 'zy': {}}

end_eff_list = []
jac_m_cond = []
jac_d_cond = []
xy_norm = []
yz_norm = []
# X_list =         print(X_i)[]
for plane in ['xy', 'zy']:

    if plane == 'xy':
        re_x_rand = -80 + 160*np.random.rand(samples)
        re_y_rand = -80 + 190*np.random.rand(samples)
        re_z_rand = 50*np.ones(samples)

    if plane == 'zy':
        re_x_rand = np.zeros(samples)
        re_y_rand = -80 + 190*np.random.rand(samples)
        re_z_rand = 160*np.random.rand(samples)

    planes[plane]['r_e'] = []
    planes[plane]['contraction'] = []
    planes[plane]['jac_m_cond'] = []
    planes[plane]['jac_d_cond'] = []
    planes[plane]['tensions'] = []
    planes[plane]['torques'] = []

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
            jacobian_m, jacobian_d, _ = jacobians(X_i)
            
            tensions = get_tensions(jacobian_m, 
                                    force, 
                                    pretension=1)
            
            jacobians_s = np.zeros(4)
            for j in range(4):
                jacobians_s[j] = tsa_jacobian_x(X_i[j], L[j], r[j])
            torques = get_torques(jacobian_d, 
                                  jacobians_s, 
                                  force, 
                                  pretension=1)

            # print(jacobian_s)

            planes[plane]['r_e'].append(r_e_calc)
            planes[plane]['jac_m_cond'].append(np.linalg.cond(jacobian_m))
            planes[plane]['jac_d_cond'].append(np.linalg.cond(jacobian_d))
            planes[plane]['contraction'].append(X_i)
            planes[plane]['tensions'].append(tensions)
            planes[plane]['torques'].append(torques)

    planes[plane]['r_e'] = np.array(planes[plane]['r_e'])
    planes[plane]['contraction'] = np.array(planes[plane]['contraction'])

    np.savez('plane_' + plane,
             r_e=planes[plane]['r_e'],
             contraction=planes[plane]['contraction'],
             jac_m_cond=planes[plane]['jac_m_cond'],
             jac_d_cond=planes[plane]['jac_d_cond'],
             tensions=planes[plane]['tensions'],
             torques=planes[plane]['torques'])
