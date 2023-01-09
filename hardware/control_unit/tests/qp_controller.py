import numpy as np
import quadprog

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


def get_tensions(jacobian_mech, force, pretension=1):
    tensions = quadprog_solve_qp(np.eye(4),
                                 np.zeros(4),
                                 -np.eye(4),
                                 -pretension*np.ones(4),
                                 A=jacobian_mech.T,
                                 b=force)
    return tensions


def get_torques(jacobian_dev, jacobian_strings, force, pretension=1):

    jacobian_s = jacobian_strings#np.diag(jacobian_strings)
    torques = quadprog_solve_qp(np.eye(4),
                                np.zeros(4),
                                -jacobian_s,
                                -pretension*np.ones(4),
                                A=jacobian_dev.T,
                                b=force)
    return torques
