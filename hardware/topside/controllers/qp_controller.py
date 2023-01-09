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


def get_torques(jacobian_dev, jacobian_strings, force, pretension=1, weights=np.eye(4)):

    jacobian_s = jacobian_strings  # np.diag(jacobian_strings)

    torques = quadprog_solve_qp(weights,
                                np.zeros(4),
                                -jacobian_s,
                                -pretension*np.ones(4),
                                A=jacobian_dev.T,
                                b=force)
    return torques


def qp_posture_torques(jacobian_dev,
                       jacobian_strings,
                       force,
                       tension_bounds=[0.5, 100],
                       torque_bounds=[-100, 100],
                       weights=np.eye(4)):

    jacobian_s = jacobian_strings  # np.diag(jacobian_strings)

    # parse bounds
    tension_min = tension_bounds[0]
    tension_max = tension_bounds[1]
    torque_min = torque_bounds[0]
    torque_max = torque_bounds[1]

    # Inequality constraints
    P = jacobian_dev @ jacobian_dev.T + 0.01*weights
    q = - jacobian_dev @ force

    G = np.vstack([
        -jacobian_s,
        jacobian_s,
        -np.eye(4),
        np.eye(4)
    ])

    # Equality constraints
    h = np.hstack([
        -tension_min*np.ones(4),
        tension_max*np.ones(4),
        -torque_min*np.ones(4),
        torque_max*np.ones(4)
    ])

    # A = jacobian_dev.T
    # b = force

    torques = quadprog_solve_qp(P,
                                q=q,
                                G=G,
                                h=h)

    return torques
