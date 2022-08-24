from config import *
import numpy as np

# //////////////////
# /// TSA models ///
# //////////////////


def theta2x(theta, L, r):
    '''Get contraction from motor angle 
       (TSA forward kinematics)'''
    return L - np.sqrt(L**2 - (theta * r)**2)


def x2theta(X, L, r, sign=1):
    '''Get motor angle from contraction 
       (TSA inverce kinematics)'''
    return sign*np.sqrt(L**2 - (L - X)**2)/r


def tsa_jacobian(theta, X, L, r):
    '''Jacobian of TSA as function of 
       contraction and angle'''
    return theta * r**2/(L-X)


def tsa_jacobian_theta(theta, L, r):
    '''Jacobian of TSA as function of motor angle'''
    X = theta2x(theta, L, r)
    return tsa_jacobian(theta, X, L, r)


def tsa_jacobian_x(X, L, r):
    '''Jacobian of TSA as function of contraction'''
    theta = x2theta(X, L, r, sign=1)
    return tsa_jacobian(theta, X, L, r)

# ////////////////////////
# /// Mechanism models ///
# ////////////////////////


def carriage_pos(X, phi, L):
    rc_x = (D - d)*np.cos(phi)
    rc_y = (D - d)*np.sin(phi)
    rc_z = X - L + d_c
    return np.array([rc_x, rc_y, rc_z])


frame_hinges_loc_pos = np.array([R*np.cos(phi_angles),
                                 R*np.sin(phi_angles),
                                 np.zeros(3)]).T


def trilaterate(spheres_centers, spheres_radii):
    ''' Find the intersection of three spheres,
        spheres_centers = [p1,p2,p3] are the centers,
        spheres_radii = [r1,r2,r3] are the radii
        '''

    p1, p2, p3 = spheres_centers
    r1, r2, r3 = spheres_radii

    temp1 = p2-p1
    e_x = (temp1)/np.linalg.norm(temp1)

    temp2 = p3-p1
    i = e_x @ temp2
    temp3 = temp2 - i*e_x
    e_y = temp3/np.linalg.norm(temp3)
    e_z = np.cross(e_x, e_y)
    d = np.linalg.norm(p2-p1)
    j = e_y @ temp2
    x = (r1*r1 - r2*r2 + d*d) / (2*d)
    y = (r1*r1 - r3*r3 - 2*i*x + i*i + j*j) / (2*j)
    temp4 = r1*r1 - x*x - y*y
    if temp4 < 0:
        raise Exception("The three spheres do not intersect!")
    z = np.sqrt(temp4)
    intersection = p1 + x*e_x + y*e_y + z*e_z

    return intersection


def forward_kinematics(contractions):
    '''Solve for end effector posture given
       carriages positions (forward kinematics)
       using trilateration'''

    spheres_centers = 3*[0]
    spheres_radii = np.array([l, l, l])

    for i in range(3):
        rc_i = carriage_pos(contractions[i], phi_angles[i], L[i])
        rh_i = frame_hinges_loc_pos[i]
        spheres_centers[i] = rc_i - rh_i

    end_effector_pos = trilaterate(spheres_centers, spheres_radii)
    return end_effector_pos


def inverse_kinematics(end_eff_pos):
    '''Solve for carriage positions given
       end effector posture (inverse kinematics)'''
    X = np.zeros(4)
    re_x, re_y, re_z = end_eff_pos
    for i in range(4):
        if i == 3:
            X[i] = L[i] - np.linalg.norm(end_eff_pos - rc_4)
        else:
            rd_i = np.array([(D-d-R)*np.cos(phi_angles[i]) - re_x,
                             (D-d-R)*np.sin(phi_angles[i]) - re_y])

            X[i] = re_z + L[i] - np.sqrt(l**2 - rd_i @ rd_i) - d_c

    return X


def jacobians(contractions):
    '''Calculate mechanism and device Jacobians based on 
       carriage positions'''

    jac_m = np.zeros((4, 3))
    jac_d = np.zeros((4, 3))
    r_e = forward_kinematics(contractions)

    for i in range(4):
        if i == 3:
            delta_r = rc_4 - r_e
            jac_m[i] = delta_r/np.linalg.norm(delta_r)
        else:
            rc_i = carriage_pos(contractions[i], phi_angles[i], L[i])
            rh_i = frame_hinges_loc_pos[i]
            rf_i = r_e + rh_i
            jac_m[i] = (rc_i - rf_i)/(contractions[i] - L[i] + d_c - r_e[2])

        jac_d[i] = jac_m[i]/tsa_jacobian_x(contractions[i], L[i], r[i])

    return jac_m, jac_d