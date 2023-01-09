# from params import *
import numpy as np



# Mechanism parameters
d = 15  # mm
D = 110  # mm
R = 33  # mm
l = 213.20  # mm
d_c = 93.5  # mm
d_b = 162.5
rc_4 = np.array([0, 0, -185]) # mm 
phi_angles = [2*np.pi*(i)/3 + np.pi/2 for i in range(3)] # rad
# ADD PARAMETERS ON INNER TRIANGLE


# TSA parameters
L = [300, 300, 300, 325]  # mm
r = [0.8, 0.8, 0.8, 0.8]  # mm



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


def carriage_pos(lin_pos, phi, L):
    rc_x = (D - d)*np.cos(phi)
    rc_y = (D - d)*np.sin(phi)
    rc_z = lin_pos - d_b#X - L + d_c
    
    return np.array([rc_x, rc_y, rc_z])


frame_hinges_loc_pos = np.array([R*np.cos(phi_angles),
                                 R*np.sin(phi_angles),
                                 np.zeros(3)]).T


# def trilaterate(spheres_centers, spheres_radii):
#     ''' Find the intersection of three spheres,
#         spheres_centers = [p1,p2,p3] are the centers,
#         spheres_radii = [r1,r2,r3] are the radii
#         '''

#     p1, p2, p3 = spheres_centers
#     r1, r2, r3 = spheres_radii

#     temp1 = p2-p1
#     e_x = (temp1)/np.linalg.norm(temp1)

#     temp2 = p3-p1
#     i_ort = e_x @ temp2
#     temp3 = temp2 - i_ort*e_x
    
#     e_y = temp3/np.linalg.norm(temp3)
#     e_z = np.cross(e_x, e_y)
#     d = np.linalg.norm(p2-p1)
#     j_ort = e_y @ temp2
#     x = (r1*r1 - r2*r2 + d*d) / (2*d)
#     y = (r1*r1 - r3*r3 - 2*i_ort*x + i_ort*i_ort + j_ort*j_ort) / (2*j_ort)
#     temp4 = r1*r1 - x*x - y*y
#     if temp4 < 0:
#         raise Exception("The three spheres do not intersect!")
#     z = np.sqrt(temp4)
#     intersection = p1 + x*e_x + y*e_y - z*e_z

#     return intersection


def trilaterate(spheres_centers, spheres_radii):
    P1, P2, P3 =  spheres_centers
    r1, r2, r3 = spheres_radii
    p1 = np.array([0, 0, 0])
    p2 = np.array([P2[0] - P1[0], P2[1] - P1[1], P2[2] - P1[2]])
    p3 = np.array([P3[0] - P1[0], P3[1] - P1[1], P3[2] - P1[2]])
    v1 = p2 - p1
    v2 = p3 - p1

    Xn = (v1)/np.linalg.norm(v1)

    tmp = np.cross(v1, v2)

    Zn = (tmp)/np.linalg.norm(tmp)

    Yn = np.cross(Xn, Zn)

    i = np.dot(Xn, v2)
    d = np.dot(Xn, v1)
    j = np.dot(Yn, v2)

    X = ((r1**2)-(r2**2)+(d**2))/(2*d)
    Y = (((r1**2)-(r3**2)+(i**2)+(j**2))/(2*j))-((i/j)*(X))
    Z1 = np.sqrt(max(0, r1**2-X**2-Y**2))
    Z2 = -Z1

    K1 = P1 + X * Xn + Y * Yn + Z1 * Zn
    K2 = P1 + X * Xn + Y * Yn + Z2 * Zn
    return K1#,K2


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



def full_kinematics(cart_positions, motor_angles):
    '''Calculate mechanism and device Jacobians, as well as forward kinematics based on 
       carriage positions'''

    jac_m = np.zeros((4, 3))
    jac_d = np.zeros((4, 3))
    jac_s = np.zeros((4, 4))
    r_e = forward_kinematics(cart_positions)

    for i in range(4):
        if i == 3:
            delta_r = rc_4 - r_e
            jac_m[i] = delta_r/np.linalg.norm(delta_r)
        else:
            rc_i = carriage_pos(cart_positions[i], phi_angles[i], L[i])
            rh_i = frame_hinges_loc_pos[i]
            rf_i = r_e + rh_i
            jac_m[i] = (rc_i - rf_i)/(rc_i[2] - r_e[2])
        jac_s[i,i] = tsa_jacobian_theta(motor_angles[i], L[i], r[i])
        jac_d[i] = jac_m[i]/jac_s[i,i]

    return r_e, jac_m, jac_d, jac_s



# def kinematics()