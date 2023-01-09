# from params import *
import numpy as np
from types import SimpleNamespace

# ////////////////////
# MECHANISM PARAMETERS 
mechanism = SimpleNamespace()
mechanism.d = 15  # d
mechanism.D = 110  # D
mechanism.R = 33  # R
mechanism.l = 213.20  # l
mechanism.d_c = 93.5  # mm
mechanism.d_b = 162.5 #
mechanism.rc_4 = np.array([0, 0, -185]) # mm 
mechanism.phi_angles = [2*np.pi*(i)/3 + np.pi/2 for i in range(3)] # rad

# //////////////////
# STRINGS PARAMETERS
strings = SimpleNamespace()
strings.length = [295, 295, 295, 325 ]  # string length
strings.radii = [0.62, 0.62, 0.62, 0.62]  # string radius

# //////////////////
#  MOTOR PARAMETERS 
motors = SimpleNamespace()
motors.inertia = 4*[1]
motors.damping = 4*[1]
motors.torque_constant = 4*[1]
 
