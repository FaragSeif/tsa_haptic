from time import perf_counter
import numpy as np
from models import *
from calculations import *
from scipy.interpolate import griddata
import matplotlib.pyplot as plt


plt.style.use('plots/test_style.mplstyle')
samples = 200
t = np.linspace(0, 30, samples)

force = -np.array([0, 0, -30])

R = 45
axes = {'x': {'x': np.linspace(-50, 50, samples),
              'y': np.zeros(samples),
              'z': 80*np.ones(samples),
              'contractions': np.zeros((4, samples)),
              'torques': np.zeros((4, samples)),
              'tensions': np.zeros((4, samples))},
        'y': {'x': np.zeros(samples),
              'y': np.linspace(-50, 80, samples),
              'z': 80*np.ones(samples),
              'contractions': np.zeros((4, samples)),
              'torques': np.zeros((4, samples)),
              'tensions': np.zeros((4, samples))},
        'z': {'x': np.zeros(samples),
              'y': np.zeros(samples),
              'z': np.linspace(60, 120, samples),
              'contractions': np.zeros((4, samples)),
              'torques': np.zeros((4, samples)),
              'tensions': np.zeros((4, samples))},
        'spiral': {'x': R*np.cos(0.1*2*np.pi*t),
                   'y': R*np.sin(0.1*2*np.pi*t),
                   'z': np.linspace(60, 120, samples),
                   'contractions': np.zeros((4, samples)),
                   'torques': np.zeros((4, samples)),
                   'tensions': np.zeros((4, samples))}}

# DOES KINEMATICS WORK CORRECTLY?

pretension = 1

# for axis in ['x', 'y', 'z', 'spiral']:
for axis in ['x', 'y', 'z', 'spiral']:
    for i in range(samples):
        r_x, r_y, r_z = axes[axis]['x'][i], axes[axis]['y'][i], axes[axis]['z'][i]
        r_i = np.array([r_x, r_y, r_z])
        X_i = inverse_kinematics(r_i)
        jacobian_m, jacobian_d, _ = jacobians(X_i)

        # print(jacobian_d)
      #   print(jacobian_d[:3,:])
        
        tensions = get_tensions(jacobian_m,
                                force,
                                pretension=1)

        jacobians_s = np.zeros(4)

        for j in range(4):
            jacobians_s[j] = tsa_jacobian_x(X_i[j], L[j], r[j])

        torques = get_torques(jacobian_d,
                              jacobians_s,
                              force,
                              pretension=2)

        axes[axis]['tensions'][:, i] = tensions
        axes[axis]['torques'][:, i] = torques
        axes[axis]['contractions'][:, i] = X_i


plt.figure(figsize=(9, 3))
plt.plot(axes['x']['x'], np.linalg.norm(axes[axis]['torques'].T, axis = 1))
plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
plt.grid(True)
plt.xlim(axes[axis][axis][0], axes[axis][axis][-1])
plt.ylabel(r'Torques $\mathbf{\tau}$ (mNm)')
plt.xlabel(r'Time  $t$')

plt.savefig('plots/torques_' + str(axis) + '.png')
plt.show()

# for axis in ['x', 'y', 'z']:
#     plt.figure(figsize=(9, 3))
#     plt.plot(axes[axis][axis], np.linalg.norm(axes[axis]['torques'].T, axis = 1))
#     plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
#     plt.grid(True)
#     plt.xlim(axes[axis][axis][0], axes[axis][axis][-1])
#     plt.ylabel(r'Torques $\mathbf{\tau}$ (mNm)')
#     plt.xlabel(r'Time  $t$')

#     plt.savefig('plots/torques_' + str(axis) + '.png')
#     plt.show()


# plt.figure(figsize=(9, 3))
# plt.plot(axes['spiral']['z'], axes['spiral']['torques'].T)
# plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
# plt.grid(True)
# plt.xlim(axes['spiral']['z'][0], axes['spiral']['z'][-1])
# # plt.ylabel(r'Torques $\boldsymbol{\tau}$ (mNm)')
# plt.xlabel(r'Time  $t$')
# plt.tight_layout()
# plt.savefig('plots/torques_spiral_z.png')
# plt.show()


# plt.figure(figsize=(9, 3))
# plt.plot(axes['spiral']['z'], axes['spiral']['tensions'].T)
# plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
# plt.grid(True)
# plt.xlim(axes['spiral']['z'][0], axes['spiral']['z'][-1])
# # plt.ylabel(r'Tension $\mathbf{T}$ (mNm)')
# plt.xlabel(r'Time  $t$')
# plt.tight_layout()
# plt.savefig('plots/tensions_spiral_z.png')
# plt.show()


# print(np.linalg.norm(axes['x']['torques'], axis=0))
# plt.plot()

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
