import numpy as np
from kinematics import *
from time import perf_counter

end_effector_pos = np.array([0,0,70])

start = perf_counter()
contraction = inverse_kinematics(end_effector_pos)
end = perf_counter()
print(end - start)

start = perf_counter()
end_effector_forward = forward_kinematics(contraction)
end = perf_counter()
print(end - start)

start = perf_counter()
jacobian_m, jacobian_d = jacobians(contraction)
end = perf_counter()
print(end - start)

