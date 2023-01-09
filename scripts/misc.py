import numpy as np 


torques_samples = 5000
unit_vectors = np.random.randn(torques_samples, 4)
sphere = np.diag(1/np.linalg.norm(unit_vectors, axis = 1)) @ unit_vectors
sector_sphere = np.abs(sphere)
# sector_sphere = sphere
torques_pos = sector_sphere
torque_all = sphere

def generate_sphere_points(dimensions, samples = 1000, positive_sector = False):
    unit_vectors = np.random.randn(samples, dimensions)
    points = np.diag(1/np.linalg.norm(unit_vectors, axis = 1)) @ unit_vectors
    if positive_sector:
        points = np.abs(points)
    return points

def generate_rectangle_points(min_bounds, max_bounds, samples = 1000):
    n = np.shape(max_bounds)[0]
    # print(n)
    points = min_bounds + (max_bounds-min_bounds)*np.random.rand(samples, n)
    return points