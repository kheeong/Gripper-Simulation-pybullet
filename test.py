import numpy as np

# Sphere parameters
r = 0.6  # Radius
center = np.array([0.0, 0.0, 0.0])  # Center of the sphere

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

n_samples = 1000

# Generate random values
phi = np.random.uniform(0, 2 * np.pi, n_samples)
cos_theta = np.random.uniform(-1, 1, n_samples)
theta = np.arccos(cos_theta)

# Compute Cartesian coordinates
x = r * np.sin(theta) * np.cos(phi)
y = r * np.sin(theta) * np.sin(phi)
z = r * cos_theta

x += center[0]
y += center[1]
z += center[2]

# Combine into a list of coordinates
points = np.vstack((x, y, z)).T
print(points[0])
# Visualize the points
#fig = plt.figure(figsize=(8, 6))
#ax = fig.add_subplot(111, projection='3d')

#ax.scatter(x, y, z, color='r', s=1)
#ax.set_box_aspect([1,1,1])
#ax.set_xlabel('X')
#ax.set_ylabel('Y')
#ax.set_zlabel('Z')
#plt.title('Random Points on Sphere Surface')
#plt.show()

distances = np.sqrt(x**2 + y**2 + z**2)

# Check if distances are approximately equal to the radius
tolerance = 1e-6
on_surface = np.all(np.abs(distances - r) < tolerance)

print(f"All points lie on the sphere's surface: {on_surface}")