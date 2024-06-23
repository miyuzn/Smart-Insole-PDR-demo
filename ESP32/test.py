import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata
from scipy.spatial import ConvexHull
from matplotlib.patches import Polygon

# Coordinates of pressure sensors on a shoe insole
coordinate_x_35_insole = [
    -40.6, -21.2, -6.5, 7.2, 17.3,
    -39.6, -24.3, -8.2, 4.3, 15.2,
    -35.3, -23, -8.9, 4.5, 16.2,
    -17, -8.5, 0, 8.5, 17,
    -19.5, -11, -2.5, 6, 14.5,
    -29, -19, -9, 1, 11,
    -30, -20.5, -11, -1.5, 8
]
coordinate_y_35_insole = [
    100.5, 104, 100.7, 88, 73,
    70.8, 68.9, 65, 59.8, 54,
    39, 36, 32, 28.5, 25.2,
    0, 0, 0, 0, 0,
    -40, -40, -40, -40, -40,
    -70, -70, -70, -70, -70,
    -90, -90, -90, -90, -90,
]

# Sample pressure values at each sensor location (randomly generated for demonstration)
pressure_values = np.random.rand(35) * 100  # Random pressures between 0 and 100

# Creating the foot-shaped polygon using a convex hull
points = np.array(list(zip(coordinate_x_35_insole, coordinate_y_35_insole)))
hull = ConvexHull(points)
foot_polygon = Polygon(points[hull.vertices], closed=True)

# Create a grid for interpolation
grid_x, grid_y = np.mgrid[-50:30:500j, -110:100:500j]

# Interpolate the data
grid_z = griddata(
    points,
    pressure_values,
    (grid_x, grid_y),
    method='cubic'
)

# Mask out areas outside the foot polygon by setting them to NaN
for ix in range(grid_z.shape[0]):
    for iy in range(grid_z.shape[1]):
        if not foot_polygon.contains_point((grid_x[ix, 0], grid_y[0, iy])):
            grid_z[ix, iy] = np.nan

# Plotting
plt.figure(figsize=(10, 8))
plt.imshow(grid_z.T, extent=(-50, 30, -110, 100), origin='lower', cmap='hot', alpha=0.7)
plt.colorbar(label='Pressure (arbitrary units)')
plt.plot(points[hull.vertices, 0], points[hull.vertices, 1], 'r-', lw=2)  # Hull outline
plt.scatter(coordinate_x_35_insole, coordinate_y_35_insole, color='blue')  # Sensor locations
plt.title('Pressure Heatmap of a Foot Insole Shaped Like a Foot')
plt.xlabel('X coordinate (mm)')
plt.ylabel('Y coordinate (mm)')
plt.grid(False)
plt.show()
