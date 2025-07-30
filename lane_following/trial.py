import numpy as np
import matplotlib.pyplot as plt

def calculate_point_projection(x_0, y_0, x_1, y_1, x_c, y_c, d):
    """
    Calculate the projection of a point onto a line with a robust method.
    
    Parameters:
    x_0, y_0: Starting point of the line
    x_1, y_1: Ending point of the line
    x_c, y_c: Current point to project
    d: Distance to move along the line
    
    Returns:
    Projected point coordinates
    """
    current_position = np.array([x_c, y_c])
    v = np.array([x_1 - x_0, y_1 - y_0])
    v_hat = v / np.linalg.norm(v)
    print(v_hat)
    m = v[1]/(v[0] + 0.00001)
    print(m)

    if m > 100:
        x_i = x_0
        y_i = current_position[1] 

    else:
        c = y_0 - m*x_0
        print(c)
        x_i = (m*current_position[1] - m*c + current_position[0])/(m**2 + 1)
        y_i = m*x_i + c
    
    closest_point = np.array([x_i, y_i])
    x_d, y_d = np.sum([closest_point, d*v_hat], axis=0)
    print(x_d, y_d) 

    return x_d, y_d, x_i, y_i

# Example usage with visualization
x_0, y_0 = 1.0, 1.0
x_1, y_1 = 6.0, 9.0
x_c, y_c = 5.0, 2.0
d = 0.8

# Calculate projection
result_x, result_y, closest_x, closest_y = calculate_point_projection(x_0, y_0, x_1, y_1, x_c, y_c, d)

# Plotting
plt.figure(figsize=(10, 8))

# Plot the original line
plt.plot([x_0, x_1], [y_0, y_1], 'b-', label='Original Line')

# Plot the current point
plt.plot(x_c, y_c, 'ro', label='Current Point')

# Plot the closest point on the line
plt.plot(closest_x, closest_y, 'go', label='Closest Point')

# Plot the final projected point
plt.plot(result_x, result_y, 'mo', label='Projected Point')

# Plot lines connecting points for clarity
plt.plot([x_c, closest_x], [y_c, closest_y], 'r--', label='Perpendicular Line')
plt.plot([closest_x, result_x], [closest_y, result_y], 'g--', label='Projection Move')

plt.title('Point Projection Visualization')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.grid(True)
plt.legend()
plt.axis('equal')

# Print the coordinates
print(f"Original Line: ({x_0}, {y_0}) to ({x_1}, {y_1})")
print(f"Current Point: ({x_c}, {y_c})")
print(f"Closest Point on Line: ({closest_x}, {closest_y})")
print(f"Projected Point: ({result_x}, {result_y})")
plt.xlim(0, 11)
plt.ylim(0, 11)
plt.show()