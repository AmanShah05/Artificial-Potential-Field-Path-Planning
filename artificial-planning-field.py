import numpy as np
import matplotlib.pyplot as plt
import random

def add_goal(X, Y, s, r, loc):
    delx = np.zeros_like(X)
    dely = np.zeros_like(Y)
    for i in range(len(X)):
        for j in range(len(Y)):
            d = np.sqrt((loc[0] - X[i][j])**2 + (loc[1] - Y[i][j])**2)
            theta = np.arctan2(loc[1] - Y[i][j], loc[0] - X[i][j])
            if d < r:
                delx[i][j] = 0
                dely[i][j] = 0
            elif d > r + s:
                delx[i][j] = 50 * s * np.cos(theta)
                dely[i][j] = 50 * s * np.sin(theta)
            else:
                delx[i][j] = 50 * (d - r) * np.cos(theta)
                dely[i][j] = 50 * (d - r) * np.sin(theta)
    return delx, dely


def add_obstacle(X, Y, s, obstacle, r):
    delx = np.zeros_like(X)
    dely = np.zeros_like(Y)
    for i in range(len(X)):
        for j in range(len(Y)):
            d_obstacle = np.sqrt((obstacle[0]-X[i][j])**2 + (obstacle[1]-Y[i][j])**2)
            theta_obstacle = np.arctan2(obstacle[1]-Y[i][j], obstacle[0]-X[i][j])
            if d_obstacle < r:
                delx[i][j] = -50 * np.cos(theta_obstacle)
                dely[i][j] = -50 * np.sin(theta_obstacle)
            elif d_obstacle > r + s:
                # No influence if the distance is greater than r + s
                continue
            else:
                delx[i][j] = -50 * (s + r - d_obstacle) * np.cos(theta_obstacle)
                dely[i][j] = -50 * (s + r - d_obstacle) * np.sin(theta_obstacle)
    return delx, dely


def plot_graph(X, Y, delx, dely,obj, fig, ax, loc,r,i, color,start_goal=np.array([[0,0]])  ):
  
  ax.quiver(X, Y, delx, dely)
  ax.add_patch(plt.Circle(loc, r, color=color))
  ax.set_title(f'Robot path with {i} obstacles ')
  ax.annotate(obj, xy=loc, fontsize=10, ha="center")
  return ax

def combine_fields(delx_goal, dely_goal, delx_obstacles, dely_obstacles, goal, r):
    delx = delx_goal + delx_obstacles
    dely = dely_goal + dely_obstacles
    for i in range(len(delx)):
        for j in range(len(dely)):
            d_goal = np.sqrt((goal[0] - X[i][j])**2 + (goal[1] - Y[i][j])**2)
            if d_goal < r:
                delx[i][j] = 0
                dely[i][j] = 0
    return delx, dely
# Define the functions add_goal, add_obstacle, and plot_graph here as in your provided code.
# User inputs
grid_size = int(input("Enter the size of the grid (e.g., 50 for a 50x50 grid): "))
s = float(input("Enter the scalar value 's' that defines the decay of potential field with distance: "))
goal_radius = float(input("Enter the radius of influence for the goal: "))
obstacle_radius = float(input(f"Enter the radius of influence for obstacle: "))
goal_x = int(input("Enter the goal's x-coordinate: "))
goal_y = int(input("Enter the goal's y-coordinate: "))
goal = [goal_x, goal_y]
create_obstacles = input("Do you want to create obstacles at a given location (g) or at random locations (r)? ")
num_obstacles = int(input("How many obstacles do you want to add? "))

# Setup the grid
x = np.arange(0, grid_size, 1)
y = np.arange(0, grid_size, 1)
X, Y = np.meshgrid(x, y)
seek_points = np.array([[0,0]])  # Assume starting point is (0,0)
# Generate all obstacle locations
obstacle_locs = []
for j in range(num_obstacles):
    if create_obstacles.lower() == 'g':
        obs_x = int(input(f"Enter x-coordinate for obstacle {j+1}: "))
        obs_y = int(input(f"Enter y-coordinate for obstacle {j+1}: "))
        obstacle_locs.append([obs_x, obs_y])
    else:
        obstacle_locs.append(random.sample(range(0, grid_size), 2))

# Only one plot for all obstacles
fig, ax = plt.subplots(figsize=(10, 10))
# Add the goal
delx_goal, dely_goal = add_goal(X, Y, s, goal_radius, goal)
plot_graph(X, Y, delx_goal, dely_goal, 'Goal', fig, ax, goal, goal_radius, 0, 'b')


# Initialize obstacle fields
delx_obstacles_total = np.zeros_like(X)
dely_obstacles_total = np.zeros_like(Y)

# Compute the obstacle fields and combine them
for obstacle_loc in obstacle_locs:
    delx_obstacle, dely_obstacle = add_obstacle(X, Y, s, obstacle_loc, obstacle_radius)
    delx_obstacles_total += delx_obstacle
    dely_obstacles_total += dely_obstacle

# Combine the goal and obstacle fields
delx_combined, dely_combined = combine_fields(delx_goal, dely_goal, delx_obstacles_total, dely_obstacles_total, goal, goal_radius)

# Streamplot to show the path
plot_graph(X, Y, delx_combined, dely_combined, 'Obstacle', fig, ax, obstacle_loc, obstacle_radius, num_obstacles, 'r')
ax.streamplot(X,Y,delx_combined,dely_combined, start_points=seek_points,linewidth=4, cmap='autu')

# Show the obstacles on the plot
for obstacle_loc in obstacle_locs:
    ax.add_patch(plt.Circle(obstacle_loc, obstacle_radius, color='red'))
    ax.annotate('Obstacle', xy=obstacle_loc, fontsize=10, ha="center", va="center")

# Enhance plot with labels and legends
ax.set_xlabel('X coordinate')
ax.set_ylabel('Y coordinate')
ax.set_title('Robot Path Planning')

plt.show()