import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import math
from queue import PriorityQueue
import random
import time

class Node:
    def __init__(self, x, y, theta=0, parent=None, cost=0):
        self.x = x
        self.y = y
        self.theta = theta  # Only used for Hybrid A*
        self.parent = parent
        self.cost = cost

    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        return abs(self.x - other.x) < 0.1 and abs(self.y - other.y) < 0.1

class HybridAStar:
    def __init__(self):
        # Motion parameters
        self.step_size = 3.0
        self.turning_radius = 5.0
        self.theta_steps = 8  # Number of steering angles to consider

    def get_neighbors(self, node, obstacles):
        neighbors = []
        # Generate steering angles
        theta_range = np.linspace(-math.pi/4, math.pi/4, self.theta_steps)

        for theta in theta_range:
            new_theta = node.theta + theta
            # Normalize angle
            new_theta = (new_theta + math.pi) % (2 * math.pi) - math.pi

            # Calculate new position
            new_x = node.x + self.step_size * math.cos(new_theta)
            new_y = node.y + self.step_size * math.sin(new_theta)

            # Check if new position is collision-free.  CRITICAL: check the *path*
            if not self.check_path_collision(node.x, node.y, new_x, new_y, obstacles):
                new_node = Node(new_x, new_y, new_theta, node, node.cost + self.step_size)
                neighbors.append(new_node)

        return neighbors

    def heuristic(self, node, goal):
        return math.sqrt((node.x - goal.x)**2 + (node.y - goal.y)**2)

    def check_collision(self, x, y, obstacles):
        for obs in obstacles:
            if obs['type'] == 'circle':
                if math.sqrt((x - obs['x'])**2 + (y - obs['y'])**2) < obs['radius']:
                    return True
            elif obs['type'] == 'rectangle':
                if (x > obs['x'] and x < obs['x'] + obs['width'] and
                    y > obs['y'] and y < obs['y'] + obs['height']):
                    return True
        return False

    def check_path_collision(self, x1, y1, x2, y2, obstacles, num_segments=5):
        # Checks for collision along the *path* between two points, not just at the end.
        # Discretize the path into segments and check each segment.
        for i in range(num_segments + 1):
            x = x1 + (x2 - x1) * i / num_segments
            y = y1 + (y2 - y1) * i / num_segments
            if self.check_collision(x, y, obstacles):
                return True
        return False

    def find_path(self, start, goal, obstacles, max_iterations=5000):
        start_node = Node(start[0], start[1], start[2])
        goal_node = Node(goal[0], goal[1], goal[2])

        open_set = PriorityQueue()
        open_set.put((0, start_node))
        closed_set = set()

        iteration = 0
        while not open_set.empty() and iteration < max_iterations:
            current_cost, current_node = open_set.get()

            if self.heuristic(current_node, goal_node) < self.step_size:
                return self.reconstruct_path(current_node)

            # Add to closed set using x,y,theta coordinates (more precise)
            closed_set.add((round(current_node.x, 2), round(current_node.y, 2), round(current_node.theta, 2)))

            # Generate neighbors
            for neighbor in self.get_neighbors(current_node, obstacles):
                neighbor_key = (round(neighbor.x, 2), round(neighbor.y, 2), round(neighbor.theta, 2))
                if neighbor_key in closed_set:
                    continue

                priority = neighbor.cost + self.heuristic(neighbor, goal_node)
                open_set.put((priority, neighbor))

            iteration += 1

        return None  # No path found

    def reconstruct_path(self, node):
        path = []
        current = node
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        return path[::-1]

class RRTPlanner:
    def __init__(self):
        self.step_size = 3.0
        self.goal_sample_rate = 0.1
        self.max_iterations = 5000

    def sample_point(self, bounds, goal, obstacles):
        if random.random() < self.goal_sample_rate:
            return goal.x, goal.y

        while True:
            x = random.uniform(bounds['min_x'], bounds['max_x'])
            y = random.uniform(bounds['min_y'], bounds['max_y'])
            if not self.check_collision(x, y, obstacles):
                return x, y

    def find_nearest(self, nodes, point):
        distances = [(node, math.sqrt((node.x - point[0])**2 + (node.y - point[1])**2))
                    for node in nodes]
        return min(distances, key=lambda x: x[1])[0]

    def steer(self, from_node, to_point):
        dx = to_point[0] - from_node.x
        dy = to_point[1] - from_node.y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < self.step_size:
            new_x = to_point[0]
            new_y = to_point[1]
        else:
            theta = math.atan2(dy, dx)
            new_x = from_node.x + self.step_size * math.cos(theta)
            new_y = from_node.y + self.step_size * math.sin(theta)

        return Node(new_x, new_y, parent=from_node)

    def check_collision(self, x, y, obstacles):
        for obs in obstacles:
            if obs['type'] == 'circle':
                if math.sqrt((x - obs['x'])**2 + (y - obs['y'])**2) < obs['radius']:
                    return True
            elif obs['type'] == 'rectangle':
                if (x > obs['x'] and x < obs['x'] + obs['width'] and
                    y > obs['y'] and y < obs['y'] + obs['height']):
                    return True
        return False

    def find_path(self, start, goal, obstacles, bounds):
        start_node = Node(start[0], start[1])
        goal_node = Node(goal[0], goal[1])
        nodes = [start_node]

        for _ in range(self.max_iterations):
            # Sample random point
            rnd_point = self.sample_point(bounds, goal_node, obstacles)

            # Find nearest node
            nearest_node = self.find_nearest(nodes, rnd_point)

            # Create new node
            new_node = self.steer(nearest_node, rnd_point)

            # Check if new node is collision free
            if not self.check_collision(new_node.x, new_node.y, obstacles):
                nodes.append(new_node)

                # Check if we can connect to goal
                if math.sqrt((new_node.x - goal_node.x)**2 + (new_node.y - goal_node.y)**2) < self.step_size:
                    goal_node.parent = new_node
                    return self.reconstruct_path(goal_node)

        return None

    def reconstruct_path(self, node):
        path = []
        current = node
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        return path[::-1]

class CombinedPathPlanner:
    def __init__(self):
        self.hybrid_astar = HybridAStar()
        self.rrt = RRTPlanner()

    def plan_path(self, start, goal, obstacles, bounds):
        print("Attempting Hybrid A* path planning...")
        path = self.hybrid_astar.find_path(start, goal, obstacles)

        if path is None:
            print("Hybrid A* failed. Switching to RRT...")
            path = self.rrt.find_path((start[0], start[1]), (goal[0], goal[1]), obstacles, bounds)

        return path

# NEW: Function to simulate dynamic obstacle movement
def move_dynamic_obstacles(dynamic_obstacles, bounds):
    for obs in dynamic_obstacles:
        # Simple linear movement (can be made more complex)
        obs['x'] += obs['speed_x']
        obs['y'] += obs['speed_y']

        # Bounce off the walls
        if obs['x'] - obs['radius'] < bounds['min_x'] or obs['x'] + obs['radius'] > bounds['max_x']:
            obs['speed_x'] *= -1
        if obs['y'] - obs['radius'] < bounds['min_y'] or obs['y'] + obs['radius'] > bounds['max_y']:
            obs['speed_y'] *= -1

# Modified visualize_path to handle dynamic obstacles and animation
def visualize_path(path, obstacles, dynamic_obstacles, start, goal, bounds):
    fig, ax = plt.subplots(figsize=(10, 10))  # Create figure and axes objects

    def init():
        # Initialize static elements (obstacles, start, goal)
        ax.clear()  # Clear everything
        for obs in obstacles:
            if obs['type'] == 'circle':
                circle = Circle((obs['x'], obs['y']), obs['radius'], fill=True, color='red', alpha=0.5)
                ax.add_patch(circle)
            elif obs['type'] == 'rectangle':
                rect = Rectangle((obs['x'], obs['y']), obs['width'], obs['height'],
                               fill=True, color='red', alpha=0.5)
                ax.add_patch(rect)

        # Plot start and goal (only once)
        ax.plot(start[0], start[1], 'go', label='Start', markersize=15)
        ax.plot(goal[0], goal[1], 'ro', label='Goal', markersize=15)

        # Set bounds and other plot properties (only once)
        ax.set_xlim(bounds['min_x'] - 5, bounds['max_x'] + 5)
        ax.set_ylim(bounds['min_y'] - 5, bounds['max_y'] + 5)
        ax.grid(True)
        ax.legend()
        ax.set_aspect('equal')
        ax.set_title('UAV Path Planning')
        return ax.patches + ax.lines  # Return artists to be animated

    def animate(i):
        # Update dynamic elements (dynamic obstacles and path)
        move_dynamic_obstacles(dynamic_obstacles, bounds)

        # Clear previous dynamic obstacles
        for patch in ax.patches:
            if hasattr(patch, 'is_dynamic') and patch.is_dynamic:
                patch.remove()

        # Plot dynamic obstacles
        dynamic_patches = []
        for obs in dynamic_obstacles:
            circle = Circle((obs['x'], obs['y']), obs['radius'], fill=True, color='orange', alpha=0.7)
            circle.is_dynamic = True  # Mark as dynamic for later removal
            ax.add_patch(circle)
            dynamic_patches.append(circle)


        # Plot path
        path_line, = ax.plot([], [], 'b-', label='Path')  # Initialize path line

        if path:
            path = np.array(path)
            path_line.set_data(path[:, 0], path[:, 1])
        else:
            path_line.set_data([], [])  # Clear the path
        ax.legend()
        return [path_line] + dynamic_patches

    import matplotlib.animation as animation
    ani = animation.FuncAnimation(fig, animate, init_func=init, frames=200, blit=False, repeat=True)
    plt.show()

def main():
    # Define environment bounds
    bounds = {
        'min_x': 0,
        'max_x': 100,
        'min_y': 0,
        'max_y': 100
    }

    # Define static obstacles (mix of circles and rectangles)
    static_obstacles = [
        {'type': 'circle', 'x': 20, 'y': 20, 'radius': 5},
        {'type': 'rectangle', 'x': 60, 'y': 60, 'width': 10, 'height': 20},
        {'type': 'rectangle', 'x': 30, 'y': 70, 'width': 15, 'height': 15}
    ]

    # NEW: Define dynamic obstacles
    dynamic_obstacles = [
        {'type': 'circle', 'x': 40, 'y': 50, 'radius': 8, 'speed_x': 0.5, 'speed_y': 0.3}, # Moving Circle
        {'type': 'circle', 'x': 70, 'y': 30, 'radius': 6, 'speed_x': -0.3, 'speed_y': 0.7}  # Moving Circle
    ]

    # Combine static and dynamic obstacles for planning
    obstacles = static_obstacles + dynamic_obstacles

    # Define start and goal positions (x, y, theta)
    start = (10, 10, 0)
    goal = (90, 90, 0)

    # Create planner and find path
    planner = CombinedPathPlanner()
    path = planner.plan_path(start, goal, obstacles, bounds)

    if path:
        print("Path found successfully!")
        # Visualize the result
        visualize_path(path, static_obstacles, dynamic_obstacles, start, goal, bounds) # Pass dynamic_obstacles
    else:
        print("No path found!")

if __name__ == "__main__":
    main()