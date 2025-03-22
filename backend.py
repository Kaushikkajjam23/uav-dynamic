# from flask import Flask, jsonify, request
# from flask_cors import CORS
# import numpy as np
# import math
# import random
# from queue import PriorityQueue  # Import PriorityQueue

# # Import the algorithm
# #from uav_path_planning import RRTPlanner # Or from algorithm if you renamed the file
# #from uav_path_planning import HybridAStar

# app = Flask(__name__)
# CORS(app)  # Enable Cross-Origin Resource Sharing

# class Node:
#     def __init__(self, x, y, parent=None):
#         self.x = x
#         self.y = y
#         self.parent = parent
#         self.cost = 0

#     def __lt__(self, other):
#         return self.cost < other.cost

#     def __eq__(self, other):
#         return abs(self.x - other.x) < 0.1 and abs(self.y - other.y) < 0.1

# class RRTPlanner:
#     def __init__(self):
#         self.step_size = 0.001 #Reduced Stepsize as Lat/Long is smaller
#         self.goal_sample_rate = 0.1
#         self.max_iterations = 5000

#     def sample_point(self, bounds, goal, obstacles):
#         if random.random() < self.goal_sample_rate:
#             return goal.x, goal.y
            
#         while True:
#             x = random.uniform(bounds['min_lat'], bounds['max_lat'])
#             y = random.uniform(bounds['min_lng'], bounds['max_lng'])
#             if not self.check_collision(x, y, obstacles):
#                 return x, y

#     def find_nearest(self, nodes, point):
#         distances = [(node, math.sqrt((node.x - point[0])**2 + (node.y - point[1])**2)) 
#                     for node in nodes]
#         return min(distances, key=lambda x: x[1])[0]

#     def steer(self, from_node, to_point):
#         dx = to_point[0] - from_node.x
#         dy = to_point[1] - from_node.y
#         distance = math.sqrt(dx**2 + dy**2)
        
#         if distance < self.step_size:
#             new_x = to_point[0]
#             new_y = to_point[1]
#         else:
#             theta = math.atan2(dy, dx)
#             new_x = from_node.x + self.step_size * math.cos(theta)
#             new_y = from_node.y + self.step_size * math.sin(theta)
            
#         return Node(new_x, new_y, parent=from_node)

#     def check_collision(self, x, y, obstacles):
#         for obs in obstacles:
#             if obs['type'] == 'circle':
#                 if math.sqrt((x - obs['lat'])**2 + (y - obs['lng'])**2) < obs['radius']:
#                     return True
#             elif obs['type'] == 'rectangle':
#                 if (x > obs['lat'] and x < obs['lat'] + obs['width'] and 
#                     y > obs['lng'] and y < obs['lng'] + obs['height']):
#                     return True
#         return False

#     def find_path(self, start, goal, obstacles, bounds):
#         start_node = Node(start[0], start[1])
#         goal_node = Node(goal[0], goal[1])
#         nodes = [start_node]
        
#         for _ in range(self.max_iterations):
#             # Sample random point
#             rnd_point = self.sample_point(bounds, goal_node, obstacles)
            
#             # Find nearest node
#             nearest_node = self.find_nearest(nodes, rnd_point)
            
#             # Create new node
#             new_node = self.steer(nearest_node, rnd_point)
            
#             # Check if new node is collision free
#             if not self.check_collision(new_node.x, new_node.y, obstacles):
#                 nodes.append(new_node)
                
#                 # Check if we can connect to goal
#                 if math.sqrt((new_node.x - goal_node.x)**2 + (new_node.y - goal_node.y)**2) < self.step_size:
#                     goal_node.parent = new_node
#                     return self.reconstruct_path(goal_node)
        
#         return None

#     def reconstruct_path(self, node):
#         path = []
#         current = node
#         while current is not None:
#             path.append((current.x, current.y))
#             current = current.parent
#         return path[::-1]

# class HybridAStar:
#     def __init__(self):
#         # Motion parameters
#         self.step_size = 0.001 #Reduce Stepsize
#         self.turning_radius = 5.0
#         self.theta_steps = 8  # Number of steering angles to consider

#     def get_neighbors(self, node, obstacles):
#         neighbors = []
#         # Generate steering angles
#         theta_range = np.linspace(-math.pi/4, math.pi/4, self.theta_steps)
        
#         for theta in theta_range:
#             new_theta = node.theta + theta
#             # Normalize angle
#             new_theta = (new_theta + math.pi) % (2 * math.pi) - math.pi
            
#             # Calculate new position
#             new_x = node.x + self.step_size * math.cos(new_theta)
#             new_y = node.y + self.step_size * math.sin(new_theta)
            
#             # Check if new position is collision-free
#             if not self.check_collision(new_x, new_y, obstacles):
#                 new_node = Node(new_x, new_y, new_theta, node, node.cost + self.step_size)
#                 neighbors.append(new_node)
                
#         return neighbors

#     def heuristic(self, node, goal):
#         return math.sqrt((node.x - goal.x)**2 + (node.y - goal.y)**2)

#     def check_collision(self, x, y, obstacles):
#         for obs in obstacles:
#             if obs['type'] == 'circle':
#                 if math.sqrt((x - obs['lat'])**2 + (y - obs['lng'])**2) < obs['radius']:
#                     return True
#             elif obs['type'] == 'rectangle':
#                 if (x > obs['lat'] and x < obs['lat'] + obs['width'] and 
#                     y > obs['lng'] and y < obs['lng'] + obs['height']):
#                     return True
#         return False

#     def find_path(self, start, goal, obstacles, max_iterations=5000):
#         start_node = Node(start[0], start[1], 0)
#         goal_node = Node(goal[0], goal[1], 0)
        
#         open_set = PriorityQueue()
#         open_set.put((0, start_node))
#         closed_set = set()
        
#         iteration = 0
#         while not open_set.empty() and iteration < max_iterations:
#             current_cost, current_node = open_set.get()
            
#             if self.heuristic(current_node, goal_node) < self.step_size:
#                 return self.reconstruct_path(current_node)
            
#             # Add to closed set using x,y coordinates
#             closed_set.add((round(current_node.x, 5), round(current_node.y, 5))) #Increasing precison
            
#             # Generate neighbors
#             for neighbor in self.get_neighbors(current_node, obstacles):
#                 if (round(neighbor.x, 5), round(neighbor.y, 5)) in closed_set:
#                     continue
                    
#                 priority = neighbor.cost + self.heuristic(neighbor, goal_node)
#                 open_set.put((priority, neighbor))
            
#             iteration += 1
        
#         return None  # No path found

#     def reconstruct_path(self, node):
#         path = []
#         current = node
#         while current is not None:
#             path.append((current.x, current.y))
#             current = current.parent
#         return path[::-1]

# class CombinedPathPlanner:
#     def __init__(self):
#         self.hybrid_astar = HybridAStar()
#         self.rrt = RRTPlanner()

#     def plan_path(self, start, goal, obstacles, bounds):
#         #print("Attempting Hybrid A* path planning...")
#         #path = self.hybrid_astar.find_path(start, goal, obstacles)
        
#         #if path is None:
#         #    print("Hybrid A* failed. Switching to RRT...")
#         path = self.rrt.find_path((start[0], start[1]), (goal[0], goal[1]), obstacles, bounds)
        
#         return path

# from flask import Flask, jsonify, request
# from flask_cors import CORS
# import numpy as np
# import math
# import random
# from queue import PriorityQueue

# # Import the algorithm
# #from uav_path_planning import RRTPlanner # Or from algorithm if you renamed the file
# #from uav_path_planning import HybridAStar

# app = Flask(__name__)
# CORS(app)  # Enable Cross-Origin Resource Sharing

# # --- API Endpoint ---
# @app.route('/plan_path', methods=['POST'])
# def plan_path():
#     data = request.get_json()
#     start = data['start']
#     goal = data['goal']
#     obstacles = data['obstacles']
#     bounds = data['bounds']

#     planner = CombinedPathPlanner() #RRTPlanner()  #Or HybridAStar
#     path = planner.plan_path(start, goal, obstacles, bounds)

#     if path:
#         return jsonify({'path': path})
#     else:
#         return jsonify({'path': []})  # Return an empty list instead of None

# if __name__ == '__main__':
#     app.run(host='0.0.0.0', port=5000, debug=False)

# #     version2
# # from flask import Flask, jsonify, request
# # from flask_cors import CORS
# # import numpy as np
# # import math
# # import random

# # # Import the algorithm
# # from uav_path_planning import RRTPlanner  # Or from algorithm if you renamed the file

# # app = Flask(__name__)
# # CORS(app)  # Enable Cross-Origin Resource Sharing

# # # --- API Endpoint ---
# # @app.route('/plan_path', methods=['POST'])
# # def plan_path():
# #     data = request.get_json()
# #     start = data['start']
# #     goal = data['goal']
# #     obstacles = data['obstacles']
# #     bounds = data['bounds']

# #     planner = RRTPlanner()
# #     path = planner.find_path(start, goal, obstacles, bounds)

# #     if path:
# #         return jsonify({'path': path})
# #     else:
# #         return jsonify({'path': []})  # Return an empty list instead of None

# # if __name__ == '__main__':
# #     app.run(debug=False)



# #     version3
# # from flask import Flask, jsonify, request
# # from flask_cors import CORS
# # import numpy as np
# # import math
# # import random

# # # Import the algorithm
# # from uav_path_planning import RRTPlanner  # Or from algorithm if you renamed the file

# # app = Flask(__name__)
# # CORS(app)  # Enable Cross-Origin Resource Sharing

# # # --- API Endpoint ---
# # @app.route('/plan_path', methods=['POST'])
# # def plan_path():
# #     data = request.get_json()
# #     start = data['start']
# #     goal = data['goal']
# #     obstacles = data['obstacles']
# #     bounds = data['bounds']

# #     planner = RRTPlanner()
# #     path = planner.find_path(start, goal, obstacles, bounds)

# #     if path:
# #         return jsonify({'path': path})
# #     else:
# #         return jsonify({'path': []})  # Return an empty list instead of None

# # if __name__ == '__main__':
# #     app.run(debug=False)

###version 4
from flask import Flask, jsonify, request
from flask_cors import CORS
import numpy as np
import math
import random
from queue import PriorityQueue

app = Flask(__name__)
CORS(app)  # Enable Cross-Origin Resource Sharing

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

            # Check if new position is collision-free
            if not self.check_collision(new_x, new_y, obstacles):
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

    def find_path(self, start, goal, obstacles, max_iterations=5000):
        start_node = Node(start[0], start[1], 0)
        goal_node = Node(goal[0], goal[1], 0)

        open_set = PriorityQueue()
        open_set.put((0, start_node))
        closed_set = set()

        iteration = 0
        while not open_set.empty() and iteration < max_iterations:
            current_cost, current_node = open_set.get()

            if self.heuristic(current_node, goal_node) < self.step_size:
                return self.reconstruct_path(current_node)

            # Add to closed set using x,y coordinates
            closed_set.add((round(current_node.x, 5), round(current_node.y, 5))) #Increasing precison

            # Generate neighbors
            for neighbor in self.get_neighbors(current_node, obstacles):
                if (round(neighbor.x, 5), round(neighbor.y, 5)) in closed_set:
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

@app.route('/plan_path', methods=['POST'])
def plan_path():
    data = request.get_json()
    start = data['start']
    goal = data['goal']
    obstacles = data['obstacles']
    bounds = data['bounds']

    planner = CombinedPathPlanner()
    path = planner.plan_path(start, goal, obstacles, bounds)

    if path:
        return jsonify({'path': path})
    else:
        return jsonify({'path': []})  # Return an empty list instead of None

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)