from flask import Flask, jsonify, request
from flask_cors import CORS
import numpy as np
import math
import random

# Import the algorithm
from uav_path_planning import RRTPlanner  # Or from algorithm if you renamed the file

app = Flask(__name__)
CORS(app)  # Enable Cross-Origin Resource Sharing

# --- API Endpoint ---
@app.route('/plan_path', methods=['POST'])
def plan_path():
    data = request.get_json()
    start = data['start']
    goal = data['goal']
    obstacles = data['obstacles']
    bounds = data['bounds']

    planner = RRTPlanner()
    path = planner.find_path(start, goal, obstacles, bounds)

    if path:
        return jsonify({'path': path})
    else:
        return jsonify({'path': []})  # Return an empty list instead of None

if __name__ == '__main__':
    app.run(debug=False)