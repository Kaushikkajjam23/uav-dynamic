# Real-Time Conflict-free Path Planning for Multi-UAVs Based on Improved Hybrid A* and RRT Algorithms

This repository implements real-time conflict-free path planning algorithms for multiple UAVs using improved versions of Hybrid A* and RRT (Rapidly-exploring Random Tree) algorithms. The project provides both 2D (main branch) and 3D (threeD branch) simulation environments.

## Author
- Kaushik Kajjam

## Requirements

```
# requirements.txt
numpy>=1.19.0
matplotlib>=3.3.0
flask>=2.0.0
flask-cors>=3.0.10
scipy>=1.5.0
networkx>=2.5.0
shapely>=1.7.0
scikit-learn>=0.24.0
```

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/Kaushikkajjam23/uav-dynamic.git
   cd uav-dynamic
   ```

2. Install the required packages:
   ```bash
   pip install -r requirements.txt
   ```

3. Choose the branch you want to work with:

   For 2D simulation (default):
   ```bash
   git checkout main
   ```

   For 3D simulation:
   ```bash
   git checkout threeD
   ```

## How to Run

### Step 1: Start the Backend Server
Run the backend Flask server:
```bash
python backend.py
```
This will start the server, typically on http://localhost:5000

### Step 2: Open the Web Interface
Open the `index.html` file in your web browser:
- You can directly open it by double-clicking the file
- Or use a local server for better performance:
  ```bash
  # Using Python's built-in HTTP server
  python -m http.server
  ```
  Then navigate to http://localhost:8000 and open index.html

### Step 3: Using the Interface
The web interface allows you to:
- Configure the number of UAVs
- Set start and goal positions
- Add obstacles
- Choose between Hybrid A* and RRT algorithms
- Visualize the generated paths

Click "Run Simulation" to execute the path planning algorithm and visualize the results.

## Features

- **Multi-UAV Path Planning**: Generate conflict-free paths for multiple UAVs simultaneously
- **Improved Hybrid A* Algorithm**: Enhanced version of A* for smoother path planning
- **RRT Implementation**: Rapidly-exploring Random Tree for efficient exploration of the configuration space
- **Collision Avoidance**: Algorithms ensure paths are free from collisions with obstacles and other UAVs
- **Interactive Visualization**: Web-based interface for easy configuration and result visualization
- **2D and 3D Simulations**: Support for both 2D (main branch) and 3D (threeD branch) environments

## Branch Information

- **main**: Contains the 2D implementation of the path planning algorithms
- **threeD**: Contains the 3D implementation with additional altitude considerations

## Understanding the Code (For Beginners)

- **Backend (backend.py)**: Flask server that processes path planning requests and runs the algorithms
- **Frontend (index.html)**: Web interface for configuring simulations and visualizing results
- **Algorithms**:
  - Hybrid A* combines discrete A* search with continuous motion primitives
  - RRT builds a tree of reachable states by random sampling
- **Conflict Resolution**: The system detects and resolves potential conflicts between UAV paths

## Troubleshooting

- If the backend fails to start, check if the required port (default: 5000) is available
- For visualization issues, ensure your browser supports HTML5 Canvas
- If paths aren't generating, check the console for error messages
- CORS issues may occur if opening the HTML directly; use a local server instead

## Extending the Project

- Add new path planning algorithms
- Implement dynamic obstacle avoidance
- Enhance the visualization with 3D WebGL rendering
- Add realistic UAV dynamics constraints
