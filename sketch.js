let path = [];
let obstacles = [
  { type: "circle", x: 200, y: 200, z: 0, radius: 50 },
  { type: "rectangle", x: 400, y: 300, z: 100, width: 100, height: 200 },
  { type: "rectangle", x: 600, y: 100, z: -50, width: 150, height: 150 },
];
let bounds = {
  min_x: 0,
  max_x: 800,
  min_y: 0,
  max_y: 600,
  min_z: -100,
  max_z: 100,
};
// Define dynamic obstacles
let dynamic_obstacles = [
  {
    type: "circle",
    x: 400,
    y: 500,
    z: 100,
    radius: 40,
    speedX: 1,
    speedY: 0.7,
  },
  {
    type: "circle",
    x: 600,
    y: 400,
    z: 50,
    radius: 30,
    speedX: -1,
    speedY: -0.7,
  },
];
let droneX = 100;
let droneY = 100;
let startPoint = { x: 100, y: 100 }; // Store the start point
let goalPoint = { x: 700, y: 500 }; // Store the goal point
let currentPathIndex = 0; // Index to track current point on the path
let pathPlanningInterval = 60; // replan interval
let pathPlanningCounter = 0; //Frame Counter
let collisionDetected = false; // collision check;
let displayBoxWidth = 200; // Width of the display box
let displayBoxHeight = 100; // Height of the display box
let goalReached = false; // Flag to indicate if the goal has been reached

function setup() {
  createCanvas(800, 600, WEBGL);
  perspective(PI / 3.0, width / height, 0.1, 1000);
  // Initialize Start and End Points using UI.
  updateStartGoalPoints();
  //Setting first cordinates
  droneX = startPoint.x;
  droneY = startPoint.y;
  planPath();
}
function updateStartGoalPoints() {
  startPoint = {
    x: parseInt(document.getElementById("startX").value),
    y: parseInt(document.getElementById("startY").value),
  };
  goalPoint = {
    x: parseInt(document.getElementById("goalX").value),
    y: parseInt(document.getElementById("goalY").value),
  };
}
function draw() {
  background(220);
  lights();

  moveDynamicObstacles();
  drawObstacles();
  drawPath();
  drawDrone();
  drawStartGoal();
  updateDronePosition();
  drawCoordinatesDisplay();

  // Display "Goal Reached" message , stop processing path as well;
  if (goalReached) {
    displayGoalReachedMessage();
    noLoop(); // Stop the draw loop , which make the loop STOP.
    return;
  }

  // Replan path every N frames.  Or immediately if collision , always take a replan and never get stuck!
  pathPlanningCounter++;
  if (pathPlanningCounter >= pathPlanningInterval || collisionDetected) {
    collisionDetected = false; // Reset collision flag
    updateStartGoalPoints(); //Update start/goal again for good measures, may be user updated

    // Replan from CURRENT DRONE Position so
    startPoint = {
      x: droneX, // Replanning will start with current Drone Coordinates not static start
      y: droneY,
    };
    planPath();
    pathPlanningCounter = 0;
  }
}
// Display Goal Reached Message
function displayGoalReachedMessage() {
  // Disable depth test so the box always appears on top
  push();
  translate(
    -width / 2 + displayBoxWidth / 2,
    height / 2 - displayBoxHeight / 2,
    0
  ); // Position the box near the bottom
  noStroke();
  fill(255, 255, 255, 150); // Semi-transparent white
  box(displayBoxWidth, displayBoxHeight / 2, 1); // Smaller box

  fill(0);
  textAlign(CENTER, CENTER);
  textSize(16);
  text("Goal Reached!", 0, 0);
  pop();
}
//Coordinate Box display
function drawCoordinatesDisplay() {
  // Disable depth test so the box always appears on top
  //  disableDepthTest();  //This may cause issues

  // Draw the semi-transparent white box on the left side
  push();
  translate(
    -width / 2 + displayBoxWidth / 2,
    -height / 2 + displayBoxHeight / 2,
    0
  ); // Position the box
  noStroke(); // No outline
  fill(255, 255, 255, 150); // Semi-transparent white
  box(displayBoxWidth, displayBoxHeight, 1); // Tiny box

  // Display current coordinates in the box
  fill(0); // Black text
  textAlign(CENTER, CENTER); // Center text
  textSize(16);
  text(`X: ${droneX.toFixed(2)}\nY: ${droneY.toFixed(2)}`, 0, 0);
  pop();

  // Re-enable depth test
  //  enableDepthTest();
}
function drawStartGoal() {
  // Draw Start Point (Green)
  push();
  fill(0, 255, 0); // Green
  translate(startPoint.x - width / 2, startPoint.y - height / 2, 0);
  sphere(10); // Radius 10
  pop();

  // Draw Goal Point (Red)
  push();
  fill(255, 0, 0); // Red
  translate(goalPoint.x - width / 2, goalPoint.y - height / 2, 0);
  sphere(10); // Radius 10
  pop();
}
function drawPath() {
  if (path.length > 1) {
    push();
    stroke(0, 0, 255); // Blue
    strokeWeight(3);

    beginShape(LINES);
    for (let i = 0; i < path.length - 1; i++) {
      let x1 = path[i][0] - width / 2;
      let y1 = path[i][1] - height / 2;
      let x2 = path[i + 1][0] - width / 2;
      let y2 = path[i + 1][1] - height / 2;

      vertex(x1, y1, 0);
      vertex(x2, y2, 0);
    }
    endShape();
    pop();
  }
}
function drawObstacles() {
  for (let obs of obstacles) {
    push(); // Save current transformation matrix
    let zOffset = -obs.z;
    stroke(0, 0, 0); //Outline

    if (obs.type === "circle") {
      translate(obs.x - width / 2, obs.y - height / 2, zOffset); // Center and apply Z
      sphere(obs.radius);
    } else if (obs.type === "rectangle") {
      translate(obs.x - width / 2, obs.y - height / 2, zOffset); // Center and apply Z
      box(obs.width, obs.height, 50); // Use box for 3D rectangle
    }
    pop(); // Restore previous transformation matrix
  }

  // Dynamic obstacles
  for (let obs of dynamic_obstacles) {
    push();
    translate(obs.x - width / 2, obs.y - height / 2, -obs.z);
    fill(255, 150, 0); // Orange
    sphere(obs.radius);
    pop();
  }
}

function drawDrone() {
  push();
  fill(0, 255, 0);
  translate(droneX - width / 2, droneY - height / 2, 0); // Translate to the drone's position and center
  rotateY(frameCount * 0.01); //Add Rotation!
  cone(20, 40); //Show a cone to imply movement
  pop();
}

function updateDronePosition() {
  if (path.length > 0 && currentPathIndex < path.length) {
    //Move the drone along the path
    droneX = path[currentPathIndex][0];
    droneY = path[currentPathIndex][1];
    currentPathIndex++;

    // Check if the drone is near the goal
    let d = dist(droneX, droneY, goalPoint.x, goalPoint.y);
    if (d < 20) {
      // Distance threshold
      goalReached = true;
      path = [];
      currentPathIndex = 0;
      console.log("Goal Reached!");
      return; // Stop further path updates
    }
    //To improve accuracy and consider STATIC Obstacles , should run here; , and if it DOES hit the static, we always replan to be smart;
    if (checkCollision(droneX, droneY, obstacles)) {
      console.log("Collided with static obstacle. Replan.");
      collisionDetected = true;
      startPoint = {
        x: droneX, // Replanning will start with current Drone Coordinates not static start
        y: droneY,
      };
      updateStartGoalPoints(); // To make sure what user want to do is what it is.
      planPath();
      return; // IMPORTANT: Stop current path processing and let replanning take over
    }
    // Check for collision with dynamic obstacles. If it DOES hit it , we replan, and run after static obstacle check;
    if (checkCollision(droneX, droneY, dynamic_obstacles)) {
      console.log("Collided with dynamic obstacle. Replan.");
      collisionDetected = true;
      startPoint = {
        x: droneX, // Replanning will start with current Drone Coordinates not static start
        y: droneY,
      };
      updateStartGoalPoints(); // To make sure what user want to do is what it is.
      planPath();
      return; // IMPORTANT: Stop current path processing and let replanning take over
    }
  } else {
    if (!goalReached) {
      console.log(
        "Path Ended but Goal NOT reached . Let replan from drone point"
      );
      collisionDetected = true; //Cause replan if it ever ends at odd place
    }
  }
}

function checkCollision(x, y, obstaclesToCheck) {
  for (let obs of obstaclesToCheck) {
    let distance = dist(x, y, obs.x, obs.y);
    if (distance < obs.radius + 20) {
      //20 is the drone Radius to consider.
      return true;
    }
  }
  return false;
}

function planPath() {
  let startX = document.getElementById("startX").value;
  let startY = document.getElementById("startY").value;
  let goalX = document.getElementById("goalX").value;
  let goalY = document.getElementById("goalY").value;
  // update the UI
  updateStartGoalPoints();
  let start = [parseFloat(startPoint.x), parseFloat(startPoint.y)];
  let goal = [parseFloat(goalPoint.x), parseFloat(goalPoint.y)];
  goalReached = false;

  const data = {
    start: start, //Used this variable to pass updated corodinates
    goal: goal,
    obstacles: obstacles.concat(), //  Only static obstables , dynamic is not part of this logic.
    bounds: bounds,
  };
  httpPost(
    "http://127.0.0.1:5000/plan_path",
    "json",
    data,
    function (response) {
      path = response.path;
      console.log("Path received:", path);
      currentPathIndex = 0; // Reset path index
    },
    function (error) {
      console.error("Error:", error);
    }
  );
}

//Move Dynamic Obstacles
function moveDynamicObstacles() {
  for (let obs of dynamic_obstacles) {
    obs.x += obs.speedX;
    obs.y += obs.speedY;
    // Simple collision detection with canvas boundaries
    if (obs.x + obs.radius > width || obs.x - obs.radius < 0) {
      obs.speedX *= -1;
    }
    if (obs.y + obs.radius > height || obs.y - obs.radius < 0) {
      obs.speedY *= -1;
    }
  }
}
