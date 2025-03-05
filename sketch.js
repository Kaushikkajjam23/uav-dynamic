let path = [];
let obstacles = [
  { type: "circle", x: 200, y: 200, radius: 50 },
  { type: "rectangle", x: 400, y: 300, width: 100, height: 200 },
  { type: "rectangle", x: 600, y: 100, width: 150, height: 150 },
];
let bounds = {
  min_x: 0,
  max_x: 800,
  min_y: 0,
  max_y: 600,
};
// Define dynamic obstacles
let dynamic_obstacles = [
  {
    type: "circle",
    x: 400,
    y: 500,
    radius: 40,
    speedX: 1,
    speedY: 0.7,
  },
  {
    type: "circle",
    x: 600,
    y: 400,
    radius: 30,
    speedX: -1,
    speedY: -0.7,
  },
];

function setup() {
  createCanvas(800, 600);
}

function draw() {
  background(220);
  //move Obstacles around , ensure it does not collide
  moveDynamicObstacles();

  // Draw obstacles
  for (let obs of obstacles) {
    if (obs.type === "circle") {
      ellipse(obs.x, obs.y, 2 * obs.radius);
    } else if (obs.type === "rectangle") {
      rect(obs.x, obs.y, obs.width, obs.height);
    }
  }
  //Draw dynamic obstacle
  for (let obs of dynamic_obstacles) {
    ellipse(obs.x, obs.y, 2 * obs.radius);
  }

  // Draw path
  beginShape();
  noFill();
  stroke(0, 0, 255); // Blue color
  strokeWeight(2);
  for (let point of path) {
    vertex(point[0], point[1]);
  }
  endShape();

  // Draw start and goal (replace with UI elements)
  fill(0, 255, 0); // Green color
  ellipse(
    document.getElementById("startX").value,
    document.getElementById("startY").value,
    20
  ); //start x and y
  fill(255, 0, 0); // Red color
  ellipse(
    document.getElementById("goalX").value,
    document.getElementById("goalY").value,
    20
  ); //end xy
  //Dynamic Replanning
  if (frameCount % 30 == 0) {
    planPath();
  }
}

function planPath() {
  let startX = document.getElementById("startX").value;
  let startY = document.getElementById("startY").value;
  let goalX = document.getElementById("goalX").value;
  let goalY = document.getElementById("goalY").value;

  let start = [parseFloat(startX), parseFloat(startY)];
  let goal = [parseFloat(goalX), parseFloat(goalY)];

  const data = {
    start: start,
    goal: goal,
    obstacles: obstacles.concat(dynamic_obstacles), // Concat static and dynamic obstacles
    bounds: bounds,
  };

  httpPost(
    "http://127.0.0.1:5000/plan_path",
    "json",
    data,
    function (response) {
      path = response.path;
      console.log("Path received:", path);
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
