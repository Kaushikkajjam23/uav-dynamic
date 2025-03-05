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

function setup() {
  createCanvas(800, 600, WEBGL);
  // Set the camera position relative to the origin
  perspective(PI / 3.0, width / height, 0.1, 1000);
}

function draw() {
  background(220);

  lights();

  // Draw obstacles with a 3D look
  for (let obs of obstacles) {
    push(); // Save current transformation matrix

    //Basic Z index offset
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
  //Update Drone position
  if (path.length > 0) {
    droneX = path[0][0];
    droneY = path[0][1];
  }

  // Draw Drone
  push();
  fill(0, 255, 0);
  translate(droneX - width / 2, droneY - height / 2, 0); // Translate to the drone's position and center
  rotateY(frameCount * 0.01); //Add Rotation!
  cone(20, 40); //Show a cone to imply movement
  pop();

  //Frame Count update for dynamic replanning every 30 frames
  if (frameCount % 30 == 0) {
    planPath();
  }
}
function planPath() {
  const data = {
    start: [100, 100],
    goal: [700, 500],
    obstacles: obstacles.concat(), // Concat static and dynamic obstacles, this is known to cause the issue . will tackle later
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
