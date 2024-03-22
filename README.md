Challenges in Robotics: 

Autonomous navigation in robotics presents several challenges. Robots must interact with dynamic environments, make precise movements, and make real-time decisions to reach their goals while avoiding collisions with obstacles. These complexities demand adaptive control strategies.

Specific Problem: 

The specific problem addressed in this project is the need for an adaptive control system that can manage varying distances to the target and integrate obstacle detection and avoidance mechanisms seamlessly. The goal is to create a control system that is robust, responsive, and capable of adapting to changing scenarios.

Code Breakdown and Explanation
Initialization:

The code begins by initializing the connection to the robot simulation environment (V-REP) and obtaining handles for various components, including motors, sensors, and the robot itself. This setup is crucial for communication and control.

Main Control Loop:

The main control loop continuously reads sensor data, computes control actions, and adjusts the robot's movement. It encompasses the entire control process.

PID Control Implementation:

The code implements PID controllers for both angular and positional control. These controllers help the robot maintain the desired angle and distance from the target.

Obstacle Avoidance Logic:

The code incorporates obstacle avoidance logic using proximity sensor data. It calculates a movement direction based on the sensor readings and employs strategies to avoid obstacles effectively.

Adaptive Gains Control:

Rationale for Adaptive Gains
Adaptive gains are essential for efficient control at different distances from the target. The code dynamically adjusts PID gains to optimize the robot's performance in various scenarios.
Implementation Details
The code dynamically modifies PID gains based on the robot's current distance from the target, ensuring optimal control throughout the navigation process.

Integration of PID Control and Obstacle Avoidance
