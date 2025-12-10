# Bridging Python Agents to ROS 2 Controllers with `rclpy`

## The Nexus of AI and Robotics

`rclpy`, the Python client library for ROS 2, forms a critical bridge connecting intelligent Python-based AI agents with robust robot control systems. This chapter explores practical patterns for integrating these agents with ROS 2, enabling AI algorithms to perceive robot states, issue commands, and interact with the physical world through ROS 2 controllers. We will focus on common scenarios in humanoid robotics like high-level task planning, perception processing, and dynamic control.

## Understanding `rclpy`: The Pythonic Interface to ROS 2

`rclpy` is the official Python API for ROS 2. It provides all the necessary functionalities to create ROS 2 nodes, publish and subscribe to topics, call and provide services, and interact with actions, all within a familiar Pythonic syntax. Its design emphasizes ease of use for rapid prototyping and development, while still offering the performance benefits of ROS 2's underlying C++ core (RCL).

```mermaid
graph TD
    subgraph Python AI Agent
        A[AI Algorithm (TensorFlow/PyTorch)]
        B[Perception Logic (OpenCV)]
        C[Decision Making (LLM/Planning)]
        D[rclpy Node (Python)]
    end

    subgraph ROS 2 Ecosystem
        E[ROS 2 Topic: /joint_states] -- Data Flow --> D
        F[ROS 2 Topic: /cmd_vel] <-- Command Flow -- D
        G[ROS 2 Service: /robot_mode] <-- Service Call -- D
        H[ROS 2 Action: /navigate_to_pose] <-- Action Goal -- D
        I[ROS 2 Controller (C++/hardware)]
        J[Sensors (Lidar/Camera)]
    end

    A -- Calls D.publish() --> F
    B -- Calls D.subscribe() --> E
    C -- Calls D.call_service() --> G
    C -- Calls D.send_goal() --> H
    F --> I
    E <-- J
```
*Figure 2.1: Bridging Python AI agents to ROS 2 controllers via `rclpy`.*

## Key Integration Patterns

Integrating Python AI agents with ROS 2 typically involves one or more of the following patterns:

1.  **Sensor Data Processing (Subscriber)**: Python agents subscribe to sensor topics, process raw data (e.g., image recognition, point cloud segmentation), and potentially publish processed results.
2.  **Command Generation (Publisher)**: Python agents generate high-level or low-level commands (e.g., velocity commands, joint trajectories) and publish them to controller topics.
3.  **Task Orchestration (Service/Action Client)**: Python agents act as clients to trigger specific, discrete robot behaviors (services) or manage long-running tasks with feedback (actions).
4.  **Configuration/State Management (Service Server)**: Python agents can expose their internal state or configuration parameters via ROS 2 services, allowing other nodes to query or modify them.

### Lab 2.1: Python AI Agent for Simple Obstacle Avoidance (Publisher)

1.  **Create a new Python package (if not already done)**:
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python simple_avoidance_agent --dependencies rclpy geometry_msgs sensor_msgs
    cd simple_avoidance_agent/simple_avoidance_agent
    ```
2.  **Create `avoidance_agent.py`**:
    ```python
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist # For velocity commands
    from sensor_msgs.msg import LaserScan # For LiDAR data

    class AvoidanceAgent(Node):
        def __init__(self):
            super().__init__('avoidance_agent')
            self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
            self.subscription = self.create_subscription(
                LaserScan,
                '/scan', # Assuming your robot publishes LiDAR data on /scan
                self.scan_callback,
                10)
            self.subscription  # prevent unused variable warning
            self.get_logger().info('Avoidance Agent has started, waiting for scan data...')

            self.linear_speed = 0.2  # m/s
            self.angular_speed = 0.5 # rad/s

        def scan_callback(self, msg):
            # Very simple obstacle detection logic: check front-left, front, front-right sectors
            # LiDAR scan_ranges is typically 0 to 359 degrees, with 0 being straight ahead.
            # Adjust indices based on your LiDAR's angular resolution and range.
            num_ranges = len(msg.ranges)
            if num_ranges == 0:
                return

            # Example for a 360-degree LiDAR, adjust if your LiDAR is different (e.g., 180 degrees)
            # Front sector: ~ (-15 to +15 degrees)
            front_idx = range(0, 15)
            front_idx_alt = range(num_ranges - 15, num_ranges) # For wrap-around

            # Filter out 'inf' (no detection) and 'nan' (invalid data)
            valid_ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]

            if not valid_ranges:
                return # No valid obstacle data

            # Check a small sector in front
            threshold_distance = 0.7 # meters

            front_danger = False
            for i in front_idx:
                if msg.ranges[i] < threshold_distance:
                    front_danger = True
                    break
            for i in front_idx_alt:
                if msg.ranges[i] < threshold_distance:
                    front_danger = True
                    break

            twist_msg = Twist()
            if front_danger:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = self.angular_speed # Turn right to avoid
                self.get_logger().warn('Obstacle detected! Turning right.')
            else:
                twist_msg.linear.x = self.linear_speed # Move forward
                twist_msg.angular.z = 0.0
                self.get_logger().info('Path clear, moving forward.')

            self.publisher_.publish(twist_msg)

    def main(args=None):
        rclpy.init(args=args)
        avoidance_agent = AvoidanceAgent()
        try:
            rclpy.spin(avoidance_agent)
        except KeyboardInterrupt:
            pass
        avoidance_agent.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```
3.  **Update `setup.py`**:
    Add the entry point in `~/ros2_ws/src/simple_avoidance_agent/setup.py`:
    ```python
    entry_points={
        'console_scripts': [
            'avoidance_agent = simple_avoidance_agent.avoidance_agent:main',
        ],
    },
    ```
4.  **Build and Source**:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```
5.  **Run Simulation and Agent**:
    *   Terminal 1 (Start Gazebo with a robot):
        ```bash
        ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
        # You might need to manually spawn a turtlebot3_waffle or similar if it's not default
        # ros2 run gazebo_ros spawn_entity.py -entity turtlebot3_waffle -file $(ros2 pkg prefix turtlebot3_description)/share/turtlebot3_description/urdf/turtlebot3_waffle.urdf -x 0 -y 0 -z 0.1
        ```
    *   Terminal 2 (Run your Python agent):
        ```bash
        ros2 run simple_avoidance_agent avoidance_agent
        ```
    Observe the robot moving forward and turning when it detects an obstacle in Gazebo.

**Common Pitfall**: Incorrect topic names or message types, leading to the agent not receiving data or the robot not responding.
**Fix**: Use `ros2 topic list -t` and `ros2 topic info /scan` (or `/cmd_vel`) to verify.

## Lab 2.2: Python Agent for High-Level Task Orchestration (Action Client)

1.  **Create a new Python package (if not already done)**:
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python nav_agent --dependencies rclpy nav2_msgs geometry_msgs
    cd nav_agent/nav_agent
    ```
2.  **Create `nav_commander.py`**:
    ```python
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from nav2_msgs.action import NavigateToPose # Nav2's action type
    from geometry_msgs.msg import PoseStamped # For sending a target pose

    class NavCommander(Node):
        def __init__(self):
            super().__init__('nav_commander')
            self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.get_logger().info('Navigation Commander agent started.')

        def send_goal(self, x, y, yaw_degrees):
            self.get_logger().info('Waiting for Nav2 action server...')
            self._action_client.wait_for_server()

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map' # Usually 'map' frame for navigation goals
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = x
            goal_msg.pose.pose.position.y = y
            goal_msg.pose.pose.orientation.z = self.degrees_to_quaternion_z(yaw_degrees)
            goal_msg.pose.pose.orientation.w = self.degrees_to_quaternion_w(yaw_degrees)

            self.get_logger().info(f'Sending navigation goal: x={x}, y={y}, yaw={yaw_degrees} degrees')
            self._send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )

            self._send_goal_future.add_done_callback(self.goal_response_callback)

        def degrees_to_quaternion_z(self, degrees):
            import math
            return math.sin(math.radians(degrees) / 2)

        def degrees_to_quaternion_w(self, degrees):
            import math
            return math.cos(math.radians(degrees) / 2)

        def goal_response_callback(self, future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected by Nav2 server :(')
                return

            self.get_logger().info('Goal accepted by Nav2 server :)')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)

        def get_result_callback(self, future):
            status = future.result().status
            if status == ActionClient.GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Navigation Goal succeeded!')
            else:
                self.get_logger().warn(f'Navigation Goal failed with status: {status}')
            rclpy.shutdown()

        def feedback_callback(self, feedback_msg):
            # Nav2 provides detailed feedback, including current pose, distance remaining, etc.
            current_pose = feedback_msg.current_pose.pose.position
            distance_remaining = feedback_msg.distance_remaining
            self.get_logger().info(f'Feedback: Current Pose (x={current_pose.x:.2f}, y={current_pose.y:.2f}), '
                                   f'Distance Remaining: {distance_remaining:.2f}m')

    def main(args=None):
        rclpy.init(args=args)
        nav_commander = NavCommander()
        # Example goal: Go to x=2.0, y=1.5, facing 90 degrees (North)
        nav_commander.send_goal(2.0, 1.5, 90.0)
        rclpy.spin(nav_commander)

    if __name__ == '__main__':
        main()
    ```
3.  **Update `setup.py`**:
    Add the entry point in `~/ros2_ws/src/nav_agent/setup.py`:
    ```python
    entry_points={
        'console_scripts': [
            'nav_commander = nav_agent.nav_commander:main',
        ],
    },
    ```
4.  **Build and Source**:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```
5.  **Run Nav2 Simulation and Agent**:
    *   Terminal 1 (Start Nav2 with a robot, e.g., TurtleBot3):
        ```bash
        # Example for TurtleBot3
        export TURTLEBOT3_MODEL=waffle_pi # or burger
        ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/path/to/your/map.yaml
        ```
    *   Terminal 2 (Run your Python agent):
        ```bash
        ros2 run nav_agent nav_commander
        ```
    Observe the robot navigating to the specified pose in the simulation, with feedback logged by your agent.

**Common Pitfall**: Nav2 setup can be complex. Ensure your map is correctly loaded, localization (AMCL) is running, and the `navigate_to_pose` action server is active.
**Fix**: Debug Nav2 components separately before running the agent. Use `ros2 action list` to verify `navigate_to_pose` is available.

## Lab 2.3: Python Agent for Joint Position Control (Publisher/Service Client)

1.  **Create a new Python package**:
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python joint_controller_agent --dependencies rclpy trajectory_msgs std_srvs
    cd joint_controller_agent/joint_controller_agent
    ```
2.  **Create `humanoid_joint_controller.py`**:
    ```python
    import rclpy
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # For joint commands
    from std_srvs.srv import Trigger # For homing service

    class HumanoidJointController(Node):
        def __init__(self):
            super().__init__('humanoid_joint_controller')
            self.joint_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
            self.home_client = self.create_client(Trigger, '/robot_home_service') # Custom service to home robot (assumed)

            # Define joint names (replace with actual joint names from your humanoid's URDF)
            self.joint_names = [
                'left_shoulder_joint', 'left_elbow_joint',
                'right_shoulder_joint', 'right_elbow_joint',
                'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
                'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
                # Add more joints as per your robot's URDF
            ]
            self.get_logger().info('Humanoid Joint Controller agent started.')

        def send_joint_command(self, positions, time_from_start_sec=1.0):
            if len(positions) != len(self.joint_names):
                self.get_logger().error('Number of positions must match number of joints.')
                return

            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in positions] # Ensure floats
            point.time_from_start.sec = int(time_from_start_sec)
            point.time_from_start.nanosec = int((time_from_start_sec - int(time_from_start_sec)) * 1e9)
            msg.points.append(point)

            self.joint_publisher.publish(msg)
            self.get_logger().info(f'Sent joint command: {positions}')

        def call_home_service(self):
            if not self.home_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Home service not available.')
                return False

            request = Trigger.Request()
            future = self.home_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                self.get_logger().info(f'Home service response: {future.result().success}, {future.result().message}')
                return future.result().success
            else:
                self.get_logger().error('Home service call failed.')
                return False

    def main(args=None):
        rclpy.init(args=args)
        controller = HumanoidJointController()

        # Example: Move to a specific pose (e.g., 'neutral' pose)
        controller.get_logger().info('Moving robot to a neutral pose.')
        neutral_positions = [0.0] * len(controller.joint_names) # All joints to 0.0 radians
        controller.send_joint_command(neutral_positions, time_from_start_sec=2.0)
        rclpy.spin_once(controller, timeout_sec=2.5) # Allow time for command to be processed

        # Example: Call a homing service (if available)
        controller.get_logger().info('Calling home service...')
        controller.call_home_service()
        rclpy.spin_once(controller, timeout_sec=1.0)

        # Example: Move to a 'waving' pose (adjust positions for your specific humanoid)
        controller.get_logger().info('Moving right arm to a waving pose.')
        wave_positions = [0.0] * len(controller.joint_names)
        # Assuming 'right_shoulder_joint' is at index 2, 'right_elbow_joint' at index 3
        if len(controller.joint_names) > 3:
            wave_positions[2] = -1.0 # Raise arm
            wave_positions[3] = -0.5 # Bend elbow
        controller.send_joint_command(wave_positions, time_from_start_sec=1.5)
        rclpy.spin_once(controller, timeout_sec=2.0)

        controller.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```
3.  **Update `setup.py`**:
    Add the entry point in `~/ros2_ws/src/joint_controller_agent/setup.py`:
    ```python
    entry_points={
        'console_scripts': [
            'humanoid_joint_controller = joint_controller_agent.humanoid_joint_controller:main',
        ],
    },
    ```
4.  **Build and Source**:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```
5.  **Run Simulated Humanoid and Agent**:
    *   Terminal 1 (Start Gazebo with a humanoid robot and a joint trajectory controller, e.g., `ros2 launch my_humanoid_config humanoid.launch.py`):
        Ensure your humanoid model publishes joint states and has a controller subscribed to `/joint_trajectory_controller/joint_trajectory`.
        You might need a custom service server for `/robot_home_service` for the `call_home_service` to work.
    *   Terminal 2 (Run your Python agent):
        ```bash
        ros2 run joint_controller_agent humanoid_joint_controller
        ```
    Observe the simulated humanoid robot moving its joints as commanded.

**Common Pitfall**: Incorrect joint names or an unavailable joint trajectory controller.
**Fix**: Verify joint names from your robot's URDF. Ensure the correct `controller_manager` and `joint_trajectory_controller` are loaded in your simulation.

## Python AI for Perception (Subscriber/Publisher)

1.  **Create a new Python package**:
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python simple_vision_agent --dependencies rclpy sensor_msgs cv_bridge python-opencv
    cd simple_vision_agent/simple_vision_agent
    ```
2.  **Create `color_detector.py`**:
    ```python
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2
    import numpy as np

    class ColorDetector(Node):
        def __init__(self):
            super().__init__('color_detector')
            self.subscription = self.create_subscription(
                Image,
                '/camera/image_raw', # Raw camera image topic
                self.image_callback,
                10)
            self.publisher_ = self.create_publisher(Image, '/vision/segmented_image', 10)
            self.bridge = CvBridge()
            self.get_logger().info('Color Detector agent started, waiting for image data.')

            # Define the desired color range (e.g., for detecting a green object in HSV)
            # You might need to adjust these values based on your camera and lighting
            self.lower_green = np.array([40, 40, 40])
            self.upper_green = np.array([80, 255, 255])
            self.get_logger().info(f"Detecting color in HSV range: {self.lower_green} to {self.upper_green}")

        def image_callback(self, msg):
            try:
                # Convert ROS Image message to OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                self.get_logger().error(f"Error converting image: {e}")
                return

            # Convert BGR to HSV
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Threshold the HSV image to get only green colors
            mask = cv2.inRange(hsv_image, self.lower_green, self.upper_green)

            # Bitwise-AND mask and original image
            segmented_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            # Find contours to detect objects
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            detected_objects = 0
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500: # Filter small noise
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(segmented_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    detected_objects += 1

            if detected_objects > 0:
                self.get_logger().info(f"Detected {detected_objects} green objects.")

            # Convert OpenCV image back to ROS Image message and publish
            try:
                ros_image = self.bridge.cv2_to_imgmsg(segmented_image, 'bgr8')
                ros_image.header = msg.header # Maintain timestamp and frame_id
                self.publisher_.publish(ros_image)
            except Exception as e:
                self.get_logger().error(f"Error converting and publishing image: {e}")

    def main(args=None):
        rclpy.init(args=args)
        color_detector = ColorDetector()
        try:
            rclpy.spin(color_detector)
        except KeyboardInterrupt:
            pass
        color_detector.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```
3.  **Update `setup.py`**:
    Add the entry point in `~/ros2_ws/src/simple_vision_agent/setup.py`:
    ```python
    entry_points={
        'console_scripts': [
            'color_detector = simple_vision_agent.color_detector:main',
        ],
    },
    ```
4.  **Build and Source**:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```
5.  **Run Camera Source and Agent**:
    *   Terminal 1 (Start a camera publisher, e.g., from a Gazebo simulation with a camera or a USB camera driver):
        ```bash
        # Example for a simulated camera (e.g., from a robot description)
        ros2 launch your_robot_description display.launch.py
        # Or a real USB camera
        ros2 run camera_ros camera_ros_driver
        ```
    *   Terminal 2 (Run your Python agent):
        ```bash
        ros2 run simple_vision_agent color_detector
        ```
    *   Terminal 3 (Visualize the output):
        ```bash
        rqt_image_view /vision/segmented_image
        ```
    Observe the `rqt_image_view` displaying the segmented image with detected green objects highlighted.

**Common Pitfall**: `cv_bridge` not correctly installed or `python-opencv` missing.
**Fix**: Ensure `apt-get install ros-<ROS_DISTRO>-cv-bridge python3-opencv` (for Debian/Ubuntu) or `pip install opencv-python` in your ROS 2 environment.

## Common Pitfalls and Solutions for Python-ROS 2 Integration

*   **Missing Dependencies**: Forgetting to declare dependencies in `package.xml` and `setup.py` (e.g., `rclpy`, `geometry_msgs`, `sensor_msgs`, `nav2_msgs`).
    *   **Fix**: Always check `package.xml` for `build_depend`/`exec_depend` and `setup.py` for `install_requires`.
*   **Not Sourcing Environment**: `ros2` commands or Python scripts failing because ROS 2 environment variables aren't set.
    *   **Fix**: `source /opt/ros/<ROS_DISTRO>/setup.bash` and `source ~/ros2_ws/install/setup.bash` in *every new terminal*.
*   **Incorrect Message/Service/Action Types**: Mismatch between publisher/subscriber or client/server types.
    *   **Fix**: Use `ros2 topic info <topic>`, `ros2 service type <service>`, `ros2 action info <action>` to verify.
*   **Custom Interface Definition Issues**: Not rebuilding after changing `.srv`, `.action`, or `.msg` files, or missing `rosidl_default_generators` dependencies.
    *   **Fix**: `colcon build` and `source install/setup.bash` after any interface changes. Ensure `package.xml` and `setup.py` are correctly configured for custom interfaces.
*   **Synchronous `rclpy.spin()` in Long-Running Tasks**: `rclpy.spin(node)` blocks the thread, preventing other callbacks or agent logic from running.
    *   **Fix**: For complex agents, consider using `rclpy.spin_once(node, timeout_sec=...)` in a loop, `rclpy.callback_groups.MutuallyExclusiveCallbackGroup`, or dedicated threads for blocking operations to keep the main node responsive.

## Student Exercises (with Hidden Solutions)

**Exercise 2.1: Advanced Obstacle Avoidance**
Enhance the `avoidance_agent.py` to differentiate between obstacles on the left and right sides. If an obstacle is detected on the left, turn right. If on the right, turn left. If directly in front, stop and turn randomly.

<details>
  <summary>Solution (click to expand)</summary>

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random

class AdvancedAvoidanceAgent(Node):
    def __init__(self):
        super().__init__('advanced_avoidance_agent')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription
        self.get_logger().info('Advanced Avoidance Agent has started, waiting for scan data...')

        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5 # rad/s
        self.threshold_distance = 0.7 # meters

    def scan_callback(self, msg):
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        # Define sectors for front, left, and right (adjust indices based on your LiDAR)
        # Assuming 360-degree LiDAR, 0 degrees is front
        front_left_sector = [i for i in range(315, 360)] # e.g., 315 to 359
        front_right_sector = [i for i in range(0, 45)]   # e.g., 0 to 44

        # Filter valid ranges
        ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        if not ranges:
            self.move_forward() # If no obstacles, move forward
            return

        left_obstacle = any(msg.ranges[i] < self.threshold_distance for i in front_left_sector if i < num_ranges)
        right_obstacle = any(msg.ranges[i] < self.threshold_distance for i in front_right_sector if i < num_ranges)
        
        # Check if anything directly in front (a narrower band)
        narrow_front_sector = [i for i in range(350, 360)] + [i for i in range(0, 10)]
        front_danger = any(msg.ranges[i] < self.threshold_distance for i in narrow_front_sector if i < num_ranges)


        twist_msg = Twist()
        if front_danger:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = random.choice([-self.angular_speed, self.angular_speed]) # Random turn
            self.get_logger().warn('Obstacle directly in front! Stopping and turning randomly.')
        elif left_obstacle:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -self.angular_speed # Turn right
            self.get_logger().warn('Obstacle on left! Turning right.')
        elif right_obstacle:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.angular_speed # Turn left
            self.get_logger().warn('Obstacle on right! Turning left.')
        else:
            twist_msg.linear.x = self.linear_speed
            twist_msg.angular.z = 0.0
            self.get_logger().info('Path clear, moving forward.')

        self.publisher_.publish(twist_msg)

    def move_forward(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        twist_msg.angular.z = 0.0
        self.publisher_.publish(twist_msg)
        self.get_logger().info('No obstacles detected, moving forward.')


def main(args=None):
    rclpy.init(args=args)
    agent = AdvancedAvoidanceAgent()
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Remember to add `advanced_avoidance_agent = simple_avoidance_agent.advanced_avoidance_agent:main` to `setup.py` console_scripts and rebuild.
</details>

**Exercise 2.2: Robot State Logger (Subscriber)**
Create a Python agent that subscribes to the `/joint_states` topic (message type `sensor_msgs/msg/JointState`, which is common for humanoid robots). This agent should log the names and positions of all joints whenever an update is received.

<details>
  <summary>Solution (click to expand)</summary>

**`my_robot_pkg/my_robot_pkg/joint_state_logger.py`**:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState # For joint state data

class JointStateLogger(Node):
    def __init__(self):
        super().__init__('joint_state_logger')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states', # Common topic for joint states
            self.joint_state_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Joint State Logger agent started, waiting for joint state data.')

    def joint_state_callback(self, msg):
        self.get_logger().info('--- Joint State Update ---')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else 'N/A'
            velocity = msg.velocity[i] if i < len(msg.velocity) else 'N/A'
            effort = msg.effort[i] if i < len(msg.effort) else 'N/A'
            self.get_logger().info(f'Joint: {name}, Position: {position:.4f}, Velocity: {velocity:.4f}, Effort: {effort:.4f}')

def main(args=None):
    rclpy.init(args=args)
    joint_state_logger = JointStateLogger()
    try:
        rclpy.spin(joint_state_logger)
    except KeyboardInterrupt:
        pass
    joint_state_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Add to `setup.py` `console_scripts`:
`'joint_state_logger = my_robot_pkg.joint_state_logger:main',`
Run with: `ros2 run my_robot_pkg joint_state_logger` (assuming a robot publishing `/joint_states` in simulation or real hardware).
</details>

**Exercise 2.3: Simple Service for Robot Status**
Create a Python agent that provides a ROS 2 service `/robot_status` using `std_srvs/srv/Trigger`. When called, this service should return `success=True` and a message indicating the robot is "Online and ready."

<details>
  <summary>Solution (click to expand)</summary>

**`my_robot_pkg/my_robot_pkg/robot_status_server.py`**:
```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger # Standard Trigger service type

class RobotStatusService(Node):
    def __init__(self):
        super().__init__('robot_status_server')
        self.srv = self.create_service(Trigger, 'robot_status', self.status_callback)
        self.get_logger().info('Robot Status Service ready at /robot_status.')

    def status_callback(self, request, response):
        response.success = True
        response.message = 'Robot is Online and ready.'
        self.get_logger().info('Received status request. Responding: Online and ready.')
        return response

def main(args=None):
    rclpy.init(args=args)
    robot_status_service = RobotStatusService()
    try:
        rclpy.spin(robot_status_service)
    except KeyboardInterrupt:
        pass
    robot_status_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Add to `setup.py` `console_scripts`:
`'robot_status_server = my_robot_pkg.robot_status_server:main',`
Run with: `ros2 run my_robot_pkg robot_status_server` (in one terminal) and `ros2 service call /robot_status std_srvs/srv/Trigger "{}"` (in another).
</details>

## Further Reading and Official Resources

*   **ROS 2 `rclpy` Tutorials**: The official tutorials are an excellent resource for getting started with Python in ROS 2.
    *   [Writing a Simple Publisher and Subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
    *   [Writing a Simple Service and Client (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)
    *   [Writing an Action Server and Client (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Action-Server-And-Client.html)
*   **ROS 2 CLI Tools**: Essential for debugging and introspection.
    *   [Using ros2 topic CLI tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Using-ROS2-cli-Tools.html)
*   **OpenCV-Python Tutorials**: For deepening your understanding of computer vision tasks.
    *   [OpenCV-Python Tutorials](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
*   **Nav2 Documentation**: For advanced navigation capabilities.
    *   [Navigation2 Documentation](https://navigation.ros.org/)

By effectively utilizing `rclpy`, Python AI agents become powerful components within the ROS 2 ecosystem, transforming complex algorithms into real-world robotic behaviors. This modularity and rich toolset empower developers to design highly capable Physical AI systems. The next chapter will explore the critical role of URDF in defining these robotic systems.