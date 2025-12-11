---
sid ebar_position: 3
---

# The AI-Robot Brain: Nav2 for Bipedal Path Planning

Navigating a wheeled robot like the Carter is a well-solved problem. Navigating a bipedal humanoid, however, is a monumental challenge. Humanoids are inherently unstable, have complex kinematics, and interact with the world in a much more dynamic way. While the standard Nav2 stack is a great starting point, applying it to humanoids requires special considerations and custom plugins.

This chapter delves into the advanced topic of adapting Nav2 for bipedal locomotion. We won't be building a perfect bipedal walker from scratch, but we will explore the key concepts, the limitations of default Nav2, and the types of modifications required. We will focus on how a high-level planner can generate a path and how a specialized local planner and controller are needed to turn that path into stable footsteps.

## The Challenge: Wheeled vs. Bipedal Navigation

The standard Nav2 stack makes a key assumption: the robot is a differential drive or holonomic platform. Its motion is continuous and can be controlled by publishing `geometry_msgs/Twist` messages (linear and angular velocity).

A bipedal robot's motion is fundamentally different:
-   **Discrete Motion**: It moves in discrete steps (footsteps), not a continuous glide.
-   **Dynamic Stability**: It must actively balance to avoid falling over. The center of mass, Zero Moment Point (ZMP), and support polygon are critical concepts.
-   **Complex Control**: Commanding a biped is not as simple as sending a velocity. It involves commanding a sequence of footsteps, which a lower-level controller then translates into joint torques and positions for the legs.

Therefore, we cannot use the default Nav2 controllers (like DWB or TEB) directly. The bridge between Nav2's high-level plan and the robot's legs must be rebuilt.

```mermaid
graph TD
    subgraph Standard Nav2 for Wheeled Robots
        A[Global Plan (NavFn, Smac Planner)] --> B[Local Planner/Controller (DWB, TEB)];
        B -- Publishes --> C((/cmd_vel [Twist]));
    end
    subgraph Modified Nav2 for Bipedal Robots
        D[Global Plan (NavFn, Smac Planner)] --> E[**Custom Bipedal Local Planner**];
        E -- Publishes --> F((/planned_footsteps [FootstepArray]));
        F --> G[**Whole Body Controller**];
        G -- Commands --> H[Leg Joints];
    end
```
The key difference is that the output of the local planner is not a velocity command, but a **sequence of planned footsteps**.

--- 

## The Components of a Bipedal Navigation System

### 1. The Global Planner
This part of Nav2 can often be used as-is. Planners like `Smac Planner` or `NavFn` are excellent at finding a valid, collision-free path (a sequence of x, y coordinates) from the start to the goal on a 2D costmap. This high-level path is still a valid and useful guide for a humanoid.

### 2. The Local Planner (Custom)
This is where the magic happens. A custom local planner for a biped would:
-   Subscribe to the global plan from Nav2.
-   Look at the next segment of the global path.
-   Generate a short sequence of discrete footstep placements (position and orientation for the left and right feet) that follow the path.
-   Check each proposed footstep for validity (e.g., is the ground flat? Is it too far from the other foot?).
-   Publish this sequence of footsteps to a custom topic (e.g., `/planned_footsteps`).

### 3. The Whole-Body Controller
This is a low-level, high-frequency control loop that is responsible for executing the footsteps and keeping the robot balanced. It would:
-   Subscribe to the `/planned_footsteps` topic.
-   Use an inverse kinematics solver to calculate the required joint angles for the legs to place the foot at the desired location.
-   Use feedback from the IMU and foot pressure sensors to constantly adjust the robot's posture (e.g., by shifting the torso) to keep the center of mass over the support foot.
-   This is an incredibly complex piece of software, often involving techniques like Model Predictive Control (MPC).

--- 
## Lab: Simulating a Bipedal Planner (Conceptual) 

Actually implementing a full bipedal local planner and controller is beyond the scope of a single chapter. However, we can create a "mock" version to understand the data flow and the logic involved.

In this lab, we will:
1.  Run the standard Nav2 stack to generate a global plan.
2.  Create a Python script that acts as a **dummy bipedal local planner**. It will subscribe to the global plan and, instead of generating real footsteps, it will simply print out where it *would* place the next footstep.
3.  Visualize this process in RViz.

**Prerequisites**:
-   A working Nav2 simulation (you can use the VSLAM setup from the previous lab).

```bash
# In one terminal, run Isaac Sim with the Carter robot
# In another terminal, run the Isaac ROS Docker container
docker run --rm -it --net=host --gpus all \
  -v ~/isaac_ros_ws:/workspaces/isaac_ros_ws \
  nvcr.io/isaac/isaac-ros-vslam:2.0.0

# Inside the container
ros2 launch isaac_ros_vslam isaac_ros_vslam_nav2.launch.py
```

### Step 1: Launch Nav2
Launch the Isaac ROS VSLAM and Nav2 demo from the previous chapter. Make sure you have a map and the robot is localized.

### Step 2: Write the Dummy Bipedal Planner Node
Create a new Python package `bipedal_planner_py` with dependencies on `rclpy`, `nav_msgs`, and `geometry_msgs`. Then, create the node.

**File**: `bipedal_planner_py/bipedal_planner_py/dummy_planner.py`
```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

class DummyBipedalPlanner(Node):
    def __init__(self):
        super().__init__('dummy_bipedal_planner')
        self.plan_subscription = self.create_subscription(
            Path, '/plan', self.plan_callback, 10)
        
        self.footstep_publisher = self.create_publisher(Marker, '/planned_footstep_marker', 10)
        
        self.step_length = 0.3  # 30 cm steps
        self.is_left_foot = True
        self.last_footstep_pos = None

        self.get_logger().info('Dummy Bipedal Planner has started.')

    def plan_callback(self, msg: Path):
        self.get_logger().info(f'Received a new global plan with {len(msg.poses)} poses.')
        if not msg.poses:
            return

        # Initialize the first footstep at the start of the plan
        if self.last_footstep_pos is None:
            self.last_footstep_pos = msg.poses[0].pose.position

        # Iterate through the path and generate "footsteps"
        for pose in msg.poses:
            current_pos = pose.pose.position
            dist_from_last_step = self.distance(current_pos, self.last_footstep_pos)

            if dist_from_last_step >= self.step_length:
                self.get_logger().info(f'Placing {"left" if self.is_left_foot else "right"} footstep near ({current_pos.x:.2f}, {current_pos.y:.2f})')
                self.publish_footstep_marker(current_pos)
                self.last_footstep_pos = current_pos
                self.is_left_foot = not self.is_left_foot
                
def distance(self, p1: Point, p2: Point):
    return ((p1.x - p2.x)**2 + (p1.y - p2.y)**2)**0.5

def publish_footstep_marker(self, position: Point):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = self.get_clock().now().to_msg()
    marker.ns = "footsteps"
    marker.id = 0 # In a real app, this should be unique and incrementing
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position = position
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.25
    marker.scale.y = 0.15
    marker.scale.z = 0.01
    marker.color.a = 1.0
    marker.color.r = 0.0 if self.is_left_foot else 1.0
    marker.color.g = 1.0 if not self.is_left_foot else 0.0
    marker.color.b = 0.0
    self.footstep_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    planner = DummyBipedalPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
- **Add the entry point** to `setup.py` and `colcon build` your package.

### Step 3: Run and Visualize
1.  In a new terminal (inside the Docker container), run your dummy planner:
    ```bash
    ros2 run bipedal_planner_py dummy_planner
    ```
2.  Open RViz2.
3.  **Add** a `Marker` display and set the topic to `/planned_footstep_marker`.
4.  Use the **Nav2 Goal** tool to set a destination.

When you set the goal, Nav2 will publish a global plan to the `/plan` topic. Your dummy planner node will receive this, and as it processes the path, it will publish `Marker` messages. You will see a series of green and red rectangles appear in RViz, tracing the path from the robot to the goal. This simulates the output of a bipedal local planner!

*A screenshot of RViz showing the global plan (a green line) and a series of alternating red and green rectangles (the footstep markers) along that path.*

## Common Pitfalls for Bipedal Navigation

1.  **Pitfall**: The robot is unstable and falls over.
    *   **Cause**: This is the single biggest challenge. The Whole Body Controller is failing. The dynamics of the robot, the friction of the ground, or external disturbances were not accounted for properly.
    *   **Fix**: This is an area of active research. Solutions involve better state estimation (IMUs, foot sensors), more advanced control strategies (MPC), and careful tuning of the controller gains. In simulation, ensuring your URDF's inertial properties are accurate is a critical first step.

2.  **Pitfall**: Footsteps are placed in invalid locations (e.g., on an obstacle, off a ledge).
    *   **Cause**: The local planner is not checking the validity of footsteps against a local costmap or height map.
    *   **Fix**: The bipedal local planner must have its own local costmap. Before publishing a footstep, it must check the cost of the area under the proposed footstep. If the cost is too high (indicating an obstacle or a slope), the step must be rejected and a new one must be planned (e.g., stepping to the side).

## Student Exercises

<details>
<summary>Exercise 1: Adjust the Step Length</summary>
<div>
**Task**: In the `dummy_planner.py` script, change the `self.step_length` to be much larger (e.g., `1.0`) and much smaller (e.g., `0.1`). Rerun the simulation and observe how it changes the spacing of the footstep markers in RViz.

**Reflection**: This demonstrates the trade-off between speed and stability. Larger steps cover ground faster but are generally less stable and harder to execute. Smaller steps are more stable but result in slower locomotion.
</div>
</details>

<details>
<summary>Exercise 2: Conceptualize Obstacle Avoidance</summary>
<div>
**Task**: Modify the `dummy_planner.py` to *simulate* checking for obstacles. You don't have to implement a real costmap, just the logic.

**Solution Steps**:
1. Subscribe to the `/global_costmap/costmap` topic (or a similar costmap topic from Nav2). This gives you a grid of obstacle data.
2. In your `plan_callback`, when you are about to place a footstep at `current_pos`, you first need to convert this world coordinate (`current_pos.x`, `current_pos.y`) into a grid coordinate on the costmap.
3. Check the value of the costmap at that grid coordinate.
4. Add an `if` statement: `if costmap_value < THRESHOLD: ... publish footstep ... else: ... log that you are avoiding an obstacle ...`.
5. This simulates the fundamental logic that a real bipedal planner would need to use.
</div>
</details>

## Further Reading
- **The Nav2 Documentation**: [https://navigation.ros.org/](https://navigation.ros.org/) (Especially the section on writing a new Planner Plugin).
- **Zero Moment Point (ZMP)**: A key concept in walking robot stability. [https://en.wikipedia.org/wiki/Zero_moment_point](https://en.wikipedia.org/wiki/Zero_moment_point)
- **Model Predictive Control (MPC)**: A popular advanced control technique used for humanoid locomotion. [https://en.wikipedia.org/wiki/Model_predictive_control](https://en.wikipedia.org/wiki/Model_predictive_control)
- **Boston Dynamics Atlas Papers**: For state-of-the-art research, look at publications from Boston Dynamics on the control of their Atlas robot.

```