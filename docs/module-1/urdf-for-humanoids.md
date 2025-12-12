# Understanding URDF for Humanoid Robots: Anatomy of a Digital Body

## Introduction to the Unified Robot Description Format

The Unified Robot Description Format (URDF) is an XML-based file format used in ROS 2 (and its predecessor ROS 1) to describe all the physical and kinematic properties of a robot. For humanoid robots, URDF is not merely a descriptive tool; it is the fundamental blueprint that defines their digital anatomy, dictating how they are simulated, controlled, and perceived within the robotic ecosystem. It captures everything from the physical segments (links) of the robot's body to the connections (joints) that allow movement, including sensory elements, visual meshes, and collision properties.

In the context of Physical AI, a precise URDF model is indispensable. It serves as the single source of truth for inverse kinematics calculations (how to move joints to reach a target pose), dynamic simulations (how the robot moves under gravity and forces), collision detection (preventing the robot from self-colliding or hitting objects), and visualization (displaying the robot accurately in tools like RViz or Gazebo). Without a robust and accurate URDF, intelligent control algorithms developed in Python or other languages would lack the foundational understanding of the robot's physical structure, leading to erroneous behavior and potentially dangerous interactions in the real world.

## Core Components of URDF

A URDF file primarily consists of two main elements:

1.  **`<link>`**: Represents a rigid body segment of the robot. This could be a torso, a leg, an arm, a head, or even a sensor housing. Links have associated inertial properties (mass, inertia matrix), visual properties (geometry, color, mesh files), and collision properties (geometry for collision checking).
2.  **`<joint>`**: Describes how two links are connected and the type of motion allowed between them. Joints define the robot's degrees of freedom (DOF). Common joint types include:
    *   `revolute`: A single rotational degree of freedom around an axis, with limits.
    *   `continuous`: A revolute joint without limits (e.g., a wheel).
    *   `prismatic`: A single translational degree of freedom along an axis, with limits.
    *   `fixed`: No motion allowed; rigidly connects two links.
    *   `planar`: Two translational and one rotational DOF (in a plane).
    *   `floating`: All six DOFs (three translational, three rotational), typically used for the base link of a mobile robot or the root of a humanoid model in a floating base configuration.

Each joint connects a `parent` link to a `child` link.

```mermaid
graph TD
    A[Base Link (e.g., Torso)] --> J1(Joint 1: Waist)
J1 --> L1[Link 1: Lower Body]
L1 --> J2(Joint 2: Hip L)
J2 --> L2[Link 2: Upper Leg L]
L2 --> J3(Joint 3: Knee L)
J3 --> L3[Link 3: Lower Leg L]
L3 --> J4(Joint 4: Ankle L)
J4 --> L4[Link 4: Foot L]

J1 --> J5(Joint 5: Hip R)
J5 --> L5[Link 5: Upper Leg R]
L5 --> J6(Joint 6: Knee R)
J6 --> L6[Link 6: Lower Leg R]
L6 --> J7(Joint 7: Ankle R)
J7 --> L7[Link 7: Foot R]

A --> J8(Joint 8: Shoulder L)
J8 --> L8[Link 8: Upper Arm L]
L8 --> J9(Joint 9: Elbow L)
J9 --> L9[Link 9: Forearm L]
L9 --> J10(Joint 10: Wrist L)
J10 --> L10[Link 10: Hand L]

J8 --> J11(Joint 11: Shoulder R)
J11 --> L11[Link 11: Upper Arm R]
L11 --> J12(Joint 12: Elbow R)
J12 --> L12[Link 12: Forearm R]
L12 --> J13(Joint 13: Wrist R)
J13 --> L13[Link 13: Hand R]
```
*Figure 3.1: Simplified kinematic chain for a humanoid robot, illustrating links and joints.*

## Building a Simple Humanoid Leg Segment URDF

Let's construct a basic URDF for a single leg segment of a humanoid robot. This will introduce the fundamental XML structure.

**Lab 3.1: Creating a Basic Leg Segment URDF**

**Goal**: Define a URDF for a single upper leg link connected by a revolute hip joint to a fixed base.

1.  **Create a new ROS 2 package**:
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake my_humanoid_description --dependencies rclpy urdf_tutorial
    cd my_humanoid_description
    mkdir urdf rviz
    ```
    (Note: `ament_cmake` is generally preferred for URDF packages to utilize `install` targets for meshes, etc.)
2.  **Create `single_leg.urdf` in the `urdf` directory**:
    ```xml
    <?xml version="1.0"?>
    <robot name="simple_humanoid_leg">

      <!-- Base Link (Root of the leg chain) -->
      <link name="base_link">
        <visual>
          <geometry>
            <box size="0.1 0.1 0.05"/>
          </geometry>
          <material name="grey">
            <color rgba="0.7 0.7 0.7 1.0"/>
          </material>
        </visual>
        <collision>
          <geometry>
            <box size="0.1 0.1 0.05"/>
          </geometry>
        </collision>
        <inertial>
          <mass value="1.0"/>
          <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
        </inertial>
      </link>

      <!-- Hip Joint -->
      <joint name="hip_joint" type="revolute">
        <parent link="base_link"/>
        <child link="upper_leg_link"/>
        <origin xyz="0 0 -0.05" rpy="0 0 0"/> <!-- Offset from base_link to connect to top of upper_leg_link -->
        <axis xyz="1 0 0"/> <!-- Rotation around X-axis -->
        <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
      </joint>

      <!-- Upper Leg Link -->
      <link name="upper_leg_link">
        <visual>
          <geometry>
            <cylinder radius="0.05" length="0.4"/>
          </geometry>
          <origin xyz="0 0 -0.2" rpy="0 0 0"/> <!-- Center of cylinder at origin, so move down by half length -->
          <material name="blue">
            <color rgba="0.0 0.0 0.8 1.0"/>
          </material>
        </visual>
        <collision>
          <geometry>
            <cylinder radius="0.05" length="0.4"/>
          </geometry>
          <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        </collision>
        <inertial>
          <mass value="2.0"/>
          <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
        </inertial>
      </link>

      <!-- Material definitions (good practice to centralize) -->
      <material name="grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>

    </robot>
    ```
3.  **Create a launch file to display the URDF in RViz**:
    ```bash
    mkdir -p ~/ros2_ws/src/my_humanoid_description/launch
    ```
    Create `display_single_leg.launch.py` in `launch` directory:
    ```python
    from launch import LaunchDescription
    from launch_ros.actions import Node
    from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
    from launch_ros.parameter_descriptors import ParameterValue
    from launch_ros.substitutions import FindPackageShare

    def generate_launch_description():
        robot_description_content = ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name="xacro")]) , " ",
                PathJoinSubstitution([
                    FindPackageShare("my_humanoid_description"), "urdf", "single_leg.urdf"
                ])
            ]),
            value_type=str
        )

        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description_content}],
            output="screen"
        )

        joint_state_publisher_gui_node = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen"
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([FindPackageShare("urdf_tutorial"), "rviz", "urdf.rviz"])],
            # If urdf_tutorial is not available, you can create a simple rviz config or run rviz2 without -d
            # and manually add RobotModel and JointState components.
        )

        return LaunchDescription([
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node
        ])
    ```
4.  **Update `package.xml`**:
    Add `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `xacro`, `urdf_tutorial` (if using its rviz config) as dependencies.
    ```xml
    <depend>rclpy</depend>
    <depend>robot_state_publisher</depend>
    <depend>joint_state_publisher_gui</depend>
    <depend>rviz2</depend>
    <depend>xacro</depend>
    <depend>urdf_tutorial</depend> <!-- Only if using its rviz config -->
    ```
5.  **Build and Source**:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```
6.  **Launch the display**:
    ```bash
    ros2 launch my_humanoid_description display_single_leg.launch.py
    ```
    You should see the base link and upper leg link in RViz, with a GUI to control the hip joint.

**Common Pitfall**: Incorrect `origin` values in `<joint>` tags lead to links being positioned incorrectly or disjointedly.
**Fix**: Carefully visualize the coordinate frames and ensure `xyz` and `rpy` values correctly transform from the parent's frame to the child's frame. Use `rviz` to debug link placements.

## Humanoid-Specific URDF Considerations

While the basic `<link>` and `<joint>` elements are universal, designing URDFs for humanoid robots presents unique challenges:

1.  **High Degrees of Freedom (DOF)**: Humanoids typically have 30+ DOFs, requiring careful organization and naming conventions.
2.  **Kinematic Chains**: Multiple complex kinematic chains for arms, legs, and torso.
3.  **Balance and Stability**: Inertial properties (mass, center of mass, inertia matrix) are critical for dynamic walking and balance control. An accurate URDF is the foundation for whole-body control.
4.  **Self-Collision**: Humanoid limbs can easily collide with each other. Collision meshes need to be defined precisely to prevent self-intersections.
5.  **Sensors**: Integrating cameras, LiDAR, IMUs, force/torque sensors, and other perception components directly into the URDF.

## Advanced URDF with Xacro

Writing complex URDF files directly in XML can become repetitive and error-prone. **Xacro** (XML Macros) is a powerful tool that allows for modularity, parameterization, and simplification of URDF files. It uses macro definitions and property assignments to generate the final URDF XML.

**Lab 3.2: Parameterizing a Joint with Xacro**

**Goal**: Convert the `single_leg.urdf` to `single_leg.urdf.xacro` and parameterize the hip joint limits.

1.  **Rename `single_leg.urdf` to `single_leg.urdf.xacro`**:
    ```bash
    mv ~/ros2_ws/src/my_humanoid_description/urdf/single_leg.urdf ~/ros2_ws/src/my_humanoid_description/urdf/single_leg.urdf.xacro
    ```
2.  **Edit `single_leg.urdf.xacro`**:
    Add the xacro namespace and properties, and use them in the joint definition.
    ```xml
    <?xml version="1.0"?>
    <robot name="simple_humanoid_leg" xmlns:xacro="http://www.ros.org/wiki/xacro">

      <!-- Xacro Properties -->
      <xacro:property name="M_PI" value="3.1415926535897931" />
      <xacro:property name="hip_limit_lower" value="${-M_PI/2}" /> <!-- -90 degrees -->
      <xacro:property name="hip_limit_upper" value="${M_PI/2}" />  <!-- 90 degrees -->
      <xacro:property name="hip_effort" value="10.0" />
      <xacro:property name="hip_velocity" value="1.0" />

      <!-- Base Link (Root of the leg chain) -->
      <link name="base_link">
        <visual>
          <geometry>
            <box size="0.1 0.1 0.05"/>
          </geometry>
          <material name="grey">
            <color rgba="0.7 0.7 0.7 1.0"/>
          </material>
        </visual>
        <collision>
          <geometry>
            <box size="0.1 0.1 0.05"/>
          </geometry>
        </collision>
        <inertial>
          <mass value="1.0"/>
          <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
        </inertial>
      </link>

      <!-- Hip Joint -->
      <joint name="hip_joint" type="revolute">
        <parent link="base_link"/>
        <child link="upper_leg_link"/>
        <origin xyz="0 0 -0.05" rpy="0 0 0"/>
        <axis xyz="1 0 0"/>
        <limit lower="${hip_limit_lower}" upper="${hip_limit_upper}" effort="${hip_effort}" velocity="${hip_velocity}"/>
      </joint>

      <!-- Upper Leg Link -->
      <link name="upper_leg_link">
        <visual>
          <geometry>
            <cylinder radius="0.05" length="0.4"/>
          </geometry>
          <origin xyz="0 0 -0.2" rpy="0 0 0"/>
          <material name="blue">
            <color rgba="0.0 0.0 0.8 1.0"/>
          </material>
        </visual>
        <collision>
          <geometry>
            <cylinder radius="0.05" length="0.4"/>
          </geometry>
          <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        </collision>
        <inertial>
          <mass value="2.0"/>
          <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
        </inertial>
      </link>

      <!-- Material definitions -->
      <material name="grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>

    </robot>
    ```
3.  **Update `display_single_leg.launch.py`**:
    Change the `PathJoinSubstitution` for the URDF file to point to `.urdf.xacro`:
    ```python
    from launch import LaunchDescription
    from launch_ros.actions import Node
    from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
    from launch_ros.parameter_descriptors import ParameterValue
    from launch_ros.substitutions import FindPackageShare

    def generate_launch_description():
        robot_description_content = ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name="xacro")]) , " ",
                PathJoinSubstitution([
                    FindPackageShare("my_humanoid_description"), "urdf", "single_leg.urdf.xacro"
                ])
            ]),
            value_type=str
        )

        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description_content}],
            output="screen"
        )

        joint_state_publisher_gui_node = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen"
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([FindPackageShare("urdf_tutorial"), "rviz", "urdf.rviz"])],
        )

        return LaunchDescription([
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node
        ])
    ```
4.  **Build and Source**, then **Launch the display**:
    The result in RViz should be identical, but the underlying URDF is now more maintainable.

**Common Pitfall**: Forgetting the `xmlns:xacro` declaration or incorrect syntax for property usage (`$ {property_name}`).
**Fix**: Double-check Xacro syntax. Use `ros2 run xacro xacro single_leg.urdf.xacro` to expand the Xacro file into a plain URDF to check for errors.

## Python Interaction with URDF (Parsing and Visualization)

While URDF is XML, Python tools allow for programmatic interaction, which is invaluable for dynamic robot configuration or analysis.

**Lab 3.3: Parsing URDF with Python**

**Goal**: Write a Python script to parse a URDF file and extract information about its links and joints.

1.  **Create a Python script `parse_urdf.py` in `my_humanoid_description/scripts`**:
    ```bash
    mkdir -p ~/ros2_ws/src/my_humanoid_description/scripts
    touch ~/ros2_ws/src/my_humanoid_description/scripts/parse_urdf.py
    ```
    ```python
    import os
    import xml.etree.ElementTree as ET
    from ament_index_python.packages import get_package_share_directory

    def parse_urdf(package_name, urdf_filename):
        try:
            # Get path to the package share directory
            package_share_directory = get_package_share_directory(package_name)
            urdf_path = os.path.join(package_share_directory, 'urdf', urdf_filename)

            if not os.path.exists(urdf_path):
                print(f"Error: URDF file not found at {urdf_path}")
                return

            print(f"Parsing URDF from: {urdf_path}")
            tree = ET.parse(urdf_path)
            root = tree.getroot()

            print(f"Robot Name: {root.attrib.get('name', 'Unnamed Robot')}\n")

            print("--- Links ---")
            for link in root.findall('link'):
                name = link.attrib.get('name')
                inertial = link.find('inertial')
                mass = inertial.find('mass').attrib.get('value') if inertial and inertial.find('mass') else 'N/A'
                print(f"  Link Name: {name}, Mass: {mass} kg")

            print("\n--- Joints ---")
            for joint in root.findall('joint'):
                name = joint.attrib.get('name')
                joint_type = joint.attrib.get('type')
                parent = joint.find('parent').attrib.get('link') if joint.find('parent') else 'N/A'
                child = joint.find('child').attrib.get('link') if joint.find('child') else 'N/A'
                axis = joint.find('axis').attrib.get('xyz') if joint.find('axis') else 'N/A'
                limit = joint.find('limit')
                lower = limit.attrib.get('lower') if limit else 'N/A'
                upper = limit.attrib.get('upper') if limit else 'N/A'

                print(f"  Joint Name: {name}, Type: {joint_type}")
                print(f"    Parent: {parent}, Child: {child}, Axis: {axis}")
                print(f"    Limits: Lower={lower}, Upper={upper}")
                print("-" * 20)

        except Exception as e:
            print(f"An error occurred: {e}")

if __name__ == '__main__':
    # Ensure to use the correct package name and URDF filename
    parse_urdf('my_humanoid_description', 'single_leg.urdf.xacro')
    ```
2.  **Update `setup.py` (for ament_python package that uses this script)**:
    If this script were part of a Python package, you'd add it to `entry_points`. For now, you can just run it as a standalone Python script.
3.  **Build and Source**:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```
4.  **Run the script**:
    ```bash
    python3 ~/ros2_ws/src/my_humanoid_description/scripts/parse_urdf.py
    ```
    This will output the parsed link and joint information to the console.

**Common Pitfall**: Incorrect path to the URDF file, especially when using `get_package_share_directory`.
**Fix**: Verify the `package_name` and `urdf_filename` are correct and that the file actually exists at the constructed path.

## Lab 3.4: Dynamic URDF Generation with Python and Xacro

**Goal**: Use Python to dynamically generate a URDF based on configuration parameters, then visualize it. This is powerful for robots with modular components or configurable dimensions.

1.  **Create `dynamic_robot.xacro` in `urdf` directory**:
    This xacro will take arguments and define a simple robotic arm with configurable length.
    ```xml
    <?xml version="1.0"?>
    <robot name="dynamic_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

      <xacro:arg name="arm_length" default="0.3"/>
      <xacro:property name="arm_length_val" value="$(arg arm_length)"/>

      <link name="base_link">
        <visual>
          <geometry><box size="0.1 0.1 0.05"/></geometry>
          <material name="grey"><color rgba="0.7 0.7 0.7 1.0"/></material>
        </visual>
      </link>

      <joint name="shoulder_joint" type="revolute">
        <parent link="base_link"/>
        <child link="arm_link"/>
        <origin xyz="0 0 0.025" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-${M_PI/4}" upper="${M_PI/4}" effort="100.0" velocity="1.0"/>
      </joint>

      <link name="arm_link">
        <visual>
          <geometry><cylinder radius="0.02" length="${arm_length_val}"/></geometry>
          <origin xyz="0 0 ${arm_length_val/2}" rpy="0 0 0"/>
          <material name="blue"><color rgba="0.0 0.0 0.8 1.0"/></material>
        </visual>
      </link>

      <material name="grey"><color rgba="0.7 0.7 0.7 1.0"/></material>
      <material name="blue"><color rgba="0.0 0.0 0.8 1.0"/></material>

    </robot>
    ```
2.  **Create a Python launch file `display_dynamic_robot.launch.py` in `launch` directory**:
    This launch file will pass arguments to the xacro.
    ```python
    from launch import LaunchDescription
    from launch_ros.actions import Node
    from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, TextSubstitution
    from launch_ros.parameter_descriptors import ParameterValue
    from launch_ros.substitutions import FindPackageShare
    from launch.actions import DeclareLaunchArgument
    from launch.conditions import IfCondition
    from launch.substitutions import LaunchConfiguration

    def generate_launch_description():
        declared_arguments = []
        declared_arguments.append(
            DeclareLaunchArgument(
                "arm_length",
                default_value="0.5", # Default arm length
                description="Length of the robot arm in meters.",
            )
        )
        declared_arguments.append(
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Whether to start RViz2.",
            )
        )

        arm_length = LaunchConfiguration("arm_length")
        use_rviz = LaunchConfiguration("use_rviz")

        robot_description_content = ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name="xacro")]) , " ",
                PathJoinSubstitution([
                    FindPackageShare("my_humanoid_description"), "urdf", "dynamic_robot.xacro"
                ]),
                " arm_length:=", arm_length, # Pass argument to xacro
            ]),
            value_type=str
        )

        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description_content}],
            output="screen"
        )

        joint_state_publisher_gui_node = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen"
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([FindPackageShare("urdf_tutorial"), "rviz", "urdf.rviz"])],
            condition=IfCondition(use_rviz)
        )

        return LaunchDescription(declared_arguments + [
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node
        ])
    ```
3.  **Build and Source**:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```
4.  **Launch with different arm lengths**:
    ```bash
    ros2 launch my_humanoid_description display_dynamic_robot.launch.py arm_length:=0.2
    ros2 launch my_humanoid_description display_dynamic_robot.launch.py arm_length:=0.8
    ```
    Observe how the robot arm's length changes in RViz.

**Common Pitfall**: Incorrectly passing arguments to xacro or misinterpreting units (meters vs. other).
**Fix**: Verify `xacro` command line syntax. Always define and use consistent units.

## Common Pitfalls and Fixes in URDF for Humanoids

*   **Mass and Inertia**: Incorrect values lead to unrealistic simulation behavior and unstable control.
    *   **Fix**: Use CAD tools to calculate accurate inertial properties. For simple shapes, use standard formulas.
*   **Collision Geometries**: Using complex visual meshes for collision can be computationally expensive and unstable. Not defining collisions leads to robots passing through objects.
    *   **Fix**: Simplify collision geometries (boxes, cylinders, spheres) to approximate the visual mesh. Ensure collision models cover the robot's physical extent.
*   **Joint Limits**: Missing or incorrect joint limits can lead to unnatural robot poses or self-collision.
    *   **Fix**: Define realistic `lower` and `upper` limits for all revolute and prismatic joints, reflecting the physical constraints of the robot.
*   **`origin` and Coordinate Frames**: A fundamental source of errors. Links might appear disconnected or misaligned.
    *   **Fix**: Visualize coordinate frames in RViz. The `origin` tag defines the pose of the child link's frame relative to the parent link's frame. Pay close attention to `xyz` (translation) and `rpy` (roll, pitch, yaw rotation) values.
*   **Self-Collision (especially for humanoids)**: Due to high DOF and complex geometry, humanoids are prone to self-collision.
    *   **Fix**: Define collision meshes carefully. Use `<disable_collisions>` tags in URDF to explicitly ignore collisions between adjacent links that are always connected and should not collide (e.g., upper arm and forearm).

## Student Exercises (with Hidden Solutions)

**Exercise 3.1: Add a Knee Joint**
Extend the `single_leg.urdf.xacro` from Lab 3.2 to include a knee joint and a lower leg link. The knee joint should be a `revolute` joint, rotating about the X-axis, with appropriate `origin` and `limit` values.

<details>
  <summary>Solution (click to expand)</summary>

**Modified `single_leg.urdf.xacro`**:
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid_leg" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Xacro Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="hip_limit_lower" value="${-M_PI/2}" />
  <xacro:property name="hip_limit_upper" value="${M_PI/2}" />
  <xacro:property name="hip_effort" value="10.0" />
  <xacro:property name="hip_velocity" value="1.0" />

  <xacro:property name="knee_limit_lower" value="0.0" />          <!-- Can't bend backward -->
  <xacro:property name="knee_limit_upper" value="${M_PI/2}" />  <!-- 90 degrees forward bend -->
  <xacro:property name="knee_effort" value="10.0" />
  <xacro:property name="knee_velocity" value="1.0" />

  <!-- Base Link (Root of the leg chain) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.05"/>
      </geometry>
      <material name="grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Hip Joint -->
  <joint name="hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_leg_link"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${hip_limit_lower}" upper="${hip_limit_upper}" effort="${hip_effort}" velocity="${hip_velocity}"/>
  </joint>

  <!-- Upper Leg Link -->
  <link name="upper_leg_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Knee Joint (new) -->
  <joint name="knee_joint" type="revolute">
    <parent link="upper_leg_link"/>
    <child link="lower_leg_link"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/> <!-- Connects to the bottom of the upper leg link -->
    <axis xyz="1 0 0"/> <!-- Rotation around X-axis -->
    <limit lower="${knee_limit_lower}" upper="${knee_limit_upper}" effort="${knee_effort}" velocity="${knee_velocity}"/>
  </joint>

  <!-- Lower Leg Link (new) -->
  <link name="lower_leg_link">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/> <!-- Center of cylinder at origin -->
      <material name="green">
        <color rgba="0.0 0.8 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Material definitions -->
  <material name="grey">
    <color rgba="0.7 0.7 0.7 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>

</robot>
```
Remember to rebuild and relaunch the display.
</details>

**Exercise 3.2: Sensor Integration**
Add a simple camera sensor to the `base_link` of the `single_leg.urdf.xacro` (or the complete leg from Exercise 3.1). The camera should be a simple box visual element.

<details>
  <summary>Solution (click to expand)</summary>

**Modified `single_leg.urdf.xacro` (adding camera to `base_link`)**:
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid_leg" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Xacro Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="hip_limit_lower" value="${-M_PI/2}" />
  <xacro:property name="hip_limit_upper" value="${M_PI/2}" />
  <xacro:property name="hip_effort" value="10.0" />
  <xacro:property name="hip_velocity" value="1.0" />

  <xacro:property name="knee_limit_lower" value="0.0" />
  <xacro:property name="knee_limit_upper" value="${M_PI/2}" />
  <xacro:property name="knee_effort" value="10.0" />
  <xacro:property name="knee_velocity" value="1.0" />

  <!-- Base Link (Root of the leg chain) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.05"/>
      </geometry>
      <material name="grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Camera Joint (new) -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.05 0 0.025" rpy="0 0 0"/> <!-- Mounted on top-front of base_link -->
  </joint>

  <!-- Camera Link (new) -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.03 0.03 0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Hip Joint -->
  <joint name="hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_leg_link"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${hip_limit_lower}" upper="${hip_limit_upper}" effort="${hip_effort}" velocity="${hip_velocity}"/>
  </joint>

  <!-- Upper Leg Link -->
  <link name="upper_leg_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Knee Joint -->
  <joint name="knee_joint" type="revolute">
    <parent link="upper_leg_link"/>
    <child link="lower_leg_link"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${knee_limit_lower}" upper="${knee_limit_upper}" effort="${knee_effort}" velocity="${knee_velocity}"/>
  </joint>

  <!-- Lower Leg Link -->
  <link name="lower_leg_link">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0.0 0.8 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Material definitions -->
  <material name="grey">
    <color rgba="0.7 0.7 0.7 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.1 0.1 0.1 1.0"/>
  </material>

</robot>
```
Remember to rebuild and relaunch the display.
</details>

## Further Reading and Official Resources

*   **URDF Documentation**: The definitive guide to the URDF format.
    *   [ROS 2 URDF Overview](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
    *   [URDF XML Specification](http://wiki.ros.org/urdf/XML)
*   **Xacro Documentation**: For writing modular and parameterized URDFs.
    *   [ROS 2 Xacro Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html)
*   **RViz2**: The primary visualization tool for ROS 2.
    *   [RViz2 User Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Viewing-a-URDF-Model.html)
*   **Gazebo**: For simulating the robot's physical interactions.
    *   [Gazebo Tutorials](http://classic.gazebosim.org/tutorials)

Understanding and effectively utilizing URDF is a critical skill for any roboticist, especially those working with complex humanoid platforms. It forms the digital foundation upon which all simulation, control, and interaction capabilities are built. With Xacro, this foundation becomes robust, flexible, and scalable, ready to accommodate the intricate designs of advanced Physical AI systems. The next module will delve into simulating these digital twins in high-fidelity environments.

---

[**← Previous: Bridging Python Agents to ROS 2**](./bridging-python-agents-to-ros2.md) | [**Next: Module 2: The Digital Twin →**](../module-2/simulating-physics-in-gazebo.md)