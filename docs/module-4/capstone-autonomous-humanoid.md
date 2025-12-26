---
title: 'Capstone: The Autonomous Humanoid'
sidebar_label: 'Capstone Project'
---

# Module 4 Capstone: The Autonomous Humanoid

Welcome to the final and most exciting part of our journey into Physical AI. This capstone project, "The Autonomous Humanoid," is where you will integrate everything you've learned across all four modules. You will build and program a simulated humanoid robot that can understand a natural language voice command, perceive its environment, plan a complex series of actions, navigate through space, and physically interact with an object.

This project represents the culmination of this course, bridging the digital brain (AI) with the physical body (robot) to achieve true embodied intelligence.

## Project Overview

The goal of this project is to create an autonomous system where a humanoid robot in a simulated environment can execute a high-level task given by a human user via voice.

**The Scenario:**

The robot is in a simulated room containing a few objects (e.g., a table, a chair, and a can of soda on the table). The user gives a voice command like: **"Hey robot, please pick up the soda can."**

The robot must then:
1.  **Listen and Understand**: Transcribe the voice command to text.
2.  **Think and Plan**: Use a Large Language Model (LLM) to break down the command into a sequence of executable robotic actions.
3.  **See and Locate**: Visually search the room to find the soda can.
4.  **Walk and Navigate**: Plan a path to the table where the soda can is located and walk there, avoiding obstacles.
5.  **Reach and Grasp**: Use its arm to pick up the soda can.

This end-to-end task demonstrates a complete Vision-Language-Action (VLA) pipeline.

## Core Components & Architecture

Your system will be a distributed network of ROS 2 nodes, each responsible for a specific part of the task.

```mermaid
graph TD
    A[User Voice Command] --> B(Whisper Node: Speech-to-Text);
    B --> C{LLM Planner Node};
    C -- Goal: "Find the soda can" --> D[Perception Node (YOLO)];
    D -- Object Location --> C;
    C -- Goal: "Navigate to [location]" --> E[Navigation Stack (Nav2)];
    E -- Navigation Success --> C;
    C -- Goal: "Grasp at [location]" --> F[Manipulation Stack (MoveIt2)];
    F -- Grasp Success --> C;
    C -- Task Complete --> G(Status Node: Announce Completion);
```

1.  **Voice-to-Action (Speech-to-Text Node)**: A ROS 2 node that uses a speech-to-text library to listen for a voice command from a microphone, transcribe it to text, and publish it to a topic.
2.  **Cognitive Planning (LLM Planner Node)**: The "brain" of the robot. This node subscribes to the transcribed text. It then queries a powerful LLM (like Google's Gemini) with a carefully crafted prompt to generate a step-by-step plan of ROS 2 actions. It acts as a state machine, executing each step of the plan in sequence.
3.  **Perception (Perception Node)**: This node processes images from the robot's head-mounted camera. It uses an object detection model (e.g., YOLOv8) to identify and locate objects in the environment, publishing their 3D coordinates.
4.  **Navigation (Nav2)**: You will configure and launch the standard ROS 2 Navigation stack (Nav2) to handle bipedal path planning and locomotion, enabling the robot to walk to a specified coordinate.
5.  **Manipulation (MoveIt2)**: You will configure and launch the ROS 2 Manipulation stack (MoveIt2) to control the robot's arm, enabling it to plan and execute a grasp on the target object.

## Recommended Repository Structure

A clean and organized repository is crucial. Use the following structure for your ROS 2 workspace:

```
autonomous_humanoid_ws/
├── src/
│   ├── humanoid_bringup/
│   │   ├── launch/
│   │   │   └── capstone_project.launch.py
│   │   ├── worlds/
│   │   │   └── aihome.world
│   │   └── rviz/
│   │       └── humanoid_config.rviz
│   ├── humanoid_control/
│   │   └── ... (Controller configurations)
│   ├── humanoid_description/
│   │   └── urdf/
│   │       └── humanoid_robot.urdf
│   ├── humanoid_navigation/
│   │   ├── launch/
│   │   │   └── nav2.launch.py
│   │   └── params/
│   │       └── nav2_params.yaml
│   ├── humanoid_manipulation/
│   │   ├── launch/
│   │   │   └── moveit.launch.py
│   │   └── config/
│   │       └── ... (MoveIt2 config files)
│   └── humanoid_ai/
│       ├── package.xml
│       ├── setup.py
│       └── humanoid_ai/
│           ├── llm_planner_node.py
│           ├── perception_node.py
│           └── voice_to_action_node.py
└── README.md
```

## Step-by-Step Implementation Guide

### 1. Environment Setup
Ensure you have a working ROS 2 Humble, Gazebo, and all necessary Python libraries installed.
```bash
# Install key Python libraries
pip install ultralytics google-genai
```

### 2. Robot URDF & Simulation
- **URDF**: Finalize the URDF for your humanoid robot from Module 1. Ensure it has proper joint limits, inertia, and collision models. Add a camera sensor plugin.
- **Gazebo World**: Create a simple Gazebo world (`aihome.world`) with a ground plane, a table, a chair, and a model for the soda can.

### 3. Voice-to-Action Node (`voice_to_action_node.py`)
This node should:
- Initialize `rclpy`.
- Create a publisher for the transcribed text (`/voice_command`).
- Use a suitable speech-to-text library to capture audio and get the transcription.
- Publish the result.

```python
# humanoid_ai/voice_to_action_node.py (Snippet)
# Implement your chosen speech-to-text solution here.
# Example using a generic STT library:
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr # Example library
import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wav
import tempfile
import os

class VoiceToActionNode(Node):
    def __init__(self):
        super().__init__('voice_to_action_node')
        self.publisher_ = self.create_publisher(String, '/voice_command', 10)
        self.get_logger().info('Voice command node started. Listening...')
        self.recognizer = sr.Recognizer()
        self.listen_and_transcribe()

    def listen_and_transcribe(self):
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source)
            self.get_logger().info("Say something!")
            audio = self.recognizer.listen(source)

        try:
            # Example: using Google Web Speech API (requires internet, may have usage limits)
            text = self.recognizer.recognize_google(audio)
            self.get_logger().info(f'Heard: {text}')
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
        except sr.UnknownValueError:
            self.get_logger().warn("Speech Recognition could not understand audio")
        except sr.RequestError as e:
            self.get_logger().error(f"Could not request results from Speech Recognition service; {e}")

# main function and rclpy init/shutdown omitted for brevity
```

### 4. LLM Planner Node (`llm_planner_node.py`)
This is the most complex node. It acts as the brain and coordinates the other components.

```python
# humanoid_ai/llm_planner_node.py (Snippet)
import google.genai as genai
import os

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')
        self.subscription = self.create_subscription(
            String, '/voice_command', self.command_callback, 10)
        # Add clients for Nav2 and MoveIt2 actions
        # Add publisher to send goals
        self.state = "IDLE"
        
        # --- LLM Setup ---
        try:
            genai.configure(api_key=os.environ["GEMINI_API_KEY"])
            self.model = genai.GenerativeModel('gemini-pro')
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Gemini client: {e}")
            self.get_logger().error("Please make sure the GEMINI_API_KEY environment variable is set.")
            return


    def command_callback(self, msg):
        if self.state == "IDLE":
            self.get_logger().info(f'Received command: "{msg.data}"')
            self.state = "PLANNING"
            self.generate_plan(msg.data)

    def generate_plan(self, command):
        prompt = f"""
        You are the cognitive core for a humanoid robot.
        Translate the user's command into a numbered list of robotic actions.
        Available actions:
        - find_object(object_name)
        - go_to(x, y, z)
        - pick_up(object_name)
        - done()

        Command: "{command}"
        Plan:
        """
        # Call Gemini API with this prompt
        response = self.model.generate_content(prompt)

        # For this example, let's hardcode the plan
        plan = [
            "1. find_object('soda_can')",
            "2. go_to(2.5, 1.0, 0.8)", # Assume object location is returned
            "3. pick_up('soda_can')",
            "4. done()"
        ]
        self.execute_plan(plan)

    def execute_plan(self, plan):
        # A state machine to execute each step of the plan
        # This involves calling ROS 2 actions and services
        self.get_logger().info("Executing plan...")
        # ... implementation of plan execution ...
```

### 5. Perception Node (`perception_node.py`)
This node uses a pre-trained YOLO model to find objects.

```python
# humanoid_ai/perception_node.py (Snippet)
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.model = YOLO('yolov8n.pt')  # Load pretrained model
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.object_publisher = self.create_publisher(...) # Custom message for object location

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(cv_image)

        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Get class name
                cls = int(box.cls[0])
                class_name = self.model.names[cls]
                if class_name == 'soda can': # Or whatever YOLO calls it
                    # ... calculate 3D position and publish it
                    self.get_logger().info(f'Found a soda can!')
```

### 6. Main Launch File (`capstone_project.launch.py`)
This file brings everything together: Gazebo, Nav2, MoveIt2, and your custom AI nodes.

```python
# humanoid_bringup/launch/capstone_project.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    humanoid_ai_pkg = get_package_share_directory('humanoid_ai')
    nav_pkg = get_package_share_directory('humanoid_navigation')

    return LaunchDescription([
        # Launch Gazebo, Nav2, MoveIt2...
        # ...
        
        # Launch AI Nodes
        Node(
            package='humanoid_ai',
            executable='voice_to_action_node',
            name='voice_to_action_node',
            output='screen'),
        Node(
            package='humanoid_ai',
            executable='llm_planner_node',
            name='llm_planner_node',
            output='screen'),
        Node(
            package='humanoid_ai',
            executable='perception_node',
            name='perception_node',
            output='screen'),
    ])
```

## Video Demo Instructions

You must create a 3-5 minute video demonstrating your final project. The video should include:
1.  **Introduction (15s)**: Briefly introduce yourself and the project.
2.  **Code Walkthrough (60s)**: Briefly show your repository structure and highlight a key section of your `llm_planner_node.py`.
3.  **Live Demo (2-3 mins)**:
    -   Show the simulation environment.
    -   Start all the nodes.
    -   Clearly record yourself giving the voice command.
    -   Show the robot executing the full sequence: looking for the object, walking to it, and grasping it.
    -   Show the terminal output of your key nodes to illustrate what the robot is "thinking."
4.  **Conclusion (15s)**: Briefly summarize your success and what you learned.

## Grading Rubric

Your project will be evaluated based on the following criteria.

| Category               | Weight | Description                                                                                                  |
| ---------------------- | ------ | ------------------------------------------------------------------------------------------------------------ |
| **Functionality**      | 40%    | The robot successfully completes the entire task from voice command to grasp. The system is robust.          |
| **Code Quality**       | 20%    | Code is clean, well-commented, and follows ROS 2 best practices. The repository is properly structured.        |
| **VLA Implementation** | 20% | The integration of Speech-to-Text, LLM, Perception, Navigation, and Manipulation is seamless and well-architected. |
| **Video Demo**         | 15%    | The demo video is clear, concise, and effectively showcases the project's functionality and your understanding. |
| **Documentation**      | 5%     | The `README.md` file in your workspace clearly explains how to set up and run your project.                    |

---

## Conclusion

Completing this capstone project is a significant achievement. You have not only mastered the individual components of robotics and AI but have successfully woven them together into a coherent, intelligent system capable of operating in a physical (simulated) world. This project is a microcosm of the future of humanoid robotics and serves as a launching pad for your future explorations in the exciting field of Physical AI. Good luck!

---

[**← Previous: Cognitive Planning with LLMs**](./cognitive-planning-llm-to-ros2.md) | [**Next: Weekly Breakdown →**](../weekly-breakdown.md)