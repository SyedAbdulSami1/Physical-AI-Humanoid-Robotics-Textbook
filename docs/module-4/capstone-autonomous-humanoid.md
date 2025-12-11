---
sidebar_position: 3
---

# Capstone Project: The Autonomous Humanoid

This is it. The culmination of everything we have learned. In this capstone project, you will bring together all four modules—ROS 2, Digital Twins, Isaac Sim, and VLA models—to create a fully autonomous humanoid robot that can understand a spoken command, perceive its environment, plan a complex task, and execute it with physical manipulation.

**The Mission Briefing:**
Your robot will be in a simulated room containing several objects and a target bin. You will give it a high-level voice command, such as:

> *"Hey robot, please find the red block and put it in the storage bin."

The robot must then autonomously perform the entire sequence:
1.  **Listen & Transcribe**: Use a microphone and the Whisper node to hear and transcribe the command.
2.  **Perceive & Localize**: Use its sensors (camera, LiDAR) to identify the objects in the room and determine their locations.
3.  **Plan**: Use an LLM to parse the transcribed command and generate a sequence of actions based on the perceived world state.
4.  **Execute**: Execute the action plan, which will involve navigation (moving to the object), manipulation (picking it up), further navigation (moving to the bin), and final manipulation (dropping it).

This project will challenge you to integrate dozens of different nodes, services, and concepts into a single, coherent system.

## System Architecture

The final system is a beautiful, complex dance of distributed ROS 2 nodes. Understanding the architecture is key.

```mermaid
graph TD
    subgraph Human Interaction
        A[Human Voice] --> B[Microphone];
    end
    
    subgraph ROS 2 VLA Pipeline
        B -- Audio --> C[Whisper Node<br/>(voice_agent_py)];
        C -- Text: "/transcribed_text" --> D[LLM Planner Node<br/>(voice_agent_py)];
    end
    
    subgraph ROS 2 Perception & World Model
        G[Isaac Sim<br/>(Camera, LiDAR)] -- Sensor Data --> H{Perception System};
        H -- Object Detections --> I[World Model Server];
    end
    
    subgraph ROS 2 Control & Execution
        J[Action Executor Node] -- Action Status --> D;
        K[Nav2 Stack] -- Navigation Control --> L[Robot Controller];
        M[Manipulation Controller<br/>(e.g., MoveIt)] -- Arm Control --> L;
    end
    
    D -- Queries World State --> I;
    D -- JSON Plan: "/robot_action_plan" --> J;
    
    J -- Dispatches Goals --> K;
    J -- Dispatches Goals --> M;
    
    L -- Low-level commands --> F[Simulated Humanoid<br/>(Isaac Sim)];

    style C fill:#f9f,stroke:#333,stroke-width:2px
    style D fill:#f9f,stroke:#333,stroke-width:2px
    style J fill:#ccf,stroke:#333,stroke-width:2px
    style H fill:#cfc,stroke:#333,stroke-width:2px
    style L fill:#fcf,stroke:#333,stroke-width:2px
```
**Key New Components for this Project**:
-   **Perception System**: A node (or set of nodes) that subscribes to camera feeds and uses a trained computer vision model (like YOLO or a model trained on Isaac Sim's synthetic data) to detect objects and their positions.
-   **World Model Server**: A simple ROS 2 service that stores the current state of the world (e.g., `{"red_block": {"position": [x, y, z]}}`). The LLM Planner will query this service to get the necessary context for its prompt.
-   **Action Executor Node**: This node subscribes to the JSON plan from the LLM. It acts as a state machine, parsing the plan and making the appropriate calls to other ROS 2 systems (like Nav2 and MoveIt) one action at a time.
-   **Manipulation Controller (MoveIt)**: For a real humanoid, you would use a motion planning framework like MoveIt to control the arm. MoveIt can take a target pose for the end-effector (the hand) and generate a collision-free trajectory for all the arm joints.

---
## Project Repository Structure

A project of this complexity requires a well-organized repository. Here is a recommended structure for your ROS 2 workspace (`ros2_ws/src`).

```
ros2_ws/src/
├── humanoid_description/      # URDF/XACRO for the robot
│   ├── launch/
│   ├── urdf/
│   └── worlds/
├── voice_agent_py/            # Our Whisper and LLM nodes
│   ├── package.xml
│   ├── setup.py
│   └── voice_agent_py/
│       ├── whisper_node.py
│       └── llm_planner_node.py
├── perception_py/             # New package for object detection
│   ├── package.xml
│   ├── setup.py
│   └── perception_py/
│       └── yolo_detector_node.py
├── world_model_py/            # New package for the world state service
│   ├── package.xml
│   ├── setup.py
│   └── world_model_py/
│       └── world_model_server.py
└── action_executor_py/        # New package for the executor
    ├── package.xml
    ├── setup.py
    └── action_executor_py/
        └── executor_node.py
```

---
## Implementation Plan: Step-by-Step

This is a large project. Tackle it in these discrete, testable steps.

### Step 1: The Perception and World Model
Before the robot can plan, it needs to see.
1.  **Create the `perception_py` package.**
2.  **Write the `yolo_detector_node.py`**. This node will:
    -   Subscribe to an image topic from the simulator (e.g., `/camera/image_raw`).
    -   Use a pre-trained object detection model (YOLO is a great choice) to find objects in the image.
    -   For each detected object, it will publish a custom message (e.g., `vision_msgs/Detection3D`) containing the object's class name and its position. (For simplicity, you can initially "cheat" and get the 3D position directly from the simulator's ground truth data).
3.  **Create the `world_model_py` package.**
4.  **Write the `world_model_server.py`**. This node will:
    -   Subscribe to the object detection topic from the perception node.
    -   Store the detected objects and their latest known positions in a Python dictionary.
    -   Provide a ROS 2 Service (e.g., `srv/GetWorldState`) that, when called, returns the entire dictionary as a JSON string.

### Step 2: Enhancing the LLM Planner
Modify the `llm_planner_node.py` from the previous chapter.
1.  Before querying the LLM, it must first call the `/get_world_state` service.
2.  It will then dynamically construct the system prompt, injecting the real-time world state received from the service.

**Modified System Prompt Logic**:
```python
# In llm_planner_node.py

# ... call the service to get world_state_json ...
world_state = json.loads(world_state_json)

# Dynamically build the world description part of the prompt
world_description = "The current state of the world is:\n"
for obj, details in world_state.items():
    world_description += f"- A '{obj}' is at '{details['location']}'.\n"

# The full system prompt is now dynamic!
self.system_prompt = f"""
    You are a helpful robot assistant... 
    (the rest of the prompt)
    {world_description}
    Given the user's command...
"""
```

### Step 3: The Action Executor
This is the heart of the robot's execution logic.
1.  **Create the `action_executor_py` package.**
2.  **Write the `executor_node.py`**. This node is a state machine.
    -   It subscribes to `/robot_action_plan`.
    -   When it receives a plan, it stores it in a queue.
    -   It processes one action from the queue at a time.
    -   Based on the action name (`go_to`, `pick_up`, etc.), it makes the appropriate **ROS 2 Action Client** call.
        -   `go_to`: Call the `/navigate_to_pose` action on the Nav2 stack.
        -   `pick_up`: Call a `/pickup_object` action on a (simulated) manipulation controller.
        -   `drop_in`: Call a `/drop_object` action.
    -   **Crucially**, it must wait for the result of one action to be successful before starting the next one in the sequence. This is the primary use case for ROS 2 Actions over Services.

**Simplified Executor Logic**:
```python
# In executor_node.py

class ExecutorNode(Node):
    def plan_callback(self, msg):
        self.plan = json.loads(msg.data)["plan"]
        self.execute_next_action()

    def execute_next_action(self):
        if not self.plan:
            self.get_logger().info("Plan complete!")
            return
        
        action = self.plan.pop(0)
        action_name = action["action"]
        params = action["parameters"]
        
        self.get_logger().info(f"Executing action: {action_name} with params: {params}")

        if action_name == "go_to":
            # ... create a Nav2 goal message ...
            # ... call self.nav2_action_client.send_goal_async(...) ...
            # ... add a callback for when the goal is done ...
            pass # The callback will call execute_next_action()
        elif action_name == "pick_up":
            # ... call manipulation action client ...
            pass # Its callback will call execute_next_action()
```

### Step 4: Putting It All Together
The final step is to create a master launch file that starts every single node in the correct order with the correct parameters.

**File**: `humanoid_description/launch/capstone.launch.py`
```python
from launch import LaunchDescription
from launch_ros.actions import Node
# ... other launch imports

def generate_launch_description():
    return LaunchDescription([
        # --- Simulation ---
        # Launch Isaac Sim with your world and robot
        
        # --- Voice & Planning ---
        Node(package='voice_agent_py', executable='whisper_node', name='whisper_node'),
        Node(package='voice_agent_py', executable='llm_planner_node', name='llm_planner_node'),
        
        # --- Perception & World State ---
        Node(package='perception_py', executable='yolo_detector_node', name='yolo_detector_node'),
        Node(package='world_model_py', executable='world_model_server', name='world_model_server'),
        
        # --- Execution ---
        Node(package='action_executor_py', executable='executor_node', name='executor_node'),
        
        # --- Robot Control ---
        # Include the launch file for Nav2
        # Include the launch file for MoveIt (or your dummy manipulation controller)
    ])
```

---
## Video Demo Instructions

Creating a compelling video demo is as important as the project itself.
1.  **The "One-Shot" Take**: Start by recording your screen showing the Isaac Sim viewport and the RViz window side-by-side.
2.  **Start all systems**: Run your master launch file.
3.  **Speak the Command**: Clearly speak the full command to your microphone (e.g., "Hey robot, get the red block and place it in the bin").
4.  **Show the Pipeline**: In your recording, use your mouse to point to the different terminal windows, showing:
    -   The Whisper node transcribing the text.
    -   The LLM planner node logging the query and the JSON plan it received.
    -   The action executor logging that it's starting the first action.
5.  **Focus on the Simulation**: Maximize the Isaac Sim and RViz windows. Show the robot navigating to the first location. Show the path being drawn in RViz.
6.  **Show Manipulation**: Show the robot's arm moving to grasp the object. (In a simplified simulation, the object might just snap to the hand).
7.  **Show the Second Navigation**: Show the robot navigating to the bin.
8.  **Show the Final Action**: Show the robot dropping the object into the bin.
9.  **The Victory Pose**: End the video with the robot returning to its home base or waving.

## Grading Rubric

This project will be evaluated on the successful integration and functionality of the complete pipeline.

| Criteria (Total 100 points) | Incomplete (0-9 pts) | Partial (10-17 pts) | Complete (18-25 pts) |
| :--- | :--- | :--- | :--- |
| **1. Voice Recognition** | Node crashes or fails to transcribe. | Transcribes inaccurately or unreliably. | Whisper node accurately and reliably transcribes spoken commands into text and publishes them. |
| **2. Perception & World Model** | Perception node fails to run or objects are not detected. | World model is hardcoded and not based on perception data. | Perception node detects objects and the world model service is updated and serves the world state correctly. |
| **3. LLM Task Planning** | Planner node fails to query the LLM or crashes on response. | LLM plan is not valid JSON or hallucinates invalid actions. | LLM node successfully queries the LLM with a dynamic prompt and publishes a valid, executable JSON plan. |
| **4. Action Execution & Navigation** | Executor node fails to parse the plan. Robot does not move. | Executor runs only the first action or robot gets stuck during navigation. | The robot successfully executes the full sequence of actions from the plan, navigating correctly via Nav2 calls. |

---
This capstone is your grand finale. It is difficult, and it will require significant debugging. But successfully completing it means you have mastered the foundational principles of modern, AI-driven humanoid robotics. Good luck.

