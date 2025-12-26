---
sidebar_position: 2
---

# Vision-Language-Action: Cognitive Planning with LLMs

We have successfully converted a spoken command into text. Now comes the "cognitive" part of our VLA pipeline. How does a robot understand the *intent* behind a command like "Clean the room" and translate that high-level goal into a concrete sequence of physical actions? This is where Large Language Models (LLMs) like Google's Gemini come into play.

An LLM can act as the robot's "cognitive planner" or "task broker." By providing the LLM with a carefully crafted prompt—describing the robot's capabilities and the current state of the world—we can ask it to break down a complex command into a series of simpler, executable steps.

This chapter demonstrates how to create a ROS 2 node that takes transcribed text, queries an LLM to get a sequence of actions, and then publishes those actions for the robot's control system to execute.

## The LLM as a Task Planner

The core idea is to treat the LLM as a function call.
`plan = LLM(robot_abilities, world_state, user_command)`

-   **Robot Abilities**: This is a description of the actions the robot knows how to perform. We will define these as simple, callable functions (e.g., `go_to(location)`, `pick_up(object)`, `drop_in(bin)`).
-   **World State**: This is a description of the current environment, including the objects and their locations. In a real system, this would come from a perception module.
-   **User Command**: The transcribed text from our Whisper node.

The "plan" that the LLM returns should be a structured format, like JSON, that represents a sequence of function calls.

**Example Interaction**:
1.  **System Prompt to LLM**:
    > You are a helpful robot assistant. You can perform the following actions: `go_to(location)`, `pick_up(object)`, `drop_in(bin)`. The world contains: a 'toy_car' at 'location_A', a 'red_ball' at 'location_B', and a 'toy_bin' at 'location_C'. Given the user's command, generate a JSON plan of actions.

2.  **User Command**:
    > "Please pick up the red ball and put it in the toy bin."

3.  **LLM's JSON Output**:
    ```json
    [
      {"action": "go_to", "parameters": {"location": "location_B"}},
      {"action": "pick_up", "parameters": {"object": "red_ball"}},
      {"action": "go_to", "parameters": {"location": "location_C"}},
      {"action": "drop_in", "parameters": {"bin": "toy_bin"}}
    ]
    ```
This structured output can then be easily parsed and executed by our ROS 2 system.

```mermaid
graph TD
    A[Transcribed Text: "Clean the room"] --> B{LLM Planner Node};
    subgraph LLM Planner Node
        C[System Prompt]
        D[LLM API Call]
        B -- Forms Prompt --> C;
        C -- Sends to LLM --> D;
        D -- Returns JSON --> B;
    end
    B -- Publishes Plan --> E((/robot_action_plan [JSON]));
    E --> F[Action Executor Node];
```
---

## Lab: Building an LLM Planner Node

This lab will create the `llm_planner_node`. It will subscribe to the `/transcribed_text` topic from our Whisper node, make an API call to an LLM, and publish the resulting JSON plan.

**Prerequisites**:
-   An API key for a Google AI (Gemini).
-   The `google-genai` Python package: `pip install google-genai`.
-   A running `whisper_node` from the previous lab.

### Step 1: Create the Planner Node
In your `voice_agent_py` package, create a new file for the planner node.

**File**: `voice_agent_py/voice_agent_py/llm_planner_node.py`
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import google.genai as genai
import json
import os

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')
        self.subscription = self.create_subscription(
            String, '/transcribed_text', self.command_callback, 10)
            
        self.plan_publisher = self.create_publisher(String, '/robot_action_plan', 10)
        
        # --- LLM Setup ---
        # IMPORTANT: Set your API key in your environment variables
        # export GEMINI_API_KEY='your_key_here'
        try:
            genai.configure(api_key=os.environ["GEMINI_API_KEY"])
            self.model = genai.GenerativeModel('gemini-pro')
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Gemini client: {e}")
            self.get_logger().error("Please make sure the GEMINI_API_KEY environment variable is set.")
            return

        # This is the "System Prompt" that defines the robot's world and abilities
        self.system_prompt = """
You are a helpful robot assistant named 'Friday'. Your goal is to translate a user's natural language command into a structured JSON plan.

You can perform the following actions:
- `go_to(location)`: Navigate to a specific, named location.
- `pick_up(object_name)`: Pick up a specific, named object.
- `drop_in(bin_name)`: Drop the currently held object into a named bin.

The current state of the world is:
- A 'red_block' is at 'table_a'.
- A 'blue_sphere' is at 'table_b'.
- The 'storage_bin' is at 'corner_c'.
- The robot's 'home_base' is at 'location_d'.

Given the user's command, you must generate a JSON array of action objects. Each object must have an "action" and a "parameters" field. Do not include any actions that are not in the list above. Only respond with the raw JSON array, with no other text or explanation.
"""
        self.get_logger().info('LLM Planner Node has started.')

    def command_callback(self, msg):
        user_command = msg.data
        self.get_logger().info(f"Received user command: '{user_command}'")
        self.get_logger().info("Querying LLM for an action plan...")
        
        try:
            full_prompt = self.system_prompt + "\nUser Command: " + user_command
            response = self.model.generate_content(full_prompt)
            
            # Extract the JSON part of the response
            json_plan_str = response.text.strip()
            # Handle cases where the model might still wrap the JSON in markdown
            if json_plan_str.startswith("```json"):
                json_plan_str = json_plan_str[7:-4].strip()

            self.get_logger().info(f"Received JSON plan from LLM: {json_plan_str}")
            
            # Validate that the response is valid JSON
            try:
                json.loads(json_plan_str) # Just to check for errors
                plan_msg = String()
                plan_msg.data = json_plan_str
                self.plan_publisher.publish(plan_msg)
                self.get_logger().info("Published valid JSON plan to /robot_action_plan.")
            except json.JSONDecodeError:
                self.get_logger().error(f"LLM returned invalid JSON: {json_plan_str}")

        except Exception as e:
            self.get_logger().error(f"An error occurred while querying the LLM: {e}")


def main(args=None):
    rclpy.init(args=args)
    llm_planner_node = LLMPlannerNode()
    rclpy.spin(llm_planner_node)
    llm_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
**Code Breakdown**:
1.  **System Prompt**: This is the most important part of the code. It sets the context for the LLM, constraining its possible outputs and defining the "API" (the robot's abilities) it can use. The instruction `Only respond with the raw JSON array` is critical for getting clean, parsable output.
2.  **Gemini Client**: We initialize the Gemini client using the `google.generativeai` library.
3.  **API Call**: We call the `generate_content` method with our combined system and user prompt.
4.  **Response Parsing**: Gemini models don't have a strict JSON-only output mode like some other APIs, so we add a simple check to strip markdown formatting (` ```json ... ``` `) if the model includes it.
5.  **Publishing**: The raw JSON string from the LLM's response is published to the `/robot_action_plan` topic.

### Step 2: Add Entry Point and Build
Add the `llm_planner_node` to your `setup.py` and rebuild your workspace.

### Step 3: Run the Full Pipeline
1.  **Set your API key**: `export GEMINI_API_KEY='your_key_here'`.
2.  **Terminal 1**: Run the Whisper node.
    ```bash
    ros2 run voice_agent_py whisper_node
    ```
3.  **Terminal 2**: Run the LLM Planner node.
    ```bash
    ros2 run voice_agent_py llm_planner_node
    ```
4.  **Terminal 3**: Listen to the final plan output.
    ```bash
    ros2 topic echo /robot_action_plan
    ```
5.  **Speak a command**: "Could you please take the red block and put it in the storage bin."

You should see the Whisper node transcribe your speech, then the LLM node will receive it, query the API, and finally, you'll see the structured JSON plan appear in Terminal 3.

**Example Output on `/robot_action_plan`**:
```
data: '[{"action": "go_to", "parameters": {"location": "table_a"}}, {"action": "pick_up", "parameters": {"object": "red_block"}}, {"action": "go_to", "parameters": {"location": "corner_c"}}, {"action": "drop_in", "parameters": {"bin": "storage_bin"}}]'
```

## Common Pitfalls

1.  **Pitfall**: The LLM output is not valid JSON or includes extra text.
    *   **Cause**: The prompt was not constrained enough. Gemini models can sometimes add explanatory text.
    *   **Fix**: Emphasize the need for raw JSON in your system prompt. The code includes a basic cleanup for markdown, but you might need more robust parsing to find the JSON block if the model is particularly verbose.

2.  **Pitfall**: The LLM "hallucinates" actions or objects that don't exist.
    *   **Cause**: The user's command was ambiguous, or the LLM is being too "creative."
    *   **Fix**: Make your system prompt more strict. You can add a line like "If the user's command cannot be fulfilled with the available actions and objects, respond with an empty JSON array `[]`." After parsing the JSON, your Python code should always validate that the action and its parameters are valid before attempting execution.

## Student Exercises

<details>
<summary>Exercise 1: Add a New Robot Ability</summary>
<div>
**Task**: Give your robot a new ability: `wave(hand)`. Update the system prompt to include this new action (e.g., it can wave with its "left_hand" or "right_hand"). Then, give it a voice command like "Hello robot, please wave your left hand" and verify that the correct JSON plan is generated.

**Solution Steps**:
1. In the `system_prompt` string in `llm_planner_node.py`, add `- wave(hand)` to the list of actions.
2. You might also want to add to the world state: "The robot has a 'left_hand' and a 'right_hand'".
3. Relaunch the node and give it the new command. The expected output would be `[{"action": "wave", "parameters": {"hand": "left_hand"}}]`.
</div>
</details>

<details>
<summary>Exercise 2: Create a Dummy Action Executor</summary>
<div>
**Task**: Create a third node, `action_executor_node.py`, that subscribes to `/robot_action_plan`. This node will parse the JSON and "execute" the plan by printing what it's doing.

**Solution Steps**:
1. Create the new node file.
2. Subscribe to `/robot_action_plan`.
3. In the callback, use `json.loads(msg.data)` to parse the JSON string into a Python dictionary.
4. Loop through the list of actions in the plan.
5. For each action, use `if/elif` statements to check the action name.
6. Print a descriptive log message, e.g., `self.get_logger().info(f"Executing: Moving to {action['parameters']['location']}")`.
7. Use `time.sleep(2)` after each print statement to simulate the action taking time.
8. This closes the loop on our VLA pipeline, from voice to (simulated) action!
</div>
</details>

## Further Reading
- **Google AI for Developers**: [https://ai.google.dev/](https://ai.google.dev/)
- **JSON as a data interchange format**: [https://www.json.org/json-en.html](https://www.json.org/json-en.html)
- **Prompt Engineering Guide**: [https://www.promptingguide.ai/](https://www.promptingguide.ai/)

---

[**← Previous: Voice-to-Action with Whisper**](./voice-to-action-whisper.md) | [**Next: Capstone Project →**](./capstone-autonomous-humanoid.md)
