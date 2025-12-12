---
sidebar_position: 1
---

# Vision-Language-Action: Voice-to-Action with Whisper

We've reached the final and most exciting frontier of our curriculum: Vision-Language-Action (VLA) models. This is where we fuse the power of large-scale AI models with the physical embodiment of our robot. The goal is to create a robot that can understand natural human instructions, perceive its environment, and take meaningful action—a true cognitive robot.

This chapter focuses on the first step of this pipeline: **voice**. How do we enable a robot to listen to a spoken command and understand what was said? We will use **OpenAI's Whisper**, a state-of-the-art automatic speech recognition (ASR) model, to transcribe spoken audio into text. We will then build a ROS 2 node that can listen to a microphone, use Whisper to transcribe the audio, and publish the resulting text, making the human voice just another sensor in the ROS ecosystem.

## The VLA Pipeline

Before diving in, let's look at the high-level VLA pipeline we'll be building across this module. It's a chain of specialized AI models and robotic components working in concert.

```mermaid
graph TD
    A[Human Voice: "Clean the room"] --> B{Microphone};
    B -- Audio Stream --> C[Whisper ASR Node];
    C -- Transcribed Text --> D[LLM Cognitive Planner Node];
    D -- Sequence of ROS Actions --> E[Robot's Control System];
    subgraph This Chapter
        B; C;
    end
    subgraph Next Chapter
        D;
    end
    E -- Executes Actions --> F[Physical Robot];
```

Our focus here is on the initial stage: converting the continuous audio stream from a microphone into a discrete, textual command.

## Why Whisper?

Whisper is a neural network trained on a massive dataset of diverse audio. This makes it incredibly robust and accurate for speech recognition. Its key features include:
-   **High Accuracy**: It performs at a near-human level of accuracy across a wide range of accents, languages, and background noise conditions.
-   **Multilingual**: It can transcribe audio in dozens of languages and can even translate them into English.
-   **Open Source**: The model and its code are open source, allowing us to run it locally on our own hardware for privacy and low latency.
-   **Simple API**: The Python API for using Whisper is straightforward, making it easy to integrate into our ROS 2 nodes.

For robotics, running Whisper locally on a machine like the NVIDIA Jetson Orin is ideal. It avoids reliance on a cloud service, reducing latency and privacy concerns.

---

## Lab 1: A ROS 2 Node for Real-Time Transcription

In this lab, we'll build a "Whisperer" node. This node will listen to a microphone, accumulate audio until it detects silence, and then run the Whisper model to transcribe the captured audio segment.

**Prerequisites**:
-   A working microphone connected to your machine.
-   The `whisper` Python package installed: `pip install openai-whisper`.
-   Audio libraries: `sudo apt-get install portaudio19-dev python3-pyaudio`.
-   A ROS 2 workspace.

### Step 1: Create the Package and the Node
Create a new package `voice_agent_py` with dependencies on `rclpy` and `std_msgs`.

**File**: `voice_agent_py/voice_agent_py/whisper_node.py`
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import whisper
import numpy as np
import torch
from queue import Queue
import threading
import time

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        self.publisher_ = self.create_publisher(String, '/transcribed_text', 10)
        
        # Whisper model loading
        self.model_size = "base.en" # Use "tiny.en" for smaller devices like Raspberry Pi
        self.get_logger().info(f"Loading Whisper model: {self.model_size}...")
        self.model = whisper.load_model(self.model_size)
        self.get_logger().info("Whisper model loaded.")

        # Speech recognition setup
        self.r = sr.Recognizer()
        self.audio_queue = Queue()
        
        # Start the microphone listener in a separate thread
        self.listener_thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.listener_thread.start()
        
        self.get_logger().info("Whisper node has started and is listening.")

    def listen_loop(self):
        with sr.Microphone() as source:
            self.r.adjust_for_ambient_noise(source)
            self.get_logger().info("Calibrated for ambient noise. Ready to listen.")
            # This function will block until it hears speech and then silence
            self.r.listen_in_background(source, self.audio_callback)
            while rclpy.ok():
                time.sleep(1)

    def audio_callback(self, recognizer, audio):
        self.get_logger().info("Detected speech, adding to queue for processing.")
        self.audio_queue.put(audio)
        # Start processing in a new thread to not block the listener
        threading.Thread(target=self.process_audio_queue).start()

    def process_audio_queue(self):
        if not self.audio_queue.empty():
            audio = self.audio_queue.get()
            self.get_logger().info("Processing audio...")
            
            # Convert audio data to a numpy array that Whisper can process
            audio_data = np.frombuffer(audio.get_raw_data(), np.int16).flatten().astype(np.float32) / 32768.0
            
            # Use Whisper to transcribe
            result = self.model.transcribe(audio_data, fp16=torch.cuda.is_available())
            text = result['text']
            
            self.get_logger().info(f"Whisper transcribed: '{text}'")
            
            # Publish the transcribed text
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    whisper_node = WhisperNode()
    
    # We don't spin here because the background threads are handling everything
    # The main thread can just wait for shutdown
    try:
        while rclpy.ok():
            rclpy.spin_once(whisper_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    
    whisper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
**Code Breakdown**:
1.  **`speech_recognition` Library**: We use this excellent library to handle the low-level microphone access and, crucially, to detect speech and silence.
2.  **`whisper.load_model`**: We load a pre-trained Whisper model. `base.en` is a good balance of speed and accuracy for English.
3.  **Threading**: Microphone listening and audio processing are I/O and CPU-bound tasks. We run them in separate threads to keep the main ROS 2 node responsive.
4.  **`listen_in_background`**: This is a non-blocking function from `speech_recognition` that continuously listens. When it detects a phrase (speech followed by silence), it calls our `audio_callback`.
5.  **`audio_callback`**: This function receives the captured `AudioData` object and puts it into a queue.
6.  **`process_audio_queue`**: This function takes audio from the queue, converts it into a format Whisper understands (a NumPy array of 32-bit floats), and calls `self.model.transcribe()`.
7.  **Publishing**: The transcribed text is then published on the `/transcribed_text` topic for other nodes to use.

### Step 2: Add Entry Point and Build
Add the `whisper_node` to your `setup.py` and build your workspace with `colcon build`.

### Step 3: Run and Test
1.  Source your workspace: `source install/setup.bash`.
2.  Run the node: `ros2 run voice_agent_py whisper_node`.
3.  In another terminal, listen to the output topic: `ros2 topic echo /transcribed_text`.
4.  Speak a command into your microphone, like "Hello robot, please clean the room."
5.  After you stop speaking, you should see the node log that it's processing, and then the transcribed text will appear in the `ros2 topic echo` terminal.

<div align="center">

*Image: Terminal outputs of the Whisper node and ROS 2 topic echo.*
*A screenshot showing two terminals. The left terminal shows the output of the `whisper_node`, with logs like "Whisper transcribed: 'Hello robot, please clean the room.'". The right terminal shows the output of `ros2 topic echo`, displaying the same text as a ROS message.*

</div>

## Common Pitfalls and Best Practices

1.  **Pitfall**: The transcription is inaccurate or picks up a lot of background noise.
    *   **Cause**: Poor microphone quality, a noisy environment, or the ambient noise calibration wasn't effective.
    *   **Fix**: Use a better quality microphone (headsets are often better than built-in laptop mics). Run the node in a quieter room. The `recognizer.adjust_for_ambient_noise(source)` line is important; make sure there is a moment of silence when the node starts so it can calibrate properly.

2.  **Pitfall**: The node is slow to respond.
    *   **Cause**: The Whisper model is computationally intensive. The `base` model can be slow on CPUs.
    *   **Fix**: If you have an NVIDIA GPU, ensure PyTorch is installed with CUDA support (`fp16=torch.cuda.is_available()` will then be true). This dramatically speeds up transcription. If you are on a less powerful device (like a Raspberry Pi), use a smaller model like `tiny.en`.

3.  **Best Practice: Keyword Spotting**: In a real application, you don't want the robot to listen to everything. It should only respond when you say a "wake word" (like "Hey Alexa" or "Okay Google"). You can implement this by using a lightweight keyword spotting library to trigger the full Whisper transcription only after the wake word is detected.

## Student Exercises

<details>
<summary>Exercise 1: Change the Language</summary>
<div>
**Task**: Modify the node to transcribe a different language that you speak.

**Solution Steps**:
1.  Whisper's multilingual models are the ones that don't end in `.en`. In the `WhisperNode`, change `self.model_size` from `"base.en"` to just `"base"`.
2.  When you call the transcribe function, you can provide a language hint: `result = self.model.transcribe(audio_data, language="es")` for Spanish, for example.
3.  Relaunch the node and speak in the specified language.
</div>
</details>

<details>
<summary>Exercise 2: Create a "Confirmation" Node</summary>
<div>
**Task**: Create a second ROS 2 node that subscribes to `/transcribed_text`. If the received text contains the word "robot", this new node should publish a message "I am ready to help" to a `/robot_response` topic.

**Solution Steps**:
1.  Create a new Python script for your `ConfirmationNode`.
2.  In its `__init__`, create a subscriber to `/transcribed_text` and a publisher to `/robot_response`.
3.  In the subscriber's callback function, check if the received message (`msg.data`) contains the substring "robot": `if "robot" in msg.data.lower():`.
4.  If it does, create a `String` message with the confirmation text and publish it.
5.  Run both your `whisper_node` and your new `confirmation_node` and test it. This simple exercise demonstrates the power of chaining ROS 2 nodes together to create more complex behaviors.
</div>
</details>

## Further Reading
- **OpenAI Whisper GitHub**: [https://github.com/openai/whisper](https://github.com/openai/whisper)
- **`speech_recognition` Library PyPI**: [https://pypi.org/project/SpeechRecognition/](https://pypi.org/project/SpeechRecognition/)
- **ROS 2 `std_msgs/String`**: [http://docs.ros.org/en/humble/API/std_msgs/msg/String.html](http://docs.ros.org/en/humble/API/std_msgs/msg/String.html)
- **Picovoice for Wake Word/Keyword Spotting**: A popular library for creating custom wake words. [https://picovoice.ai/](https://picovoice.ai/)

---

[**← Previous: Nav2 for Bipedal Path Planning**](../module-3/nav2-bipedal-path-planning.md) | [**Next: Cognitive Planning with LLMs →**](./cognitive-planning-llm-to-ros2.md)
