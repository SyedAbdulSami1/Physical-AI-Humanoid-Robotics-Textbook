---
sidebar_position: 1
---

# Vision-Language-Action: Voice-to-Action with Speech Recognition

We've reached the final and most exciting frontier of our curriculum: Vision-Language-Action (VLA) models. This is where we fuse the power of large-scale AI models with the physical embodiment of our robot. The goal is to create a robot that can understand natural human instructions, perceive its environment, and take meaningful action—a true cognitive robot.

This chapter focuses on the first step of this pipeline: **voice**. How do we enable a robot to listen to a spoken command and understand what was said? We will explore using a **speech-to-text service**, a state-of-the-art automatic speech recognition (ASR) model, to transcribe spoken audio into text. We will then build a ROS 2 node that can listen to a microphone, use this service to transcribe the audio, and publish the resulting text, making the human voice just another sensor in the ROS ecosystem.

## The VLA Pipeline

Before diving in, let's look at the high-level VLA pipeline we'll be building across this module. It's a chain of specialized AI models and robotic components working in concert.

```mermaid
graph TD
    A[Human Voice: "Clean the room"] --> B{Microphone};
    B -- Audio Stream --> C[Speech-to-Text ASR Node];
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

## Why Speech-to-Text for Robotics?

A robust Automatic Speech Recognition (ASR) system is crucial for enabling robots to understand natural language commands. Key features of an effective ASR system for robotics include:
-   **High Accuracy**: The ability to accurately transcribe speech across various accents, languages, and environmental conditions is paramount for reliable robot operation.
-   **Multilingual Support**: For global applications, the ASR system should ideally support transcription in multiple languages.
-   **Deployment Flexibility**: The option to run the ASR model locally on edge devices (like the NVIDIA Jetson Orin) is often preferred to reduce latency, ensure privacy, and minimize reliance on cloud services. This contrasts with cloud-based solutions, which, while powerful, introduce network dependencies.
-   **Ease of Integration**: A straightforward API or library facilitates seamless integration into existing robotic frameworks, such as ROS 2 nodes.

For educational and research purposes, prioritizing open-source or free-tier solutions that can be run locally is often beneficial, as it provides greater control and avoids recurring costs associated with proprietary cloud services.

---

## Lab 1: A ROS 2 Node for Real-Time Transcription

In this lab, we'll build a "Whisperer" node. This node will listen to a microphone, accumulate audio until it detects silence, and then run the Whisper model to transcribe the captured audio segment.

**Prerequisites**:
-   A working microphone connected to your machine.
-   A Python environment with a suitable speech-to-text library installed (e.g., `SpeechRecognition` combined with a local STT engine, or an API client for a cloud STT service).
-   Audio libraries like `PortAudio` (often needed for `SpeechRecognition`): `sudo apt-get install portaudio19-dev python3-pyaudio`.
-   A ROS 2 workspace.

### Step 1: Implement Your Speech-to-Text ROS 2 Node
Implement a ROS 2 node that utilizes your chosen speech-to-text library or API. This node should:
1.  Listen for audio input from a microphone.
2.  Process the audio to convert speech into text.
3.  Publish the transcribed text to a ROS 2 topic (e.g., `/transcribed_text`).

**(Note: The previous code example using OpenAI Whisper has been removed. You will need to implement this section using a non-OpenAI speech-to-text solution. Consider using `speech_recognition` library with a local engine or a client for a free-tier cloud STT service.)**

### Step 2: Add Entry Point and Build
Add the `whisper_node` to your `setup.py` and build your workspace with `colcon build`.

### Step 3: Run and Test Your Speech-to-Text Node
1.  Source your workspace: `source install/setup.bash`.
2.  Run your speech-to-text node (e.g., `ros2 run voice_agent_py your_stt_node_name`).
3.  In another terminal, listen to the output topic: `ros2 topic echo /transcribed_text`.
4.  Speak a command into your microphone.
5.  You should see the transcribed text appear in the `ros2 topic echo` terminal.

<div align="center">

*Image: Terminal outputs of the Whisper node and ROS 2 topic echo.*
*A screenshot showing two terminals. The left terminal shows the output of the `whisper_node`, with logs like "Whisper transcribed: 'Hello robot, please clean the room.'". The right terminal shows the output of `ros2 topic echo`, displaying the same text as a ROS message.*

</div>

## Common Pitfalls and Best Practices for Speech-to-Text

1.  **Pitfall**: The transcription is inaccurate or picks up a lot of background noise.
    *   **Cause**: Poor microphone quality, a noisy environment, or ineffective ambient noise calibration.
    *   **Fix**: Use a better quality microphone (headsets are often better than built-in laptop mics). Run the node in a quieter room. Ensure your speech recognition library's ambient noise adjustment is functioning correctly.

2.  **Pitfall**: The speech-to-text processing is slow.
    *   **Cause**: The chosen speech-to-text model is computationally intensive, or hardware resources are limited.
    *   **Fix**: If using a local model, consider using a smaller model or leveraging GPU acceleration if available. If using a cloud service, check your internet connection and API response times.

3.  **Best Practice: Keyword Spotting**: In many robotic applications, you want the robot to respond only when a specific "wake word" or phrase is spoken. You can implement this by using a lightweight keyword spotting library to trigger the full speech-to-text transcription only after the wake word is detected.



## Further Reading

- **`speech_recognition` Library PyPI**: [https://pypi.org/project/SpeechRecognition/](https://pypi.org/project/SpeechRecognition/)
- **ROS 2 `std_msgs/String`**: [http://docs.ros.org/en/humble/API/std_msgs/msg/String.html](http://docs.ros.org/en/humble/API/std_msgs/msg/String.html)
- **Picovoice for Wake Word/Keyword Spotting**: A popular library for creating custom wake words. [https://picovoice.ai/](https://picovoice.ai/)

---

[**← Previous: Nav2 for Bipedal Path Planning**](../module-3/nav2-bipedal-path-planning.md) | [**Next: Cognitive Planning with LLMs →**](./cognitive-planning-llm-to-ros2.md)
