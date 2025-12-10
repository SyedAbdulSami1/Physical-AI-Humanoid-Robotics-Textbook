---
title: Hardware Requirements
---

# Hardware Requirements

The course is technically demanding because it combines physics simulation, visual perception, and generative AI.

- **“Digital Twin” Workstation**: Needs an NVIDIA RTX 4070 Ti (12 GB VRAM) or higher (ideally RTX 3090/4090 with 24 GB), Intel Core i7 13th Gen+ or AMD Ryzen 9, 64 GB DDR5 RAM (32 GB minimum), and Ubuntu 22.04 LTS.
- **“Physical AI” Edge Kit**: Consists of NVIDIA Jetson Orin Nano or NX as the brain, Intel RealSense D435i/D455 for vision, a USB IMU, and a USB microphone/speaker array such as ReSpeaker.
- **Three robot lab options are described**:
    - Budget/proxy approach using Unitree Go2 Edu.
    - Miniature humanoid approach using Unitree G1, Robotis OP3 or budget Hiwonder TonyPi Pro.
    - Premium approach using Unitree G1 humanoid.
- **Additional Information**: The architecture summary table, cloud-native “Ether” lab option with AWS g5/g6 instances, cost calculations, and the **Economy Jetson Student Kit** (Jetson Orin Nano Super Dev Kit $249 + RealSense D435i $349 + ReSpeaker $69 + misc $30 = ~$700 total) must all be present along with the latency trap explanation and the final solution of training in the cloud and flashing to local Jetson.
