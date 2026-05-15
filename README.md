# Haptic Stimulation and Drone Control

A comprehensive system for drone control with haptic feedback integration, featuring computer vision capabilities and neural network-based gesture recognition.

## Project Overview

This project implements a haptic feedback system integrated with UAV (Unmanned Aerial Vehicle) control. It includes control protocols for multiple drone models (E58 and K417), real-time video processing, and neural network-based gesture recognition for intuitive drone control via a haptic glove.

## Directory Structure

### **Core Drone Projects**

#### `drone_e58/`
- **E58 Drone Control Implementation**
- Contains Python control scripts and firmware for the E58 drone model
- **Key files:**
  - `drone_e58.ino` - Arduino firmware for E58 drone
  - `control_video_e58_v7.py`, `control_video_e58_v8.py` - Control scripts with video processing
  - `distance_estimator_v2.py` - Distance estimation from camera feed
  - `haptic_live_monitor.py` - Real-time haptic feedback monitoring
  - `E58_WIFI_CAM_PROTOCOL.md` - WiFi camera communication protocol documentation
- **Subdirectories:**
  - `neural/` - Neural network models for gesture recognition (TFLite models, training scripts)
  - `build/` - Arduino build artifacts
  - `archive/` - Previous versions of firmware

#### `drone_e58_module/`
- **Modular Arduino Implementation for E58 with Haptic Integration**
- Header files implementing modular drone control system
- **Key components:**
  - `drone_e58_module.ino` - Main module firmware
  - `drone_ahrs.h` - AHRS (Attitude Heading Reference System) implementation
  - `drone_haptics.h` - Haptic feedback controller
  - `drone_nn.h` - Neural network inference module
  - `drone_protocol.h` - Communication protocol handler
  - `drone_serial.h` - Serial communication interface
  - `drone_state.h` - Drone state management
  - `HAPTIC_FEEDBACK_SYSTEM_DOCUMENTATION.md` - Detailed haptic system documentation
- **Subdirectories:**
  - `neural/` - TFLite models and training data for gesture recognition

#### `drone_k417/`
- **K417 Drone Control Implementation**
- Similar structure to E58 but specific to K417 model
- **Key files:**
  - `drone_k417.ino` - Arduino firmware for K417
  - `control_video_*.py` - Multiple versions of control scripts
  - `distance_estimator_v2.py` - Distance measurement
  - `telemetry_monitor.py` - Real-time telemetry display
  - `noise_bar_detector.py` - Audio/signal noise detection
- **Subdirectories:**
  - `neural/` - Neural network models
  - `camera_prog/` - Camera-specific control and tracking code
  - `build/` & `build_noNN/` - Build artifacts (with and without neural network)
  - `Original project/` - Complete reference implementation with backend/frontend

#### `drone_k417_module/`
- **Modular Arduino Implementation for K417 with Haptic Integration**
- Header files implementing modular drone control system
- **Key components:**
  - `drone_k417_module.ino` - Main module firmware
  - `drone_ahrs.h` - AHRS (Attitude Heading Reference System) implementation
  - `drone_haptics.h` - Haptic feedback controller
  - `drone_nn.h` - Neural network inference module
  - `drone_protocol.h` - Communication protocol handler
  - `drone_serial.h` - Serial communication interface
  - `drone_state.h` - Drone state management
  - `HAPTIC_FEEDBACK_SYSTEM_DOCUMENTATION.md` - Detailed haptic system documentation
- **Subdirectories:**
  - `neural/` - TFLite models and training data for gesture recognition

#### `est_fuante_pruebas/`
- **Testing and Experimentation**
- Contains test implementations for fuente (source) components
- `est_fuante_pruebas.ino` - Test Arduino firmware

### **Root Level Files**

- `main_prog_vMahony.py` - Main program using Mahony AHRS algorithm
- `HAPTIC_FEEDBACK_SYSTEM_DOCUMENTATION.md` - System-wide haptic documentation
- `yolov8n.pt`, `yolov8n-seg.pt` - YOLOv8 pre-trained models for object detection/segmentation

## Key Technologies

- **Hardware:** Arduino (Nano RP2040 Connect), E58 & K417 drones, Haptic gloves
- **Computer Vision:** OpenCV, YOLOv8, distance estimation
- **Neural Networks:** TensorFlow Lite for gesture recognition
- **Protocols:** WiFi camera protocol, Serial communication, AHRS/IMU integration
- **AHRS:** Mahony algorithm for attitude estimation

## Documentation

- [Haptic Feedback System Documentation](./drone_e58_module/HAPTIC_FEEDBACK_SYSTEM_DOCUMENTATION.md)
- [E58 WiFi Protocol](./drone_e58/E58_WIFI_CAM_PROTOCOL.md)
- [K417 WiFi Protocol](./drone_k417/E58_WIFI_CAM_PROTOCOL.md)

## Getting Started

1. Select your drone model (E58 or K417)
2. Review the relevant control scripts and documentation
3. Upload the appropriate Arduino firmware (.ino file)
4. Run the Python control script for computer vision and haptic feedback
5. Check neural network models in `neural/` directories for gesture recognition

## Project Structure Notes

- Multiple version numbers on files indicate iterative development
- `neural/` directories contain TFLite models ready for embedded inference
- Build directories contain Arduino compilation artifacts
- Original project folders contain reference implementations with full stack (backend + frontend)
