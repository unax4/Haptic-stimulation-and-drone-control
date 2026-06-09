# K417 Final Platform

This folder contains the final integrated platform used in the thesis.

## Role in the project

The K417 implementation takes the control ideas validated on the E58 and applies them to the final drone platform, adding the final electro-tactile feedback behavior and the integrated modular firmware layout used in the thesis work.

In practice, this is the folder that represents the final system described in the dissertation: glove sensing, onboard orientation estimation, gesture inference, haptic feedback generation, and wireless drone control are all brought together here.

## Architecture overview

The K417 firmware is intentionally split into small modules so that each technical block of the system can be understood and modified independently.

- `drone_k417.ino`
  Owns the high-level setup and loop. It initializes the IMU, WiFi, haptic hardware, and inference system, then orchestrates the runtime flow.

- `drone_k417_state.h`
  Centralizes the shared state used across modules: control sticks, filter state, haptic state, neural-network state, and flight-mode flags.

- `drone_k417_ahrs.h`
  Implements the orientation pipeline based on the Mahony filter. This module turns raw IMU measurements into the yaw, pitch, and roll values later mapped into flight commands and haptic cues.

- `drone_k417_protocol.h`
  Encodes and transmits the K417 UDP packets. It also contains the logic for control burst startup, calibration messages, land behavior, and flip sequencing.

- `drone_k417_haptics.h`
  Contains the electro-tactile routing logic, pulse generation state machine, and the safe multi-channel scheduler used when more than one control cue must be rendered at nearly the same time.

- `drone_k417_nn.h`
  Wraps the embedded glove classifier and the action mapping associated with recognized hand postures.

- `drone_k417_serial.h`
  Exposes the serial command surface used for diagnostics, tuning, and bench testing.

## Runtime flow

The main execution flow of the final K417 system is:

1. Read IMU and flex-sensor data.
2. Update the orientation estimate.
3. Convert glove motion into drone stick commands.
4. Run optional neural-network inference for posture-triggered actions.
5. Update the haptic feedback state according to the current control and mode state.
6. Build and send the K417 control packet over WiFi.
7. Emit telemetry over serial for monitoring and debugging.

This makes the folder useful not only as firmware source code, but also as the clearest implementation reference for the control architecture described in the thesis.

## Main files

- [drone_k417.ino](./drone_k417.ino)
  Main modular Arduino firmware for the final K417 platform.

- `drone_k417_ahrs.h`
  Mahony-based attitude estimation.

- `drone_k417_haptics.h`
  Final haptic feedback logic, including routing, pulse generation, action cues, and the safe multi-channel continuous stimulation scheduler.

- `drone_k417_protocol.h`
  K417 packet generation, WiFi transmission helpers, and flight-action packet logic.

- `drone_k417_serial.h`
  Serial command and diagnostics helpers.

- `drone_k417_state.h`
  Shared runtime state.

- `drone_k417_nn.h`
  Embedded glove-inference support.

- `control_video_v7.py`
  Main PC-side controller retained with the K417 workflow and useful as a protocol reference.

- `telemetry_monitor.py`
  Runtime telemetry monitor.

- `noise_bar_detector.py`
  Utility used during signal/debug experiments.

- `protocol_sniff/sniff_mobile_to_drone.py`
  Reverse-engineering helper used to capture mobile-to-drone WiFi traffic during protocol analysis.

- `protocol_sniff/sniff_drone_to_pc.py`
  Reverse-engineering helper used to inspect traffic in the opposite direction and understand the WiFi communication path more completely.

## Subfolders

- `neural/`
  Dataset tools, training scripts, exported models, and the embedded inference assets used by the glove classifier.

- `captures/`
  Captured material from experiments.

- `protocol_sniff/`
  Small set of preserved sniffing scripts used during the reverse-engineering phase to study the K417 WiFi protocol.

## What is final here and what is historical

The files at the root of `drone_k417/` are the public, active implementation of the final thesis platform.

The `protocol_sniff/` folder is kept as historical support material because it documents how the WiFi protocol was studied from captured traffic. Those scripts are not part of the runtime firmware, but they are useful for understanding and reproducing the reverse-engineering stage of the work.

## Read this next

- [K417_PROTOCOL.md](./K417_PROTOCOL.md)
- [../HAPTIC_FEEDBACK_SYSTEM_DOCUMENTATION.md](../HAPTIC_FEEDBACK_SYSTEM_DOCUMENTATION.md)
