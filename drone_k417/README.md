# K417 Final Platform

This folder contains the final integrated platform used in the thesis.

## Role in the project

The K417 implementation takes the control ideas validated on the E58 and applies them to the final drone platform, adding the final electro-tactile feedback behavior and the integrated modular firmware layout used in the thesis work.

## Main files

- [drone_k417.ino](./drone_k417.ino)
  Main modular Arduino firmware for the final K417 platform.

- `drone_k417_ahrs.h`
  Mahony-based attitude estimation.

- `drone_k417_haptics.h`
  Final haptic feedback logic, including the safe multi-channel continuous stimulation scheduler.

- `drone_k417_protocol.h`
  K417 packet generation and control helpers.

- `drone_k417_serial.h`
  Serial command and diagnostics helpers.

- `drone_k417_state.h`
  Shared runtime state.

- `drone_k417_nn.h`
  Embedded glove-inference support.

- `control_video_v7.py`
  Main PC-side controller retained with the K417 workflow.

- `telemetry_monitor.py`
  Runtime telemetry monitor.

- `noise_bar_detector.py`
  Utility used during signal/debug experiments.

## Subfolders

- `neural/`
  Dataset tools, training scripts, exported models, and the embedded inference assets used by the glove classifier.

- `camera_prog/`
  Camera and auto-tracking utilities used around the K417 workflow.

- `captures/`
  Captured material from experiments.

- `build/`
  Arduino build output.

- `Original project/`
  Reverse-engineering reference material kept for traceability. It is not the main code path for the thesis deliverable.

## Read this next

- [K417_PROTOCOL.md](./K417_PROTOCOL.md)
- [../HAPTIC_FEEDBACK_SYSTEM_DOCUMENTATION.md](../HAPTIC_FEEDBACK_SYSTEM_DOCUMENTATION.md)
