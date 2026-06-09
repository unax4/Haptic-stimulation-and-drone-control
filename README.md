# Haptic Stimulation and Drone Control

This repository contains the code used for a thesis project on glove-based drone control with electro-tactile feedback.

The project evolved in two clear stages:

1. `drone_e58/` was the prototype platform used to derive the flight-control logic, validate the WiFi control workflow, and iterate on the IMU-to-stick mapping.
2. `drone_k417/` is the final integrated platform used for the thesis, where the control pipeline was ported to the K417 and combined with the final haptic feedback behavior.

## Main folders

- `drone_e58/`
  Prototype implementation for the E58 platform. It keeps the modular Arduino firmware, the PC-side control scripts used during protocol and control-law development, the WiFi protocol sniffing helpers, and the neural assets used for glove inference experiments.

- `drone_k417/`
  Final implementation for the K417 platform. This is the main firmware branch for the thesis and includes the modular Arduino controller, the haptic feedback modules, the PC-side monitoring/control utilities, the retained protocol sniffing helpers, and the neural assets used in the final system.

- `control_pcb/`
  Standalone sketch for validating the electro-tactile PCB. It is used to test the MAX5413, the HV switch matrix, pulse generation, and electrode routing without involving the drone controller.

- `TFM_Guante_Haptico/`
  Thesis source files.

- `BOH-Electro-Tactile-main/`
  External reference implementation kept in the repository because it informed the electro-tactile hardware/software design.

- `captures/`
  Captured data and support material.

- `etc/`
  Miscellaneous support scripts and older utilities kept for reference.

- `papatxe_copy_20260528154051/`
  Historical snapshot kept as backup/reference material.

## Main active programs

### Firmware

- [drone_e58/drone_e58.ino](./drone_e58/drone_e58.ino)
  Modular Arduino Nano RP2040 Connect firmware for the E58 prototype.

- [drone_k417/drone_k417.ino](./drone_k417/drone_k417.ino)
  Modular Arduino Nano RP2040 Connect firmware for the final K417 system.

- [control_pcb/control_pcb.ino](./control_pcb/control_pcb.ino)
  Electrical and routing test sketch for the haptic PCB.

### PC-side utilities

- `drone_e58/control_video_e58_v8.py`
  Main E58 Python controller used to study and validate the E58 protocol and flight behavior.

- `drone_k417/control_video_v7.py`
  Main K417 Python-side control utility retained in the final platform folder.

- `drone_k417/telemetry_monitor.py`
  Telemetry and runtime monitoring tool for the final K417 workflow.

- `drone_k417/noise_bar_detector.py`
  Debug utility used during signal and telemetry inspection.

## Structure cleanup

The repository now keeps only the modular firmware version inside each drone folder:

- `drone_e58/` contains the modular E58 sketch and its headers.
- `drone_k417/` contains the modular K417 sketch and its headers.
- The previous non-modular top-level drone sketches were removed.
- Neural assets remain under the drone folder that actually uses them.

## Recommended reading order

1. Read [README.md](./README.md)
2. Read [drone_e58/README.md](./drone_e58/README.md)
3. Read [drone_k417/README.md](./drone_k417/README.md)
4. Read [HAPTIC_FEEDBACK_SYSTEM_DOCUMENTATION.md](./HAPTIC_FEEDBACK_SYSTEM_DOCUMENTATION.md)
5. Use [control_pcb/README.md](./control_pcb/README.md) when validating the PCB independently of the drone logic.
