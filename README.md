# Haptic Stimulation and Drone Control

This repository contains the code and technical material for a thesis project on glove-based drone control with electro-tactile feedback.

The development path of the project is reflected in two main firmware branches:

1. `drone_e58/`
   Prototype platform used to derive the control logic, validate the WiFi workflow, and iterate on glove sensing and haptic integration.
2. `drone_k417/`
   Final integrated platform used in the thesis, where the control pipeline was ported to the K417 and combined with the final haptic feedback behavior.

## Repository structure

- `drone_e58/`
  Modular E58 firmware, PC-side control utilities, neural assets, and preserved WiFi protocol sniffing helpers from the reverse-engineering stage.

- `drone_k417/`
  Modular K417 firmware, final haptic-control implementation, PC-side monitoring/control tools, neural assets, and preserved WiFi protocol sniffing helpers.

- `control_pcb/`
  Standalone sketch for validating the electro-tactile PCB independently of the drone-control firmware.

- `TFM_Guante_Haptico/`
  Thesis source files.

- `captures/`
  Captured data and support material shared at repository level.

- `etc/`
  Local support material and private historical backups kept outside the public project structure.

- `papatxe_copy_20260528154051/`
  Historical snapshot kept as backup/reference material.

## Main active firmware

- [drone_e58/drone_e58.ino](./drone_e58/drone_e58.ino)
  Modular Arduino Nano RP2040 Connect firmware for the E58 prototype platform.

- [drone_k417/drone_k417.ino](./drone_k417/drone_k417.ino)
  Modular Arduino Nano RP2040 Connect firmware for the final K417 platform.

- [control_pcb/control_pcb.ino](./control_pcb/control_pcb.ino)
  PCB-level stimulation test sketch for bench validation of routing, pulse generation, and electrode selection.

## Main PC-side tools

- `drone_e58/control_video_e58_v8.py`
  Main E58 controller used during protocol and control validation.

- `drone_k417/control_video_v7.py`
  Main K417 PC-side controller kept as a protocol and workflow reference.

- `drone_k417/telemetry_monitor.py`
  Telemetry monitor for the final K417 workflow.

- `drone_k417/noise_bar_detector.py`
  Utility used during debug and signal-inspection work.

## Reverse-engineering material

The repository keeps a minimal set of packet-sniffing helpers because they are part of how the WiFi protocols were understood:

- `drone_e58/protocol_sniff/`
  Includes the preserved E58 mobile-to-drone sniffing script.

- `drone_k417/protocol_sniff/`
  Includes preserved K417 scripts for mobile-to-drone and drone-to-PC traffic capture.

These scripts are historical support material. They are not part of the runtime firmware, but they document the protocol-analysis stage that made the embedded control implementation possible.

## Public repo conventions

The public repository keeps the modular firmware versions as the active ones:

- `drone_e58/` contains the modular E58 sketch and its headers.
- `drone_k417/` contains the modular K417 sketch and its headers.
- Arduino build artifacts and auxiliary camera-program folders were removed from the public tree.
- Large experimental reference folders were removed once the relevant sniffing scripts had been preserved in `protocol_sniff/`.

## Recommended reading order

1. [drone_k417/README.md](./drone_k417/README.md) for the final system architecture
2. [HAPTIC_FEEDBACK_SYSTEM_DOCUMENTATION.md](./HAPTIC_FEEDBACK_SYSTEM_DOCUMENTATION.md) for the stimulation pipeline
3. [drone_k417/K417_PROTOCOL.md](./drone_k417/K417_PROTOCOL.md) for the final drone packet structure
4. [drone_e58/README.md](./drone_e58/README.md) for the prototype stage
5. [control_pcb/README.md](./control_pcb/README.md) for standalone PCB validation
