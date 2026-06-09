# E58 Prototype Platform

This folder contains the modular E58 implementation used to design and validate the flight-control logic before the final K417 integration.

## Role in the project

The E58 platform was the development and validation stage for:

- WiFi connection and packet flow
- IMU-to-stick mapping
- early haptic integration
- glove-based neural inference experiments
- PC-side control tooling

The control strategy developed here was later ported to the K417 final platform.

## Main files

- [drone_e58.ino](./drone_e58.ino)
  Main modular Arduino firmware for the E58 prototype.

- `drone_ahrs.h`
  Mahony-based attitude estimation.

- `drone_haptics.h`
  Haptic feedback logic for the E58 prototype.

- `drone_protocol.h`
  E58 packet generation and session-control helpers.

- `drone_serial.h`
  Serial command and diagnostic helpers.

- `drone_state.h`
  Shared runtime state.

- `drone_nn.h`
  Embedded glove-inference support.

- `control_video_e58_v8.py`
  Main Python controller used to study and validate the E58 communication path.

- `control_video_e58_v7.py`
  Earlier controller revision kept for comparison.

- `distance_estimator_v2.py`
  Vision-side distance estimation helper.

- `haptic_live_monitor.py`
  Live inspection utility for haptic-related runtime information.

- `protocol_sniff/sniffmobile_e58.py`
  Packet-capture helper preserved from the protocol reverse-engineering stage. It was used to sniff mobile-to-drone WiFi traffic while studying the E58 control packets.

## Subfolders

- `neural/`
  Dataset tools, training scripts, exported models, and the Eloquent/TFLite deployment assets used by the glove classifier.

- `archive/`
  Older firmware snapshots kept for reference only.

- `captures/`
  Captured material from experiments.

- `protocol_sniff/`
  Reverse-engineering helpers kept specifically because they were used to sniff WiFi traffic and infer the E58 protocol structure.

## Read this next

- [E58_WIFI_CAM_PROTOCOL.md](./E58_WIFI_CAM_PROTOCOL.md)
- [../HAPTIC_FEEDBACK_SYSTEM_DOCUMENTATION.md](../HAPTIC_FEEDBACK_SYSTEM_DOCUMENTATION.md)
