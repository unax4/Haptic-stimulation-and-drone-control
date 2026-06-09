# E58 WiFi CAM Protocol

This document describes the E58 communication path used in the prototype stage of the project.

## Why this file matters

The E58 was the platform used to derive the PC-side control workflow that later informed the final K417 implementation. If you want to understand where the flight-control logic came from, start here.

## Main implementation files

- `control_video_e58_v8.py`: main Python-side protocol and control implementation
- `control_video_e58_v7.py`: earlier revision kept for comparison
- `drone_e58.ino`: modular Arduino-side prototype firmware

## Network setup

- Drone IP: `192.168.4.153`
- Session port: `8080`
- Control port: `8090`

## Session flow

1. Send the connect handshake to the session port.
2. Send a short start-control burst to the control port.
3. Stream control packets at the configured controller rate.
4. Optionally receive video datagrams and reassemble JPEG frames.

## Packet format

The E58 control packet is the compact CAM8-style frame used by the prototype controllers:

- byte 0: header `0x66`
- byte 1: roll
- byte 2: pitch
- byte 3: throttle
- byte 4: yaw
- byte 5: command
- byte 6: checksum
- byte 7: tail `0x99`

Checksum:

`roll XOR pitch XOR throttle XOR yaw XOR command`

## Main commands

- takeoff
- land
- stop
- calibrate
- headless pulse
- flip/somersault

## Why the E58 folder still matters

Although the thesis final platform is the K417, this folder preserves the prototype stage where:

- the control law was iterated
- protocol assumptions were validated
- the glove-control pipeline was tested end to end
- the haptic workflow was first integrated with flight control
