# K417 Protocol

This document describes the control packet structure used by the final K417 firmware in this repository.

## Why this file matters

The K417 is the final integrated drone platform used in the thesis. Its firmware reuses the control ideas validated on the E58, but the transport packet is different and is implemented directly in the modular K417 firmware.

## Main implementation files

- `drone_k417.ino`: main modular firmware
- `drone_k417_protocol.h`: packet generation and transmission helpers
- `control_video_v7.py`: PC-side utility retained with the K417 workflow

## Network setup

- Drone IP: `192.168.169.1`
- Control port: `8800`

The firmware opens a local UDP socket and sends control packets directly to the drone.

## Start sequence

The K417 path does not use the same explicit connect/disconnect handshake as the E58 prototype code. Instead, the firmware starts the control stream by sending a neutral burst of packets.

## Packet structure

The K417 packet is a larger binary frame composed of:

1. a fixed header block
2. counter fields
3. a six-byte control block
4. checksum and suffix blocks

The active control bytes are:

- roll
- pitch
- throttle
- yaw
- command
- headless/somersault flags

The checksum is the XOR of those six control bytes.

## Supported actions

The modular K417 firmware supports:

- continuous control streaming
- takeoff and land commands
- calibration pulse
- headless mode
- flip execution with burst and settle phases
- haptic feedback synchronized with flight control

## Relation to the thesis workflow

This is the final drone-side control implementation. If the goal is to reproduce the thesis system, this folder is the main one to use together with the haptic documentation and the `control_pcb` validation sketch.
