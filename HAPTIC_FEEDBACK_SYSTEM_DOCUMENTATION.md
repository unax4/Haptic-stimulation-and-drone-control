# Haptic Feedback System Documentation

This document describes the active electro-tactile feedback architecture used by the modular drone firmware in this repository.

## Scope

The active firmware targets are:

- [drone_e58/drone_e58.ino](./drone_e58/drone_e58.ino)
- [drone_k417/drone_k417.ino](./drone_k417/drone_k417.ino)

The standalone electrical validation sketch is:

- [control_pcb/control_pcb.ino](./control_pcb/control_pcb.ino)

## Hardware blocks

The haptic system is driven by three functional blocks:

1. A digital potentiometer that sets stimulation intensity.
2. A high-voltage switch matrix that routes the signal to the selected electrode pair.
3. A pulse output pin that generates the stimulation waveform.

The exact hand-region mapping and potentiometer ranges are defined in the firmware constants for each drone platform.

## Software model

The modular firmware splits the haptic system into four tasks:

1. Interpret control activity from yaw, pitch, roll, and throttle.
2. Convert activity magnitude into a per-channel stimulation intensity.
3. Route the correct electrode pair through the HV switch.
4. Emit pulses with a non-blocking state machine.

The main files are:

- `drone_haptics.h` in `drone_e58/`
- `drone_k417_haptics.h` in `drone_k417/`

## Pulse modes

The firmware uses a small state machine for pulse generation:

- `IDLE`
- `SINGLE`
- `BURST`
- `TRAIN`
- `MULTI` for safe multi-channel continuous feedback

Discrete events such as takeoff, land, zero, stop, or mode changes use single or burst-like behavior. Continuous control feedback uses train-based behavior, and when more than one continuous channel is active the firmware switches to the multi-channel scheduler.

## Simultaneous stimulation strategy

The final simultaneous-feedback strategy is not true electrical parallel routing. Instead, it uses a safe pulse-slot scheduler.

When more than one continuous feedback channel is active:

1. The firmware builds the list of active haptic channels.
2. Only one channel is routed through the HV switch for each pulse slot.
3. The potentiometer is updated for that channel.
4. One pulse is emitted.
5. The routing is cleared again.
6. The next active channel is served in the next slot.

This approach matters because it guarantees that the channel being pulsed is also the channel actually routed at that instant. In practice, it is safer and more reliable than leaving multiple routes active or trying to overlap channel selection with pulse generation.

## Why this is the active approach

The goal of the final implementation is reliable perception of multiple feedback sources without ambiguous routing.

Compared with a simpler single-train implementation, the active pulse-slot scheduler gives:

- deterministic routing for every emitted pulse
- clear separation between electrode paths
- non-blocking behavior inside the main control loop
- a direct way to support several active control channels at once

The perceptual effect is simultaneous feedback by rapid alternation, while the electrical path remains unambiguous for every pulse.

## Control flow

At runtime the haptic path is:

1. Read IMU and flex data.
2. Compute control variables.
3. Detect which haptic channels should be active.
4. For a single active channel, use the normal train mode.
5. For multiple active channels, enter the multi-channel scheduler.
6. Update the pulse state machine every loop iteration.

## Electrical validation workflow

Use [control_pcb/control_pcb.ino](./control_pcb/control_pcb.ino) when the goal is to validate the hardware independently of the drone firmware.

That sketch is useful for:

- checking that the potentiometer changes intensity
- checking that the HV switch opens the requested path
- validating pulse timing
- verifying individual electrode routes before flight tests

## Practical note

The haptic firmware constants already encode the intended potentiometer ranges, pulse widths, and default frequencies for each platform. Those values should be edited in the corresponding sketch or haptics header, not duplicated manually across multiple documents.
