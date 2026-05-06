# E58 WIFI CAM Python Communication Documentation

This document explains, in implementation-level detail, how the Python controller communicates with the drone in this workspace.

Scope of this document:
- Codebase target: drone_e58/control_video_e58_v8.py
- Protocol family: E58 WIFI CAM over UDP
- Topics covered: connection flow, control packet build/send, receive path, command mapping, headless, flip, checksum, and known constraints

## 1. High-level communication architecture

The Python app uses UDP sockets and runs two concurrent communication planes:

1. Control plane
- Sends connect and disconnect handshakes on UDP port 8080
- Sends start-control wake packets on UDP port 8090
- Sends continuous RC control packets (CAM8 format) on UDP port 8090

2. Video receive plane
- Receives UDP datagrams from the drone on two local sockets (control and session)
- Reassembles JPEG frames from UDP payload stream by SOI and EOI markers

3. Optional telemetry parse helper
- A TelemetryParser class exists and can parse battery and altitude from raw payload bytes
- In v8, incoming UDP payloads are not currently passed into TelemetryParser.ingest in the video receive loop

## 2. Network defaults and endpoints

Default values in the controller:
- Drone IP: 192.168.4.153
- Session port: 8080
- Control port: 8090
- Default control rate: 30 Hz (GUI can change this)

Handshake payloads:
- CONNECT: 42 76 (hex)
- DISCONNECT: 42 77 (hex)
- START_CONTROL burst packet: AA 80 80 00 80 00 80 55 (hex)

## 3. Send order and session lifecycle

There are two typical startup paths in the GUI:

1. Control only start
- Send CONNECT to drone_ip:8080
- Send START_CONTROL burst to drone_ip:8090 (default 6 packets, 30 ms spacing)
- Start control loop thread that continuously emits CAM8 RC packets

2. Video start
- Create WifiCamVideoAdapter with its own control/session sockets
- Inject adapter control socket into FlightController so control and video share source endpoint
- Send CONNECT from adapter session socket
- Send START_CONTROL burst from adapter control socket
- Start control loop
- Start OpenCV display thread

Disconnect flow:
- If video adapter is active: send DISCONNECT from adapter and stop adapter
- Otherwise: send DISCONNECT from FlightController session socket
- Stop control loop and release shared socket

## 4. CAM8 control packet format

The controller builds 8-byte packets with this exact structure:

- Byte 0: 0x66 (header)
- Byte 1: roll
- Byte 2: pitch
- Byte 3: throttle
- Byte 4: yaw
- Byte 5: cmd
- Byte 6: chk
- Byte 7: 0x99 (tail)

Formula:
- cmd = command OR headless, plus optional somersault bit
- chk = roll XOR pitch XOR throttle XOR yaw XOR cmd

In code logic:
- cmd_i = (command | headless) & 0xFF
- if somersault_flag: cmd_i |= 0x08
- chk = roll_i ^ pitch_i ^ throttle_i ^ yaw_i ^ cmd_i

### 4.1 Valid range and clamping

All analog axes are clamped to byte range 0..255 before serialization.

Typical stick constants used by the controller:
- STICK_MIN = 40
- STICK_MID = 128
- STICK_MAX = 220

## 5. Command and flag model

The state machine is one-shot by flags.
A command flag is set by GUI or hotkey, then consumed in the flight loop.

Consume priority order:
1. takeoff
2. stop
3. land
4. headless event
5. calibrate
6. cam up
7. cam down
8. none

Meaning: if multiple flags are set at once, only the highest priority one is emitted in that loop iteration.

## 6. Command byte values currently implemented

Primary command constants:
- CMD_NONE = 0x00
- CMD_TAKEOFF = 0x01
- CMD_LAND = 0x02
- CMD_STOP = 0x04
- CMD_CALIBRATE = 0x80

Headless bit constants:
- HEADLESS_OFF = 0x00
- HEADLESS_ON = 0x10

Flip/somersault bit:
- SOMERSAULT flag bit = 0x08 (OR-ed into cmd byte during flip burst)

Camera constants currently in v8:
- CMD_CAM_UP = 0x00
- CMD_CAM_DOWN = 0x00

Important note:
- The camera helper docstring mentions 0x05 and 0x06, but constants are both 0x00 in v8.
- So camera up/down currently emit neutral command byte unless constants are updated.

## 7. Headless behavior in v8

Headless in v8 has two layers:

1. Protocol event layer
- When toggling headless in GUI, state.headless is toggled and state.headless_flag is set true
- On next consume_flags call, command 0x10 is emitted once
- Due checksum formula, neutral sticks produce exactly:
  - 66 80 80 80 80 10 10 99

2. Software flight-frame layer
- While state.headless is true, roll and pitch are rotated from pilot frame to drone frame
- Rotation uses estimated drone heading from commanded yaw integration
- At headless activation, current estimated heading is stored as reference
- Relative heading drives the roll/pitch transform each loop

Mathematically:
- yaw_norm = (yaw_stick - STICK_MID) / (STICK_MAX - STICK_MID)
- heading += yaw_norm * max_yaw_rate_dps * dt
- rel = wrap(heading - heading_ref)
- [roll_out, pitch_out] = rotate([roll_in, pitch_in], rel)

Consequence:
- Protocol command direction is correct
- Practical quality depends on heading estimate accuracy (no absolute magnetometer from drone in this path)

## 8. Flip behavior (complete)

This implementation has two ways to start a flip:

1. Manual (GUI or keyboard)
- GUI buttons call _cmd_flip(direction)
- Keyboard path is hold F, then arrow key
- _cmd_flip sets:
  - state.flip_dir = selected direction
  - state.flip_active = True
  - _flip_started_ts = now (for watchdog)

2. NN one-shot mode (class 7)
- Position/class 7 does not flip immediately
- It arms flip mode one-shot:
  - _nn_flip_mode = True
  - _nn_flip_trigger_latched = False
  - _nn_flip_mode_since = now
- While armed, next stick extreme triggers one flip:
  - roll == STICK_MAX -> right flip
  - roll == STICK_MIN -> left flip
  - pitch == STICK_MAX -> forward flip
  - pitch == STICK_MIN -> backward flip
- After trigger, armed mode is cleared immediately (one-shot)
- Armed mode auto-cancels after 3.0 s timeout if no trigger

### 8.1 Direction encoding sent to drone

Direction is encoded by forcing one axis to extreme while keeping the other centered:

- forward: pitch = STICK_MAX, roll = STICK_MID
- backward: pitch = STICK_MIN, roll = STICK_MID
- left: roll = STICK_MIN, pitch = STICK_MID
- right: roll = STICK_MAX, pitch = STICK_MID

The somersault bit is applied only during burst packets:
- cmd |= 0x08 during burst

### 8.2 Packet phases during a flip

When state.flip_active is true, FlightController._loop skips normal RC packet path and runs _do_flip().

Phase A: burst (rotation command)
- Packet count: FLIP_BURST_PACKETS (16)
- roll/pitch: forced to selected direction
- yaw: held from snapshot
- cmd: SOMERSAULT bit set

Phase B: recovery (settle)
- Packet count: FLIP_RECOVER_PACKETS (8)
- roll/pitch: neutral (STICK_MID)
- yaw: held from snapshot
- cmd: no somersault bit
- throttle: simple two-stage profile
  - instant stage: first FLIP_RECOVER_INSTA_PACKETS (3) at recover_insta_thr
  - taper stage: remaining packets taper to recover_end_thr

Phase C: handoff to live sticks
- A short post-flip throttle floor is applied for FLIP_POST_HOLD_S
- This prevents immediate thrust collapse right after forced recovery

### 8.3 Flip throttle profile (anti-drop logic)

To reduce altitude loss right after flips, throttle is not kept at raw pre-flip snapshot.
Instead it uses a bounded profile:

- base_thr = max(snapshot_throttle, FLIP_THR_MIN)
- burst_thr = min(STICK_MAX, base_thr + FLIP_THR_BURST_BOOST)
- recover_insta_thr = min(STICK_MAX, base_thr + FLIP_THR_RECOVER_BOOST)
- recover_end_thr = min(STICK_MAX, base_thr + FLIP_THR_POST_BOOST)

Current values:
- FLIP_THR_MIN = 165
- FLIP_THR_BURST_BOOST = 28
- FLIP_RECOVER_INSTA_PACKETS = 3
- FLIP_THR_RECOVER_BOOST = 26
- FLIP_THR_POST_BOOST = 8
- FLIP_POST_HOLD_S = 0.15

Original v8 values (before tuning changes):
- FLIP_BURST_PACKETS = 20
- FLIP_RECOVER_PACKETS = 10
- FLIP_THR_MIN = 165
- FLIP_THR_BURST_BOOST = 28
- FLIP_THR_RECOVER_BOOST = 20
- FLIP_THR_POST_BOOST = 22
- FLIP_POST_HOLD_S = 0.40

Practical meaning:
- Burst gets highest throttle
- Recovery gives a strong instant catch pulse, then settles quickly
- A short throttle floor bridges handoff back to live sticks

Per-drone variability guidance:
- Usually keep fixed across drones:
  - FLIP_BURST_PACKETS
  - FLIP_RECOVER_PACKETS
  - FLIP_RECOVER_INSTA_PACKETS
  - FLIP_THR_BURST_BOOST
- Usually tune per drone:
  - FLIP_THR_RECOVER_BOOST (primary)
  - FLIP_POST_HOLD_S (secondary)
- Optional third knob only if needed:
  - FLIP_THR_MIN

### 8.4 Exit and safety behavior

Normal completion:
- _do_flip finishes burst + recovery
- finally block always clears:
  - state.flip_active = False
  - state.flip_dir = None
- post-flip floor hold is armed briefly for handoff

Hard safety exits:
- GUI watchdog in _tick clears stale flip if active time exceeds _flip_max_active_s (2.5 s)
- stop/land paths still clear any residual flip state
- FlightController.stop() clears flip flags

NN flip-arm safety:
- Armed state expires after timeout (3.0 s)
- Latch logic requires returning sticks near center to re-arm trigger edge

### 8.5 What "flip mode exited" means in UI

The panel status shows three states:
- FLIP: ACTIVE -> currently executing burst/recovery
- FLIP: ARMED -> NN one-shot mode armed, waiting for trigger stick extreme
- FLIP: IDLE -> no active flip and not armed

This gives immediate visual confirmation that flip mode has exited.

## 9. Calibration packet behavior

When command is calibrate (0x80), the implementation forces all analog channels to stick mid:
- roll = pitch = throttle = yaw = 128

Then packet is built normally with cmd=0x80 and checksum accordingly.

## 10. Control loop timing and threading

FlightController loop:
- Runs in daemon thread
- Nominal period = 1 / rate
- Sends one control packet per iteration
- Sleeps max(1 ms, interval - processing_time)

State handling:
- Shared state is protected by a lock in DroneState
- Snapshot returns current analog values
- Counter fields c1,c2,c3 are incremented but not encoded into CAM8 payload (kept for compatibility with function signature)

## 11. Receive path: video datagrams to frames

WifiCamVideoAdapter has two non-blocking sockets:
- control socket bound to local ephemeral port
- session socket bound to local ephemeral port

Receive loop:
- Uses select over both sockets
- recvfrom up to 65535 bytes
- Accepts only packets from configured drone IP
- Feeds payload bytes to JPEG extractor

JPEG extraction strategy:
1. Fast path: detect full JPEG(s) contained in one datagram by FF D8 ... FF D9
2. Fragment path: accumulate bytes from SOI until EOI across datagrams
3. Drops overly large fragment buffer over 2 MB as safety

Frame queue:
- Queue size is 2
- On new frame, stale frame may be dropped to keep latest view responsive

## 12. Telemetry parser details

TelemetryParser expects generic payload bytes and extracts:
- battery_pct from payload[4] if 0..100
- altitude_cm from payload[6]
- raw_last as first 16 bytes for debug display

Filter rule:
- Rejects if payload length < 8
- Rejects if payload[1] == 0x01

Current integration state in v8:
- TelemetryParser object exists and GUI reads snapshot for display
- Incoming UDP payloads in WifiCamVideoAdapter._rx_loop are currently not forwarded to telemetry.ingest
- Therefore displayed telemetry may remain N/A unless another source updates it

## 13. User actions to command bytes mapping

Main command actions:
- Takeoff: sets takeoff_flag, emits cmd 0x01 one-shot
- Land: sets land_flag, emits cmd 0x02 one-shot
- Stop: sets stop_flag, emits cmd 0x04 one-shot
- Calibrate: sets calibrate_flag, emits cmd 0x80 one-shot
- Headless toggle:
  - toggles persistent software headless state
  - sets headless_flag for one-shot cmd 0x10 event

Keyboard mirrors GUI actions:
- T takeoff
- L land
- Space stop
- H headless toggle
- C calibrate
- F + arrows flip direction

## 14. Packet examples

1. Neutral no command
- roll=pitch=throttle=yaw=0x80
- cmd=0x00
- chk=0x00
- Packet: 66 80 80 80 80 00 00 99

2. Headless event (matches captured mobile app behavior)
- roll=pitch=throttle=yaw=0x80
- cmd=0x10
- chk=0x10
- Packet: 66 80 80 80 80 10 10 99

3. Takeoff from neutral
- cmd=0x01
- chk=0x01
- Packet: 66 80 80 80 80 01 01 99

4. Flip burst neutral throttle/yaw and forward direction
- roll=0x80, pitch=0xDC (220), cmd includes 0x08 bit
- chk computed with that cmd bit included

## 15. Why this works from Python

Python can control this drone because the protocol is simple UDP datagrams without session encryption:
- Fixed handshake datagrams start the drone control stack
- Continuous 8-byte RC packets drive attitude and command functions
- The drone accepts command bytes and checksum directly
- Video stream is raw UDP payload carrying JPEG data fragments

The implementation uses:
- socket for UDP networking
- threading for independent control and receive loops
- select for efficient non-blocking multi-socket receive
- byte-level packet crafting with deterministic checksum

## 16. Known caveats and practical notes

1. Headless quality caveat
- Software headless uses estimated heading from commanded yaw, not absolute yaw telemetry
- Long maneuvers may accumulate drift

2. Camera command caveat
- CMD_CAM_UP and CMD_CAM_DOWN are both 0x00 in v8
- If camera tilt is required, command bytes must be confirmed and constants updated

3. Telemetry integration gap
- TelemetryParser exists but is not fed by receive loop in v8
- To activate telemetry display from UDP replies, call telemetry.ingest(payload) in receive path before or after JPEG extraction

4. Packet counters
- build_packet signature includes c1,c2,c3 but CAM8 payload currently does not include them
- Safe for current format, but they are effectively ignored in serialized bytes

## 17. Quick verification checklist

If commands appear not to work, verify in this order:

1. Network path
- PC connected to drone AP
- Correct drone IP and ports

2. Startup sequence
- CONNECT sent to 8080
- START_CONTROL burst sent to 8090
- Control loop running at expected rate

3. Packet bytes
- Sniff outgoing UDP and confirm CAM8 format 66 ... 99
- Confirm checksum matches XOR rule

4. Headless
- Toggle should emit one packet with cmd bit 0x10
- Neutral expected packet: 66 80 80 80 80 10 10 99

5. Continuous control
- After one-shot commands, cmd should return to 0x00 unless another event is active

---

If you want, a second document can be added with Wireshark display filters and a troubleshooting matrix for each command (takeoff, land, stop, headless, flip) including expected packet traces and failure signatures.
