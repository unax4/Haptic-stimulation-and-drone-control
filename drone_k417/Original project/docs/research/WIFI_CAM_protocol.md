# WIFI CAM Protocol Notes (Observed)

This document describes the behavior captured from the mobile app and how to emulate it from Python.

## Scope

Observed environment:
- Phone app controls drone over UDP
- Drone IP example: 192.168.4.153
- Session port: 8080
- Control port: 8090

These notes are specific to the WIFI CAM protocol family captured in this repository context.

## Packet Flow

1. App idle: no packets.
2. Start/connect action: send UDP payload `42 76` to port 8080.
3. Before active RC loop: repeated 8-byte packet to 8090:
   - `AA 80 80 00 80 00 80 55`
4. Active control loop: repeated CAM8 packets to 8090:
   - `66 b1 b2 b3 b4 cmd chk 99`
5. Exit/disconnect action: send UDP payload `42 77` to port 8080.

## CAM8 Frame Format

Frame bytes:
- Byte 0: `0x66` (start marker)
- Byte 1: axis field b1
- Byte 2: axis field b2
- Byte 3: axis field b3
- Byte 4: axis field b4
- Byte 5: command (`cmd`)
- Byte 6: checksum (`chk`)
- Byte 7: `0x99` (end marker)

Checksum:
- `chk = b1 XOR b2 XOR b3 XOR b4 XOR cmd`

Neutral frame observed:
- `66 80 80 80 80 00 00 99`

## Axis Mapping (Current Hypothesis)

Based on capture behavior and prior toy-drone conventions:
- b1: roll
- b2: pitch
- b3: throttle
- b4: yaw

User observations that support this:
- Yaw/throttle operations changed b3 and b4 plus checksum.
- Roll/pitch operations changed b1 and b2 plus checksum.

Keep mapping configurable until fully verified for every action.

## Command Byte Values (Observed)

One-shot values captured:
- `0x01`: takeoff
- `0x02`: land
- `0x04`: stop
- `0x10`: headless toggle/control

For neutral axes, checksum equals command value because axis XOR is zero.

## How To Emulate the App

Use script: [experimental/wifi_cam_controller.py](experimental/wifi_cam_controller.py)

Run:

```bash
python experimental/wifi_cam_controller.py --drone-ip 192.168.4.153
```

Suggested sequence:
1. `start`
2. `takeoff`
3. `set throttle 150`
4. `set yaw 100`
5. `neutral`
6. `land`
7. `quit`

Command reference:
- `connect`
- `start` (connect + prestream burst + CAM8 loop)
- `set <roll|pitch|throttle|yaw> <0..255>`
- `takeoff`, `land`, `stop`, `headless`
- `neutral`
- `disconnect`
- `quit`

## Inbound Video Capture Workflow

When the app shows video right after connect, capture drone->PC UDP first.

1. Start inbound sniff:

```bash
python experimental/sniff_drone_to_pc.py --iface "Wi-Fi" --drone-ip 192.168.4.153 --duration 25 --no-bpf --exclude-src-ports 53 --min-udp-len 100
```

2. During that window, click connect in app and wait for video.

3. Reconstruct possible media from captured payloads:

```bash
python experimental/reconstruct_video_from_events.py --events captures/drone_to_pc_events.jsonl --out-dir captures/reconstructed
```

4. Check generated outputs:
- `captures/reconstructed/jpeg_carved/`
- `captures/reconstructed/stream_rtp.h264`
- `captures/reconstructed/wifi_uav_frames/`

Notes:
- `sniff_drone_to_pc.py` labels packets heuristically as RTP-like, WiFi-UAV-like, or JPEG-marker packets.
- `reconstruct_video_from_events.py` attempts multiple extraction strategies because some drone variants use different encodings/framing.
- If you only see DNS-like payloads with domain strings, you captured control-plane DNS traffic, not video.

## Capture Tips

Wireshark display filters:
- `udp.port == 8080 or udp.port == 8090`
- `ip.src == <phone_ip> and ip.dst == <drone_ip> and udp`

Validation checks:
- CAM8 frames should always be 8 bytes.
- Byte 0 should be `66`, byte 7 should be `99`.
- XOR checksum should match byte 6.

## Limitations

- Video transport details are not defined in this document.
- Some app actions may involve additional packets not yet mapped.
- Different drones sold under similar model names may use a different protocol family.
