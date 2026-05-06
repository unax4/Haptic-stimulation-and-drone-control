import binascii

try:
    from scapy.all import sniff, UDP
except ImportError:
    print("Please install scapy: pip install scapy")
    exit(1)

# Known expected values based on reverse engineered code
KNOWN_COMMANDS = {0x00, 0x01, 0x02, 0x04}
KNOWN_HEADLESS = {0x02, 0x03}

def packet_callback(packet):
    # Ensure it's UDP and destined for the drone's port
    if packet.haslayer(UDP) and packet[UDP].dport == 8800:
        payload = bytes(packet[UDP].payload)
        
        # Verify packet header matches the known control protocol (EF 02 7C ...)
        if len(payload) >= 36 and payload.startswith(b'\xef\x02\x7c'):
            
            # Extract analog sticks to show the state when the new command occurs
            roll = payload[20]
            pitch = payload[21]
            throttle = payload[22]
            yaw = payload[23]
            
            # Extract command flags
            command = payload[24]
            headless = payload[25]
            
            # Extract the 10-byte padding (often where auxiliary commands like camera hide)
            ctrl_pad = payload[26:36]
            
            # --- FILTER OUT ORDERS WE ALREADY HAVE ---
            
            # 1. Check for unknown command bytes
            is_new_command = command not in KNOWN_COMMANDS
            is_new_headless = headless not in KNOWN_HEADLESS
            
            # 2. Check if the physical controller uses the padding bytes for the camera
            is_pad_modified = ctrl_pad != (b'\x00' * 10)
            
            # If any unknown trait is found, isolate and print it!
            if is_new_command or is_new_headless or is_pad_modified:
                print("======================================================")
                print("[!] NEW UNKNOWN ORDER INTERCEPTED")
                print("======================================================")
                print(f"Sticks -> Roll:{roll} Pitch:{pitch} Throttle:{throttle} Yaw:{yaw}")
                
                if is_new_command:
                    print(f"[*] NEW Command Byte detected: 0x{command:02X} (Known: 00, 01, 02, 04)")
                else:
                    print(f"    Command Byte: 0x{command:02X}")
                    
                if is_new_headless:
                    print(f"[*] NEW Headless Byte detected: 0x{headless:02X} (Known: 02, 03)")
                else:
                    print(f"    Headless Byte: 0x{headless:02X}")
                    
                if is_pad_modified:
                    print(f"[*] Control Pad Bytes Modified! Hex: {ctrl_pad.hex()}")
                
                print(f"Raw Payload Hex: {payload.hex()}")
                print("======================================================\n")

print("[*] Listening for Drone Control Packets on UDP port 8800...")
print("[*] Filtering out known commands (Takeoff, Land, Stop, Calibrate, Sticks)...")
print("[*] Waiting for new inputs (Flip, Camera Up/Down)...")

# Capture packets directed to the drone
sniff(filter="udp dst port 8800", prn=packet_callback, store=0)