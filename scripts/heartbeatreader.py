import can

# === CAN Configuration ===
CAN_CHANNEL = 0
CAN_DEVICE = 0
CAN_BITRATE = 1000000
TARGET_CAN_ID = 0x01011840  # FRC heartbeat ID (extended)

def get_bits(bitstring, start, length):
    if start + length > len(bitstring):
        raise ValueError(f"Invalid bit range: {start} + {length} > {len(bitstring)}")
    return int(bitstring[start:start + length], 2)
    
def decode_frc_payload(payload):
    if len(payload) != 8:
        raise ValueError("Expected 8-byte CAN payload")

    # Correct byte order: LSB first
    bits = ''.join(f'{b:08b}' for b in payload)

    return {
        "match_time":      get_bits(bits, 56, 8),
        "match_number":    get_bits(bits, 46, 10),
        "replay_number":   get_bits(bits, 40, 6),
        "red_alliance":    get_bits(bits, 39, 1),
        "enabled":         get_bits(bits, 38, 1),
        "autonomous":      get_bits(bits, 37, 1),
        "test":            get_bits(bits, 36, 1),
        "watchdog":        get_bits(bits, 35, 1),
        "tournament_type": get_bits(bits, 32, 3),
        "year":            2000 + get_bits(bits, 26, 6) - 36,
        "month":           get_bits(bits, 22, 4) +1,
        "day":             get_bits(bits, 17, 5),
        "seconds":         get_bits(bits, 11, 6),
        "minutes":         get_bits(bits, 5, 6),
        "hours":           get_bits(bits, 0, 5),
    }


def format_time(d):
    return f"{d['year']:04}-{d['month']:02}-{d['day']:02} {d['hours']:02}:{d['minutes']:02}:{min(d['seconds'], 59):02}"

def main():
    print("Listening for FRC heartbeat via CANalyst-II...")

    try:
        bus = can.Bus(interface='canalystii', channel=CAN_CHANNEL, device=CAN_DEVICE, bitrate=CAN_BITRATE)
    except Exception as e:
        print(f"CAN initialization failed: {e}")
        return

    try:
        while True:
            msg = bus.recv(timeout=1)
            if msg and msg.arbitration_id == TARGET_CAN_ID and msg.is_extended_id and len(msg.data) == 8:
                try:
                    decoded = decode_frc_payload(msg.data)
                    print(f"[{format_time(decoded)}] "
                          f"{'RED' if decoded['red_alliance'] else 'BLUE'} | "
                          f"{'ENABLED' if decoded['enabled'] else 'DISABLED'} | "
                          f"{'AUTO' if decoded['autonomous'] else 'TELEOP'} | "
                          f"Match {decoded['match_number']} Replay {decoded['replay_number']} | "
                          f"Time Left: {decoded['match_time']}s")
                except ValueError as err:
                    print(f"[Decode error] {err}")

    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        bus.shutdown()

if __name__ == "__main__":
    main()
