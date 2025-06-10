import can
from can.notifier import Notifier, Listener

# Constants for identification
CTRE_PDP_TYPE_ID = 0x08
CTRE_PDP_MANUF_ID = 0x04
CTRE_PDP_API_ID = 0x052

REV_PDH_TYPE_ID = 0x08
REV_PDH_MANUF_ID = 0x05
REV_PDH_API_ID = 0x064

class PDIdentifier(Listener):
    def __init__(self, notifier):
        self.notifier = notifier
        self.detected = False

    def on_message_received(self, msg):
        if self.detected or not msg.is_extended_id:
            return

        can_id = msg.arbitration_id
        dev_type = (can_id >> 24) & 0x1F
        manuf = (can_id >> 16) & 0xFF
        api_id = (can_id >> 6) & 0x3FF
        dev_num = can_id & 0x3F

        if (dev_type, manuf, api_id) == (CTRE_PDP_TYPE_ID, CTRE_PDP_MANUF_ID, CTRE_PDP_API_ID):
            print(f"✅ CTRE PDP detected (Device Number: {dev_num})")
            self.detected = True
            self.notifier.stop()
        elif (dev_type, manuf, api_id) == (REV_PDH_TYPE_ID, REV_PDH_MANUF_ID, REV_PDH_API_ID):
            print(f"✅ REV PDH detected (Device Number: {dev_num})")
            self.detected = True
            self.notifier.stop()

def main():
    bus = can.interface.Bus(bustype="canalystii", channel=0, bitrate=1000000)
    listener = PDIdentifier(None)
    notifier = Notifier(bus, [listener])
    listener.notifier = notifier

    try:
        print("🔍 Listening for Power Distribution device...")
        while not listener.detected:
            pass
    except KeyboardInterrupt:
        print("\nExiting...")
        notifier.stop()

main()
