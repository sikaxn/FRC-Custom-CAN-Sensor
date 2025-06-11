import can
from can.notifier import Notifier, Listener

# Constants for identification
CTRE_PDP_TYPE_ID = 0x08
CTRE_PDP_MANUF_ID = 0x04
CTRE_PDP_API_VOLTAGE = 0x052  # Voltage only
CTRE_PDP_API_CURRENT = 0x05D  # Total current


REV_PDH_TYPE_ID = 0x08
REV_PDH_MANUF_ID = 0x05
REV_PDH_API_ID = 0x064  # Voltage + Current

class PDIdentifier(Listener):
    def __init__(self, notifier):
        self.notifier = notifier
        self.detected = False
        self.device_type = None
        self.manufacturer = None
        self.device_number = None

    def on_message_received(self, msg):
        if self.detected or not msg.is_extended_id:
            return

        can_id = msg.arbitration_id
        dev_type = (can_id >> 24) & 0x1F
        manuf = (can_id >> 16) & 0xFF
        api_id = (can_id >> 6) & 0x3FF
        dev_num = can_id & 0x3F

        if (dev_type, manuf, api_id) == (CTRE_PDP_TYPE_ID, CTRE_PDP_MANUF_ID, CTRE_PDP_API_VOLTAGE):
            print(f"✅ CTRE PDP detected (Device Number: {dev_num})")
        elif (dev_type, manuf, api_id) == (REV_PDH_TYPE_ID, REV_PDH_MANUF_ID, REV_PDH_API_ID):
            print(f"✅ REV PDH detected (Device Number: {dev_num})")
        else:
            return

        self.device_type = dev_type
        self.manufacturer = manuf
        self.device_number = dev_num
        self.detected = True
        self.notifier.stop()

class PowerMonitor(Listener):
    def __init__(self, dev_type, manuf, dev_num):
        self.dev_type = dev_type
        self.manuf = manuf
        self.dev_num = dev_num
        self.voltage = None
        self.current = None

    def on_message_received(self, msg):
        if not msg.is_extended_id:
            return

        can_id = msg.arbitration_id
        dev_type = (can_id >> 24) & 0x1F
        manuf = (can_id >> 16) & 0xFF
        api_id = (can_id >> 6) & 0x3FF
        dev_num = can_id & 0x3F

        if (dev_type, manuf, dev_num) != (self.dev_type, self.manuf, self.dev_num):
            return

        if dev_type == CTRE_PDP_TYPE_ID and manuf == CTRE_PDP_MANUF_ID:
            if api_id == CTRE_PDP_API_VOLTAGE:
                self.voltage = msg.data[6] * 0.05 + 4.0

            elif api_id == CTRE_PDP_API_CURRENT:
                raw_current = (msg.data[1] << 4) | (msg.data[2] >> 4)
                self.current = raw_current * 0.125

            if self.voltage is not None and self.current is not None:
                print(f"🔋 CTRE PDP → Voltage: {self.voltage:.2f} V | Total Current: {self.current:.2f} A")
                self.voltage = None
                self.current = None

            
        elif dev_type == REV_PDH_TYPE_ID and manuf == REV_PDH_MANUF_ID and api_id == REV_PDH_API_ID:
            # Voltage is a 12-bit value: lower 8 bits in data[0], upper 4 bits in data[1]
            v_bus_raw = ((msg.data[1] & 0x0F) << 8) | msg.data[0]
            voltage = v_bus_raw * 0.0078125  # Scale = 0.0078125

            # Total current is in data[4], scaled by 2.0
            current = msg.data[4] * 2.0

            print(f"🔋 REV PDH → Voltage: {voltage:.2f} V | Total Current: {current:.2f} A")


def main():
    bus = can.interface.Bus(bustype="canalystii", channel=0, bitrate=1000000)
    print("🔍 Listening for CTRE PDP or REV PDH...")

    identifier = PDIdentifier(None)
    identifier_notifier = Notifier(bus, [identifier])
    identifier.notifier = identifier_notifier

    try:
        while not identifier.detected:
            pass

        monitor = PowerMonitor(identifier.device_type, identifier.manufacturer, identifier.device_number)
        data_notifier = Notifier(bus, [monitor])
        print("📡 Now receiving voltage and current...")

        while True:
            pass

    except KeyboardInterrupt:
        print("❎ Exiting...")
        identifier_notifier.stop()
        if 'data_notifier' in locals():
            data_notifier.stop()

if __name__ == "__main__":
    main()
