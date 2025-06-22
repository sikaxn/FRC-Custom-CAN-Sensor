import can
import tkinter as tk
from tkinter import ttk, IntVar, messagebox
import time
import random

# === CAN Constants ===
CMD_CLEAR = 0x01
CMD_CURSOR = 0x02
CMD_WRITE = 0x03
CMD_WRITE_NEXT = 0x04
CMD_ACK = 0xAA

DEVICE_ID = 0x0A
MANUFACTURER_ID = 0x08
API_ID = 0x187
ACK_API_ID = 0x188
DEVICE_NUMBER = 33

CAN_ID = ((DEVICE_ID << 24) | (MANUFACTURER_ID << 16) | (API_ID << 6) | DEVICE_NUMBER)
ACK_ID = ((DEVICE_ID << 24) | (MANUFACTURER_ID << 16) | (ACK_API_ID << 6) | DEVICE_NUMBER)

bus = can.interface.Bus(channel=0, bustype='canalystii', bitrate=1000000)

# === GUI Class ===
class LCDGui:
    def __init__(self, root):
        self.root = root
        self.root.title("LCD CAN Sender")

        self.row = IntVar(value=0)
        self.col = IntVar(value=0)
        self.status = tk.StringVar(value="Ready")

        self.build_ui()
        self.root.after(100, self.listen_debug)

    def build_ui(self):
        frame = ttk.LabelFrame(self.root, text="LCD Controls")
        frame.grid(padx=10, pady=10)

        ttk.Label(frame, text="Row:").grid(row=0, column=0)
        ttk.Spinbox(frame, from_=0, to=3, textvariable=self.row, width=5).grid(row=0, column=1)

        ttk.Label(frame, text="Col:").grid(row=1, column=0)
        ttk.Spinbox(frame, from_=0, to=19, textvariable=self.col, width=5).grid(row=1, column=1)

        ttk.Label(frame, text="Text:").grid(row=2, column=0, sticky="n")
        self.textbox = tk.Text(frame, width=25, height=6)
        self.textbox.grid(row=2, column=1, columnspan=2)

        ttk.Button(frame, text="Send", command=self.send_multiline_text).grid(row=3, column=0, pady=5)
        ttk.Button(frame, text="Clear Screen", command=self.send_clear).grid(row=3, column=1, pady=5)
        ttk.Button(frame, text="Clear and Send", command=self.clear_and_send).grid(row=3, column=2, pady=5)

        ttk.Label(frame, textvariable=self.status).grid(row=4, column=0, columnspan=3)

    def wait_for_ack(self, tid, timeout=0.2):
        start = time.time()
        while time.time() - start < timeout:
            msg = bus.recv(timeout)
            if msg and msg.arbitration_id == ACK_ID and msg.data[0] == CMD_ACK and msg.data[1] == tid:
                return True
        return False

    def send_text(self, row, col, txt):
        tid = random.randint(1, 254)
        data = txt.encode("ascii", errors="ignore")
        chunks = [data[i:i+3] for i in range(0, len(data), 3)]

        for attempt in range(3):
            self.status.set(f"Sending attempt {attempt+1}...")
            for i, chunk in enumerate(chunks):
                isFinal = 1 if i == len(chunks) - 1 else 0
                payload = list(chunk)
                if len(payload) < 3 and isFinal:
                    payload += [32] * (3 - len(payload))
                else:
                    while len(payload) < 3:
                        payload.append(0)
                cmd = CMD_WRITE if i == 0 else CMD_WRITE_NEXT
                frame_col = col if i == 0 else 0
                frame = [cmd, isFinal, row, frame_col] + payload + [tid]
                msg = can.Message(arbitration_id=CAN_ID, data=frame, is_extended_id=True)
                bus.send(msg)
                time.sleep(0.01)

            if self.wait_for_ack(tid):
                self.status.set("✅ Message acknowledged")
                return True
            else:
                self.status.set("⚠️ No ACK, retrying...")
                time.sleep(0.05)

        self.status.set("❌ Failed to receive ACK")
        return False

    def send_multiline_text(self):
        text = self.textbox.get("1.0", "end").strip()
        base_row = self.row.get()
        col = self.col.get()
        chunks = text.split("\n")

        for i, chunk in enumerate(chunks):
            if base_row + i < 4:
                success = self.send_text(base_row + i, col, chunk[:20])
                if not success:
                    break

    def send_clear(self):
        frame = [CMD_CLEAR, 0, 0, 0, 0, 0, 0, random.randint(1, 254)]
        msg = can.Message(arbitration_id=CAN_ID, data=frame, is_extended_id=True)
        bus.send(msg)
        self.status.set("🧹 Clear command sent")

    def clear_and_send(self):
        self.send_clear()
        time.sleep(0.1)
        self.send_multiline_text()

    def listen_debug(self):
        msg = bus.recv(timeout=0.01)
        if msg and msg.arbitration_id == CAN_ID:
            print(f"[DEBUG RX] ID=0x{msg.arbitration_id:X} DATA={list(msg.data)}")
        self.root.after(100, self.listen_debug)

if __name__ == "__main__":
    root = tk.Tk()
    app = LCDGui(root)
    root.mainloop()
