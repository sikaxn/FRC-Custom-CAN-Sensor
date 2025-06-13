import tkinter as tk

GRID_ROWS = 16
GRID_COLS = 16
BUTTON_SIZE = 25

class ROISelector(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("VL53L1X ROI Area Selector")
        self.geometry(f"{GRID_COLS * BUTTON_SIZE + 200}x{GRID_ROWS * BUTTON_SIZE + 60}")
        self.point1 = None
        self.point2 = None
        self.buttons = []
        self.create_widgets()

    def create_widgets(self):
        for y in range(GRID_ROWS):
            row = []
            for x in range(GRID_COLS):
                btn = tk.Button(self, width=2, height=1,
                                command=lambda x=x, y=y: self.handle_click(x, y))
                btn.grid(row=y, column=x, padx=1, pady=1)
                row.append(btn)
            self.buttons.append(row)

        self.output_text = tk.Text(self, height=5, width=30)
        self.output_text.place(x=GRID_COLS * BUTTON_SIZE + 10, y=10)

        tk.Button(self, text="Clear", command=self.clear).place(
            x=GRID_COLS * BUTTON_SIZE + 10, y=120)

    def handle_click(self, x, y):
        if self.point1 is None:
            self.point1 = (x, y)
            self.highlight_point(x, y, "blue")
        elif self.point2 is None:
            self.point2 = (x, y)
            self.draw_rectangle()
            self.compute_roi()
        else:
            self.clear()
            self.point1 = (x, y)
            self.highlight_point(x, y, "blue")

    def highlight_point(self, x, y, color):
        self.buttons[y][x].configure(bg=color)

    def draw_rectangle(self):
        x1, y1 = self.point1
        x2, y2 = self.point2
        for y in range(min(y1, y2), max(y1, y2) + 1):
            for x in range(min(x1, x2), max(x1, x2) + 1):
                self.buttons[y][x].configure(bg="green")

    def compute_roi(self):
        x1, y1 = self.point1
        x2, y2 = self.point2
        x_center = (x1 + x2) // 2
        y_center = (y1 + y2) // 2
        roi_center = y_center * 16 + x_center
        width = abs(x2 - x1) + 1
        height = abs(y2 - y1) + 1
        self.output_text.insert(tk.END, f"ROI center zone: {roi_center}\n")
        self.output_text.insert(tk.END, f"Center (x={x_center}, y={y_center})\n")
        self.output_text.insert(tk.END, f"ROI size: {width} × {height} zones\n")

    def clear(self):
        for row in self.buttons:
            for btn in row:
                btn.configure(bg="SystemButtonFace")
        self.point1 = None
        self.point2 = None
        self.output_text.delete("1.0", tk.END)

if __name__ == "__main__":
    app = ROISelector()
    app.mainloop()
