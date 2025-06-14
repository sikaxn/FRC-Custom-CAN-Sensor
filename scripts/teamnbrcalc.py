import tkinter as tk
from tkinter import ttk

class FRCApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("FRC 队伍编号计算器")
        self.geometry("600x260")

        tk.Label(self, text="输入你的编号:", font=("Arial", 14)).pack(pady=(10, 0))

        self.entry = tk.Entry(self, font=("Arial", 16), justify="center")
        self.entry.pack(pady=5)

        go_button = tk.Button(self, text="Go", font=("Arial", 14), command=self.start_progress)
        go_button.pack(pady=5)

        self.progress = ttk.Progressbar(self, mode='determinate', length=400)
        self.progress.pack(pady=10)
        self.progress['maximum'] = 100
        self.progress['value'] = 0

        # Centered, styled Text widget
        self.output_text = tk.Text(
            self,
            height=1,
            width=40,
            font=("Arial", 20),
            bd=0,
            bg=self.cget('bg'),
            wrap="none"
        )
        self.output_text.tag_configure("center", justify="center")
        self.output_text.tag_configure("normal", foreground="blue", font=("Arial", 20))
        self.output_text.tag_configure("final", foreground="red", font=("Arial", 20))
        self.output_text.tag_configure("team", foreground="red", font=("Arial", 26, "bold"))
        self.output_text.pack(pady=20)
        self.output_text.configure(state="disabled")

        self.progress_value = 0
        self.message_index = 0

        self.messages = [
            "正在努力计算您的队伍编号...",
            "请稍等，计算机器人定位...",
            "AI 正在分析电控系统...",
            "下载比赛数据中...",
            "正在扫描您的队伍历史...",
            "查找最强战队编号中...",
            "同步Git中...",
            "加载 Java 代码中...",
            "激活 视觉 模块...",
            "最终确认队伍编号中..."
        ]

    def start_progress(self):
        self.progress_value = 0
        self.progress['value'] = 0
        self.message_index = 0
        self.clear_output()
        self.update_progress()
        self.update_message()

    def clear_output(self):
        self.output_text.configure(state="normal")
        self.output_text.delete("1.0", tk.END)
        self.output_text.tag_add("center", "1.0", "end")
        self.output_text.configure(state="disabled")

    def update_progress(self):
        if self.progress_value <= 100:
            self.progress['value'] = self.progress_value
            self.progress_value += 1
            self.after(100, self.update_progress)
        else:
            self.show_result()

    def update_message(self):
        if self.progress_value <= 100:
            msg = self.messages[self.message_index % len(self.messages)]
            self.output_text.configure(state="normal")
            self.output_text.delete("1.0", tk.END)
            self.output_text.insert(tk.END, msg, "normal")
            self.output_text.tag_add("center", "1.0", "end")
            self.output_text.configure(state="disabled")
            self.message_index += 1
            self.after(700, self.update_message)

    def show_result(self):
        number = self.entry.get().strip()
        self.output_text.configure(state="normal")
        self.output_text.delete("1.0", tk.END)
        if number.isdigit():
            self.output_text.insert(tk.END, "你的队伍编号是 ", "final")
            self.output_text.insert(tk.END, number, "team")
        else:
            self.output_text.insert(tk.END, "请输入有效的数字", "final")
        self.output_text.tag_add("center", "1.0", "end")
        self.output_text.configure(state="disabled")

if __name__ == "__main__":
    app = FRCApp()
    app.mainloop()
