"""Display-only SO-101 UI; this module never opens cameras or motor buses.

Conda's Tk can use X11 core fonts without Xft, even when fontconfig finds Noto.
Render all application text with Pillow/FreeType so Chinese glyphs and their
layout use the same font metrics. Native Tk buttons retain focus and activation.
"""

from functools import lru_cache
from pathlib import Path
import queue
import time
import tkinter as tk

from PIL import Image, ImageDraw, ImageFont, ImageTk

from so101_session import JOINTS

BG, PANEL, INK, MUTED = "#edf2f7", "#ffffff", "#15283e", "#53677d"
BLUE, RED = "#185db2", "#b42332"
JOINT_TITLES = ("底座 (°)", "大臂 (°)", "肘部 (°)", "腕部俯仰 (°)", "腕部旋转 (°)", "夹爪 (点)")
STATE_TITLES = {
    "REFERENCE": "采样腕部参照", "ALIGN": "等待双臂对齐", "SETUP_FOLLOW": "设置初始姿态",
    "START": "准备开始", "READY": "初始姿态就绪", "TELEOP": "正在跟随",
    "MANUAL": "手动整理 · 未录制", "RECORDING": "正在录制", "REVIEW": "确认本段结果",
    "SAVING": "正在处理数据", "COUNTDOWN": "复位倒计时", "RETURNING": "正在复位",
    "FIRST_REVIEW": "检查首条演示", "DONE": "采集完成", "LIMIT_PAUSED": "越界暂停 · 等待手动恢复",
    "FAULT_PAUSED": "故障暂停 · 支撑双臂 · R 重试",
}


class Typography:
    def __init__(self):
        candidates = (("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 2),
                      ("/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf", 0))
        self.path, self.index = next(((p, i) for p, i in candidates if Path(p).is_file()), (None, 0))
        if self.path is None:
            raise RuntimeError("Preview requires an installed Chinese font: Noto Sans CJK or Droid Sans Fallback")

    @lru_cache(maxsize=16)
    def font(self, size):
        return ImageFont.truetype(self.path, size, index=self.index)

    @lru_cache(maxsize=2048)
    def tile(self, text, size=18, color=INK):
        font = self.font(size)
        left, top, right, bottom = font.getbbox(text or " ")
        ascent, descent = font.getmetrics()
        # Keep a stable baseline; include actual glyph extents and advance width.
        width = max(right, int(font.getlength(text or " ") + 1)) - min(0, left) + 2
        height = max(ascent + descent, bottom) + 2
        result = Image.new("RGBA", (width, height))
        ImageDraw.Draw(result).text((1 - min(0, left), 0), text, font=font, fill=color)
        return result

    def paste(self, image, text, box, size=18, color=INK, center=False):
        tile = self.tile(text, size, color)
        x, y, width, height = box
        if tile.width > width:
            raise ValueError(f"UI cell too narrow for {text!r}: {width} < {tile.width}")
        x += (width - tile.width) // 2 if center else 0
        image.paste(tile, (x, y + (height - tile.height) // 2), tile)

    def wrap(self, text, width, size=18):
        # Character-aware wrapping also breaks long paths and diagnostic tokens.
        font = self.font(size)
        lines = []
        for paragraph in text.split("\n"):
            line, advance = "", 0.0
            for char in paragraph:
                step = font.getlength(char)
                if line and advance + step > width:
                    lines.append(line)
                    line, advance = "", 0.0
                line += char
                advance += step
            lines.append(line)
        return lines


class TextImage(tk.Label):
    def __init__(self, master, typography, text="", size=18, color=INK, **kwargs):
        super().__init__(master, bd=0, bg=kwargs.pop("bg", PANEL), **kwargs)
        self.typography, self.size, self.color = typography, size, color
        self.set(text)

    def set(self, text, color=None):
        tile = self.typography.tile(text, self.size, color or self.color)
        picture = Image.new("RGB", tile.size, self.cget("bg"))
        picture.paste(tile, (0, 0), tile)
        self.picture = ImageTk.PhotoImage(picture, master=self)
        self.configure(image=self.picture)


class MessagePanel(tk.Frame):
    """Bounded, wrapped message area; longer faults remain accessible by scroll."""
    def __init__(self, master, typography):
        super().__init__(master, bg=PANEL)
        self.typography, self.text, self.color = typography, "初始化中", INK
        self.canvas = tk.Canvas(self, height=76, bg=PANEL, highlightthickness=0)
        scrollbar = tk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.item = self.canvas.create_image(8, 4, anchor="nw")
        self.canvas.bind("<Configure>", lambda _event: self.render())
        self.canvas.bind("<Button-4>", lambda _event: self.canvas.yview_scroll(-2, "units"))
        self.canvas.bind("<Button-5>", lambda _event: self.canvas.yview_scroll(2, "units"))
        self.canvas.bind("<MouseWheel>", lambda event: self.canvas.yview_scroll(-1 if event.delta > 0 else 1, "units"))

    def set(self, text, color, reset_scroll=False):
        if (text, color) != (self.text, self.color):
            self.text, self.color = text, color
            self.render()
        if reset_scroll:
            self.canvas.yview_moveto(0)

    def render(self):
        width = max(100, self.canvas.winfo_width() - 20)
        lines = self.typography.wrap(self.text, width - 4)
        picture = Image.new("RGB", (width, max(28, len(lines) * 28)), PANEL)
        for i, line in enumerate(lines):
            self.typography.paste(picture, line, (0, i * 28, width, 28), color=self.color)
        self.picture = ImageTk.PhotoImage(picture, master=self)
        self.canvas.itemconfigure(self.item, image=self.picture)
        self.canvas.configure(scrollregion=(0, 0, width + 16, picture.height + 8))


class PreviewWindow:
    def __init__(self, frames, commands, ready, stop, closed, mode="record", camera_names=("top", "wrist"), reuse_wrist=False):
        self.frames, self.commands, self.ready, self.stop, self.closed = frames, commands, ready, stop, closed
        self.mode, self.current_state, self.packet = mode, "", None
        self.last_packet, self.last_table, self.last_status = None, None, None
        self.window = window = tk.Tk()
        self.typography = Typography()
        window.title("SO101 | 双臂控制与采集")
        window.configure(bg=BG)
        width = min(1440, window.winfo_screenwidth() - 60)
        height = min(980 if camera_names else 700, window.winfo_screenheight() - 90)
        window.geometry(f"{width}x{height}+20+20")
        window.minsize(1040, 760 if camera_names else 640)
        window.protocol("WM_DELETE_WINDOW", stop.set)
        window.columnconfigure(0, weight=1)
        window.rowconfigure(1, weight=1)

        header = tk.Frame(window, bg=BG)
        header.grid(row=0, column=0, sticky="ew", padx=16, pady=(10, 6))
        title = {"setup": "腕部映射与初始姿态", "follow": "逐关节跟随检查",
                 "teleop": "映射跟随与复位", "record": "双相机演示采集"}[mode]
        TextImage(header, self.typography, title, size=22, bg=BG).pack(side="left")
        self.state_label = TextImage(header, self.typography, "初始化中", color=BLUE, bg=BG)
        self.state_label.pack(side="right")

        self.camera_area = tk.Frame(window, bg=BG)
        self.camera_area.grid(row=1, column=0, sticky="nsew", padx=16)
        self.camera_area.rowconfigure(0, weight=1)
        self.images, self.camera_frames = {}, {}
        for column, name in enumerate(camera_names):
            self.camera_area.columnconfigure(column, weight=1, uniform="camera")
            panel = tk.Frame(self.camera_area, bg=PANEL)
            panel.grid(row=0, column=column, sticky="nsew", padx=(0, 8) if column == 0 and len(camera_names) > 1 else 0)
            panel.columnconfigure(0, weight=1)
            panel.rowconfigure(1, weight=1)
            TextImage(panel, self.typography, "全局相机 · Gemini335" if name == "top" else "腕部相机 · UGREEN").grid(row=0, column=0, pady=3)
            canvas = tk.Canvas(panel, bg="#142131", highlightthickness=0, width=1, height=1)
            canvas.grid(row=1, column=0, sticky="nsew")
            item = canvas.create_image(0, 0, anchor="center")
            self.images[name] = (canvas, item)
            canvas.bind("<Configure>", lambda _event, n=name: self.draw_camera(n))
        if not camera_names:
            self.camera_area.grid_remove()

        self.metadata = TextImage(window, self.typography, "腕转偏移：未采样    臂关节：度    夹爪：0–100 点", size=17, color=MUTED, bg=BG, anchor="w")
        self.metadata.grid(row=2, column=0, sticky="ew", padx=16, pady=4)
        self.table = tk.Canvas(window, height=266, bg=PANEL, highlightthickness=0)
        self.table.grid(row=3, column=0, sticky="ew", padx=16)
        self.table_item = self.table.create_image(0, 0, anchor="nw")
        self.table.bind("<Configure>", lambda _event: self.draw_table())
        self.details = MessagePanel(window, self.typography)
        self.details.grid(row=4, column=0, sticky="ew", padx=16, pady=(6, 4))

        controls = tk.Frame(window, bg=BG)
        controls.grid(row=5, column=0, sticky="ew", padx=16, pady=4)
        controls.columnconfigure(0, weight=1)
        actions = tk.Frame(controls, bg=BG)
        actions.grid(row=0, column=0, sticky="ew")
        if mode == "follow":
            rows = ((('s', '开始跟随'), ('m', '越界暂停后恢复')),)
        elif mode == "setup":
            setup_row = (('a', '开启映射跟随'), ('h', '保存当前姿态为起点'))
            if not reuse_wrist:
                setup_row = (('c', '采样腕转参照'), *setup_row)
            rows = (setup_row,
                    (('s', '开始跟随'), ('r', '空爪复位到初始姿态'), ('m', '恢复跟随')))
        elif mode == "teleop":
            rows = ((('s', '开始跟随'), ('r', '空爪复位到初始姿态'), ('m', '恢复跟随')),)
        else:
            rows = ((('s', '开始 / 下一段'), ('n', '成功保存并复位'), ('r', '空爪复位 / 作废重录')),
                    (('m', '恢复 / 手动整理'), ('v', '首条检查通过')))
        self.buttons = {}
        for row, entries in enumerate(rows):
            for column, (key, text) in enumerate(entries):
                actions.columnconfigure(column, weight=1, uniform="button")
                button = self.make_button(actions, key, text)
                button.grid(row=row, column=column, sticky="ew", padx=(0, 8), pady=3)
        self.make_button(controls, 'q', '停止并退出', danger=True).grid(row=0, column=1, sticky="ns", padx=(6, 0), pady=3)
        self.accepted_keys = set(self.buttons)

        help_text = ("按 s 开始跟随；越界暂停后对齐并按 m 恢复。本模式不录制、不复位。" if mode == "follow" else
                     "复位前松开物体、让开双臂路径；倒计时中松开主臂。复位从当前位置开始，程序不自动避障。")
        footer = tk.Frame(window, bg=BG)
        footer.grid(row=6, column=0, sticky="ew", padx=16, pady=(0, 8))
        for line in (help_text, "快捷键仅在本窗口有效。开始跟随时握住主臂；q / Esc / 关闭窗口将释放双臂力矩，请支撑双臂。"):
            TextImage(footer, self.typography, line, size=16, color=MUTED, bg=BG).pack(anchor="w")

        self.down, self.release_tasks = set(), {}
        window.bind("<KeyPress>", self.pressed)
        window.bind("<KeyRelease>", self.released)
        window.bind("<FocusOut>", self.unfocused)
        window.after(0, self.refresh)
        window.after(100, ready.set)

    def make_button(self, parent, key, text, danger=False):
        button = tk.Button(parent, text=f"{key} · {text}", command=lambda: self.submit(key),
                           bg=RED if danger else PANEL, activebackground=RED if danger else PANEL,
                           relief="flat", bd=0, padx=12, pady=6, cursor="hand2", takefocus=True,
                           highlightthickness=2, highlightbackground=BG, highlightcolor=BLUE)
        tile = self.typography.tile(f"{key.upper()} · {text}", 18, "#ffffff" if danger else INK)
        # Flatten alpha here: core-X11 Tk can stipple partially transparent
        # PhotoImage edges, especially white glyphs on the red stop button.
        padded = Image.new("RGB", (tile.width + 24, tile.height + 12), RED if danger else PANEL)
        padded.paste(tile, (12, 6), tile)
        button.picture = ImageTk.PhotoImage(padded, master=button)
        button.configure(image=button.picture)
        self.buttons[key] = button
        return button

    def submit(self, key):
        if key == "q":
            self.stop.set()
        else:
            try:
                self.commands.put_nowait((key, time.monotonic(), self.current_state))
            except queue.Full:
                pass

    def pressed(self, event):
        key = "q" if event.keysym == "Escape" else event.keysym.lower()
        if key not in self.accepted_keys:
            return
        pending = self.release_tasks.pop(key, None)
        if pending is not None:
            self.window.after_cancel(pending)
        if key not in self.down:
            self.down.add(key)
            self.submit(key)
        return "break"

    def released(self, event):
        key = "q" if event.keysym == "Escape" else event.keysym.lower()
        if key not in self.accepted_keys:
            return
        def clear():
            self.release_tasks.pop(key, None)
            self.down.discard(key)
        # Preserve X11 repeat suppression and focus-only command handling.
        self.release_tasks[key] = self.window.after(20, clear)
        return "break"

    def unfocused(self, _event):
        self.down.clear()
        for task in self.release_tasks.values():
            self.window.after_cancel(task)
        self.release_tasks.clear()

    def draw_camera(self, name):
        if name not in self.camera_frames:
            return
        canvas, item = self.images[name]
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if min(width, height) < 2:
            return
        picture = self.camera_frames[name].copy()
        picture.thumbnail((width, height), Image.Resampling.BILINEAR)
        canvas.picture = ImageTk.PhotoImage(picture, master=canvas)
        canvas.coords(item, width / 2, height / 2)
        canvas.itemconfigure(item, image=canvas.picture)

    def draw_table(self):
        packet = self.packet or {}
        width = self.table.winfo_width()
        if width < 950:
            return
        rows = []
        for key, title in (("leader", "主臂实测"), ("requested", "从臂映射请求"),
                           ("target", "从臂已发送目标"), ("follower", "从臂实测")):
            pose = packet.get(key)
            rows.append((title, [f"{pose[j]:.2f}" if pose else "—" for j in JOINTS]))
        for role, title in (("leader", "主臂初始姿态"), ("follower", "从臂初始姿态")):
            pose = (packet.get("reference") or {}).get(role)
            rows.append((title, [f"{pose[j]:.2f}" if pose else "—" for j in JOINTS]))
        ranges = packet.get("common_ranges")
        rows.append(("共同范围（主臂）", [f"{ranges[j][0]:.1f} ~ {ranges[j][1]:.1f}" if ranges else "—" for j in JOINTS]))
        hold = packet.get("pause_hold")
        rows.append(("暂停时从臂保持", [f"{hold[j]:.2f}" if hold else "—" for j in JOINTS]))
        signature = (width, str(rows), self.current_state)
        if signature == self.last_table:
            return
        self.last_table = signature
        picture = Image.new("RGB", (width, 266), PANEL)
        draw = ImageDraw.Draw(picture)
        draw.rectangle((0, 0, width, 33), fill="#dbe7f4")
        edges = [0, 174] + [174 + round((width - 174) * i / 6) for i in range(1, 7)]
        for column, title in enumerate(("读数 / 关节", *JOINT_TITLES)):
            self.typography.paste(picture, title, (edges[column] + 6, 0, edges[column + 1] - edges[column] - 12, 34), size=17, center=column > 0)
        for row, (title, values) in enumerate(rows):
            y = 34 + row * 29
            bg = "#fff0f0" if row == 7 and hold else "#f1f6fc" if row % 2 == 0 else PANEL
            draw.rectangle((0, y, width, y + 28), fill=bg)
            for column, text in enumerate((title, *values)):
                self.typography.paste(picture, text, (edges[column] + 6, y, edges[column + 1] - edges[column] - 12, 29),
                                      size=16 if row == 6 else 18, color=RED if row == 7 and hold else INK,
                                      center=column > 0)
        for x in edges[1:-1]:
            draw.line((x, 0, x, 265), fill="#d8e2ed")
        self.table.picture = ImageTk.PhotoImage(picture, master=self.table)
        self.table.itemconfigure(self.table_item, image=self.table.picture)

    def refresh(self):
        if self.closed.is_set():
            self.window.destroy()
            return
        packet = None
        try:
            while True:
                packet = self.frames.get_nowait()
        except queue.Empty:
            pass
        if packet is not None:
            previous_state = self.current_state
            self.packet = packet
            self.last_packet, self.current_state = packet["time"], packet["state"]
            if not packet.get("observation_valid", True):
                self.camera_frames.clear()
                for canvas, item in self.images.values():
                    canvas.itemconfigure(item, image="")
            for name, frame in packet["images"].items():
                if name in self.images:
                    self.camera_frames[name] = Image.fromarray(frame)
                    self.draw_camera(name)
            self.draw_table()
            offset = packet.get("wrist_offset_deg")
            offset_text = f"{offset:+.2f}°" if offset is not None else "未采样"
            self.metadata.set(f"腕转偏移：{offset_text}    已完成复位：{packet.get('reset_completed_count', 0)} 次    臂关节：度    夹爪：0–100 点")
            warnings = packet.get("boundary_warnings", [])
            message = packet["message"] + ("\n" + "\n".join(warnings) if warnings else "")
            self.details.set(message, RED if self.current_state in ("LIMIT_PAUSED", "FAULT_PAUSED") else INK,
                             reset_scroll=previous_state != self.current_state)
        age = time.monotonic() - self.last_packet if self.last_packet is not None else 0
        label = STATE_TITLES.get(self.current_state, self.current_state or "初始化中")
        if age > 1:
            label += f" · 画面 {age:.1f} 秒未更新"
        status = (label, RED if self.current_state in ("LIMIT_PAUSED", "FAULT_PAUSED") or age > 1 else BLUE)
        if status != self.last_status:
            self.state_label.set(*status)
            self.last_status = status
        self.window.after(50, self.refresh)

    def run(self):
        try:
            self.window.mainloop()
        finally:
            self.stop.set()


def preview_window(frames, commands, ready, stop, closed, mode="record", camera_names=("top", "wrist"), reuse_wrist=False):
    PreviewWindow(frames, commands, ready, stop, closed, mode, camera_names, reuse_wrist).run()
