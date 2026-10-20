"""Terminal trial sequencing; no robot, policy or camera imports."""

from __future__ import annotations

import csv
import os
import select
import threading
import time
from datetime import datetime


class TrialControls:
    def __init__(self):
        self.shutdown = threading.Event()
        self.segment_stop = threading.Event()
        self._enter = threading.Event()
        self._lock = threading.RLock()
        self.phase = "STARTUP"
        self.end_reason = "time_limit"

    def set_phase(self, phase):
        with self._lock:
            self.phase = phase
            self._enter.clear()

    def enter(self):
        with self._lock:
            if self.shutdown.is_set():
                return False
            if self.phase in ("WAIT_HOME", "READY"):
                self.phase = "BUSY"
                self._enter.set()
                return True
            if self.phase == "RUNNING":
                self.phase = "STOPPING"
                self.end_reason = "enter"
                self.segment_stop.set()
                return True
            # Enter during startup/reset/cooldown never queues a later movement.
            return False

    def quit(self):
        with self._lock:
            self.shutdown.set()
            self.segment_stop.set()
            self._enter.set()
            self.end_reason = "quit"

    def wait_enter(self, phase, prompt, emit):
        self.set_phase(phase)
        emit(prompt)
        while not self.shutdown.is_set():
            if self._enter.wait(.05):
                return not self.shutdown.is_set()
        return False

    def begin_run(self):
        with self._lock:
            if self.shutdown.is_set():
                return False
            self.end_reason = "time_limit"
            self.segment_stop.clear()
            self.phase = "RUNNING"
            return True


class EnterReader:
    """Read actual newline keys, including blank lines; EOF never creates an Enter."""

    def __init__(self, stream, on_enter, on_quit, emit=print):
        self.stream, self.on_enter, self.on_quit, self.emit = stream, on_enter, on_quit, emit
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, name="trial-enter-reader", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)

    def _line(self, raw):
        line = raw.decode(errors="replace").strip()
        if not line:
            if not self.on_enter():
                self.emit("当前正在连接、复位或等待；本次回车不会排队执行。")
        elif line.lower() in ("q", "quit", "exit", "/stop"):
            self.on_quit()
            return False
        else:
            self.emit("使用 Enter 进行下一步，或输入 q 后回车退出；成功/失败请另行记录。")
        return True

    def _run(self):
        pending = b""
        try:
            fd = self.stream.fileno()
            while not self._stop.is_set():
                ready, _, _ = select.select([fd], [], [], .05)
                if not ready:
                    continue
                chunk = os.read(fd, 4096)
                if self._stop.is_set():
                    return
                if not chunk:
                    # A trailing partial command may quit, but must not start a move.
                    self.on_quit()
                    return
                pending += chunk
                while b"\n" in pending:
                    line, pending = pending.split(b"\n", 1)
                    if not self._line(line):
                        return
        except (OSError, ValueError):
            if not self._stop.is_set():
                self.on_quit()


class RoundLog:
    FIELDS = ("round", "started_at", "ended_at", "duration_s", "end_reason")

    def __init__(self, stream):
        self.stream = stream
        self.writer = csv.DictWriter(stream, fieldnames=self.FIELDS)
        self.writer.writeheader()
        self.stream.flush()

    def add(self, row):
        self.writer.writerow(row)
        self.stream.flush()


class TrialLoop:
    """The only caller of backend motion methods; the input thread only signals."""

    def __init__(self, backend, controls, round_log, scene_reset_s=10, emit=print, max_rounds=None):
        self.backend, self.controls, self.round_log = backend, controls, round_log
        self.scene_reset_s, self.emit = scene_reset_s, emit
        if max_rounds is not None and max_rounds < 1:
            raise ValueError("max_rounds must be positive or None")
        self.max_rounds = max_rounds
        self.round_number = 0

    def _reset(self):
        self.controls.set_phase("RESETTING")
        self.emit("正在复位到采集初始姿态；Ctrl+C 可中止并退出。")
        self.backend.reset_home()
        self.emit("复位指令完成，从臂保持初始姿态。")

    def _scene_wait(self):
        self.controls.set_phase("COOLDOWN")
        deadline = time.monotonic() + self.scene_reset_s
        last_seconds = None
        while not self.controls.shutdown.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            seconds = int(remaining) + 1
            if seconds != last_seconds:
                self.emit(f"场景复位时间：剩余 {seconds} 秒。请另行记录本轮成功/失败。")
                last_seconds = seconds
            self.controls.shutdown.wait(min(.1, remaining))

    def run(self):
        c = self.controls
        try:
            if not c.wait_enter("WAIT_HOME", "按 Enter 连接从臂并复位；此时尚未连接硬件。", self.emit):
                return
            self.emit("正在加载模型、连接从臂和相机……")
            self.backend.connect()
            if c.shutdown.is_set():
                return
            self._reset()
            while not c.shutdown.is_set():
                total = f" / 共 {self.max_rounds} 轮" if self.max_rounds is not None else ""
                if not c.wait_enter("READY", f"准备第 {self.round_number+1:03d} 轮{total}：按 Enter 开始推理，q + Enter 退出。", self.emit):
                    break
                self.backend.prepare_run()  # clear ACT queues while inference is stopped
                if not c.begin_run():
                    break
                self.round_number += 1
                started = time.monotonic()
                started_at = datetime.now().astimezone().isoformat(timespec="milliseconds")
                self.emit(f"第 {self.round_number:03d} 轮开始：按 Enter 结束并复位；最多运行 {self.backend.duration_s:g} 秒。")
                reason = None
                try:
                    self.backend.run_policy()
                except KeyboardInterrupt:
                    reason = "quit"
                    raise
                except BaseException:
                    reason = "error"
                    raise
                finally:
                    duration = time.monotonic() - started
                    reason = reason or c.end_reason
                    c.set_phase("BUSY")
                    row = {"round": self.round_number, "started_at": started_at,
                           "ended_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                           "duration_s": round(duration, 3), "end_reason": reason}
                    self.round_log.add(row)
                    self.emit(f"第 {self.round_number:03d} 轮结束，推理时间 {duration:.2f} 秒，结束原因 {reason}。")
                if c.shutdown.is_set():
                    break
                self._reset()
                if self.max_rounds is not None and self.round_number >= self.max_rounds:
                    self.emit(f"已完成 {self.round_number} 轮并复位，正在退出并卸扭。")
                    break
                self._scene_wait()
        finally:
            c.quit()
            try:
                self.backend.close()
            finally:
                c.set_phase("STOPPED")
