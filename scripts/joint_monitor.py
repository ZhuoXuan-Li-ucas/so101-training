#!/usr/bin/env python3
"""Local SO-101 joint diagnostics; opens in read-only mode, without cameras."""

import argparse
import json
import math
import queue
import signal
import threading
import time
import tkinter as tk
from collections import deque
from pathlib import Path
from tkinter import ttk

from diagnose_teleop import ARMS, ROOT, disable_all, ensure_port_free, feedback


JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
NAMES = ('底座旋转', '大臂升降', '肘部弯曲', '腕部俯仰', '腕部旋转', '夹爪')
COLORS = {'leader': '#b87800', 'sent': '#1674d1', 'goal': '#a049b3', 'follower': '#168352'}


def targets(leader, present, bounds, cap):
    """Bound the requested target to calibrated range and measured-position error."""
    if not math.isfinite(cap) or not 0.5 <= cap <= 10:
        raise ValueError('目标差限幅必须在 0.5 到 10 之间')
    result = {}
    for name in JOINTS:
        requested, actual = leader[name], present[name]
        if not math.isfinite(requested) or not math.isfinite(actual):
            raise ValueError(f'{name}: 非有限位置')
        low, high = bounds[name]
        # Do not command motion when the measured position is significantly
        # outside calibration. Small encoder differences at the stops are common.
        if actual < low - 2 or actual > high + 2:
            raise ValueError(f'{name}: 从臂位置超出校准范围')
        step_low, step_high = max(low, actual - cap), min(high, actual + cap)
        if step_low > step_high:
            raise ValueError(f'{name}: 无法在目标差限幅内返回校准范围')
        result[name] = min(step_high, max(step_low, requested))
    return result


def open_buses():
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    buses = {}
    # Check all ports before opening either, to avoid sharing with teleoperation.
    for serial, _ in ARMS.values():
        ensure_port_free(f'/dev/serial/by-id/usb-1a86_USB_Single_Serial_{serial}-if00')
    try:
        for role, (serial, relative) in ARMS.items():
            cal = json.loads((ROOT / 'data/calibration' / relative).read_text())
            bus = FeetechMotorsBus(
                f'/dev/serial/by-id/usb-1a86_USB_Single_Serial_{serial}-if00',
                {n: Motor(v['id'], 'sts3215', MotorNormMode.RANGE_0_100 if n == 'gripper' else MotorNormMode.DEGREES)
                 for n, v in cal.items()},
                {n: MotorCalibration(**v) for n, v in cal.items()},
            )
            buses[role] = bus
            bus.connect(handshake=False)
            bus.set_baudrate(1_000_000)
        return buses
    except Exception:
        for bus in buses.values():
            if bus.is_connected:
                bus.disconnect(disable_torque=False)
        raise


def read_bounds(bus):
    result = {}
    for name, motor in bus.motors.items():
        cal = bus.calibration[name]
        low = bus._normalize({motor.id: cal.range_min})[motor.id]
        high = bus._normalize({motor.id: cal.range_max})[motor.id]
        result[name] = (min(low, high), max(low, high))
    return result


def faults_in(health):
    return [f'{role}/{name}: status={v["Status"]}, packet={v["packet_error"]}'
            for role, values in health.items() for name, v in values.items()
            if v['Status'] or v['packet_error']]


class Monitor:
    def __init__(self, duration=None):
        self.commands = queue.Queue()
        self.quit = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.history = deque(maxlen=600)
        self.message = '正在连接，只读取状态…'
        self.duration = duration
        self.failed = False
        self.log_path = ROOT / 'logs' / f'joint-monitor-{time.strftime("%Y%m%d-%H%M%S")}.jsonl'
        self.thread = threading.Thread(target=self.run, name='motor-diagnostics', daemon=False)

    def run(self):
        buses = {}
        owns_torque = False
        running = False
        start = time.monotonic()
        cap = 2.0
        last_tick = None
        latest_health = {}
        next_health = 0.0
        last_flush = 0.0
        previous_sent = None
        try:
            buses = open_buses()
            bounds = read_bounds(buses['follower'])
            calibration_ok = all(bus.is_calibrated for bus in buses.values())
            self.message = '只读监视：主臂目标 / 从臂实测；目标回读此时是历史值'
            with self.log_path.open('x') as log:
                def record(value):
                    log.write(json.dumps(value, ensure_ascii=False) + '\n')

                record({'event': 'start', 'calibration_matches': calibration_ok, 'bounds': bounds})
                while not self.quit.is_set():
                    tick = time.monotonic()
                    if running and last_tick and tick - last_tick > 0.25:
                        raise RuntimeError('控制周期超过 250 ms，停止跟随')
                    if self.duration is not None and tick - start >= self.duration:
                        break
                    timings = {}
                    def measured(key, call):
                        before = time.perf_counter()
                        value = call()
                        timings[key] = (time.perf_counter() - before) * 1000
                        return value

                    leader = measured('leader_read', lambda: buses['leader'].sync_read('Present_Position', num_retry=2))
                    follower = measured('follower_read', lambda: buses['follower'].sync_read('Present_Position', num_retry=2))
                    if tick >= next_health:
                        latest_health = measured('health_read', lambda: {role: feedback(bus) for role, bus in buses.items()})
                        next_health = tick + 0.2
                        record({'event': 'health', 't': tick - start, 'arms': latest_health})
                    faults = faults_in(latest_health)
                    if faults and running:
                        raise RuntimeError('舵机故障，停止跟随：' + '; '.join(faults))

                    while not self.commands.empty():
                        command, value = self.commands.get_nowait()
                        if command == 'stop':
                            running = False
                            if owns_torque:
                                disable_all(buses['follower'])
                                owns_torque = False
                            previous_sent = None
                            next_health = 0
                            self.message = '已停止跟随、关闭从臂扭矩；继续只读监视'
                            record({'event': 'stop_follow', 't': tick - start})
                        elif command == 'start' and not running:
                            cap = float(value)
                            targets(leader, follower, bounds, cap)
                            current_health = {role: feedback(bus) for role, bus in buses.items()}
                            if faults_in(current_health) or not calibration_ok:
                                self.message = '未启动：请先处理舵机故障或校准不一致'
                                continue
                            for bus in buses.values():
                                disable_all(bus)
                            present = buses['follower'].sync_read('Present_Position', normalize=False)
                            buses['follower'].sync_write('Goal_Position', present, normalize=False)
                            owns_torque = True  # Cleanup even if enabling only partially succeeds.
                            buses['follower'].enable_torque()
                            running = True
                            next_health = 0
                            self.message = f'诊断跟随中：目标差限幅 {cap:g}° / 夹爪 {cap:g}%；停止按钮或 Esc 退出跟随'
                            record({'event': 'start_follow', 't': tick - start, 'cap': cap})

                    sent = None
                    if running:
                        sent = targets(leader, follower, bounds, cap)
                        measured('write', lambda: buses['follower'].sync_write('Goal_Position', sent))
                    goal = measured('goal_read', lambda: buses['follower'].sync_read('Goal_Position', num_retry=2))
                    # A successful broadcast write alone is not an acknowledgement.
                    # Compare the actual motor goal registers with the sent values.
                    ack_error = max(abs(goal[n] - sent[n]) for n in JOINTS) if sent else None
                    if sent and ack_error > 0.2:
                        raise RuntimeError(f'下发目标与舵机目标回读不一致：最大差 {ack_error:.3f}；已停止')
                    sample = {
                        'event': 'sample', 't': tick - start, 'running': running, 'cap': cap,
                        'leader': leader, 'follower': follower, 'goal': goal, 'sent': sent,
                        'previous_sent': previous_sent, 'timings_ms': timings,
                        'cycle_ms': (tick - last_tick) * 1000 if last_tick else None,
                        'ack_error': ack_error, 'faults': faults, 'health': latest_health,
                    }
                    previous_sent = sent
                    with self.lock:
                        self.latest = sample
                        self.history.append(sample)
                    record({k: v for k, v in sample.items() if k != 'health'})
                    if tick - last_flush > 1:
                        log.flush()
                        last_flush = tick
                    last_tick = tick
                    self.quit.wait(max(0, 1 / 30 - (time.monotonic() - tick)))
        except Exception as exc:
            self.failed = True
            self.message = f'错误：{exc}'
            print(self.message, flush=True)
            with self.log_path.open('a') as log:
                log.write(json.dumps({'event': 'error', 'message': str(exc)}, ensure_ascii=False) + '\n')
        finally:
            if owns_torque:
                try:
                    disable_all(buses['follower'])
                    self.message += '；从臂扭矩已关闭'
                except Exception as exc:
                    self.message += f'；无法确认扭矩关闭，请断开从臂电源：{exc}'
            for bus in buses.values():
                if bus.is_connected:
                    bus.disconnect(disable_torque=False)
            print(self.message, flush=True)


class Window:
    def __init__(self, monitor):
        self.monitor = monitor
        self.root = tk.Tk()
        self.root.title('SO101 Joint Monitor | 主从臂六关节诊断')
        self.root.geometry('1420x930+100+40')
        self.closing = False
        style = ttk.Style()
        style.configure('Treeview', rowheight=30, font=('sans', 11))
        style.configure('Treeview.Heading', font=('sans', 10, 'bold'))
        toolbar = ttk.Frame(self.root, padding=10)
        toolbar.pack(fill='x')
        self.cap = tk.StringVar(value='2')
        ttk.Label(toolbar, text='目标差限幅（度 / 夹爪 %）').pack(side='left')
        self.cap_input = ttk.Combobox(toolbar, textvariable=self.cap, values=('2', '5', '10'), width=5, state='readonly')
        self.cap_input.pack(side='left', padx=8)
        self.start_button = ttk.Button(toolbar, text='启动诊断跟随（从臂会运动）', command=self.start)
        self.start_button.pack(side='left', padx=8)
        tk.Button(toolbar, text='停止跟随 / Esc', bg='#bb2639', fg='white', command=self.stop, padx=20).pack(side='left', padx=8)
        ttk.Label(toolbar, text='初始为只读；关闭窗口也会停止本工具启动的跟随').pack(side='left', padx=10)
        self.status = tk.StringVar(value=monitor.message)
        ttk.Label(self.root, textvariable=self.status, font=('sans', 12, 'bold'), padding=8).pack(fill='x')
        self.timing = tk.StringVar(value='等待读数…')
        ttk.Label(self.root, textvariable=self.timing, padding=8).pack(fill='x')
        columns = ('joint', 'leader', 'sent', 'goal', 'follower', 'error', 'torque', 'load', 'voltage', 'temp', 'fault')
        self.table = ttk.Treeview(self.root, columns=columns, show='headings', height=6)
        labels = ('关节（臂：度，夹爪：%）', '主臂目标', '实际下发', '目标回读', '从臂实测', '主从差', '从臂扭矩', '负载原值', '电压 V', '温度 ℃', '主/从状态')
        for key, label in zip(columns, labels):
            self.table.heading(key, text=label)
            self.table.column(key, width=225 if key == 'joint' else 101, anchor='w' if key == 'joint' else 'center')
        for name in JOINTS:
            self.table.insert('', 'end', iid=name, values=(name,))
        self.table.tag_configure('fault', background='#ffd0d0')
        self.table.tag_configure('lag', background='#fff0cf')
        self.table.pack(fill='x', padx=10)
        legend = ttk.Frame(self.root, padding=8)
        legend.pack(fill='x')
        for key, label in [('leader', '主臂目标'), ('sent', '实际下发'), ('goal', '目标回读'), ('follower', '从臂实测')]:
            tk.Label(legend, text='━━ ' + label, fg=COLORS[key], font=('sans', 11, 'bold')).pack(side='left', padx=12)
        ttk.Label(legend, text='只读时没有下发指令；目标回读保留上次目标。图显示最近 12 秒。').pack(side='left', padx=12)
        charts = ttk.Frame(self.root)
        charts.pack(fill='both', expand=True, padx=10)
        self.canvases = {}
        for i, (name, label) in enumerate(zip(JOINTS, NAMES)):
            frame = ttk.LabelFrame(charts, text=f'{label} · {name}', padding=3)
            frame.grid(row=i // 3, column=i % 3, sticky='nsew', padx=4, pady=4)
            canvas = tk.Canvas(frame, bg='white', highlightthickness=0)
            canvas.pack(fill='both', expand=True)
            self.canvases[name] = canvas
        for i in range(3):
            charts.columnconfigure(i, weight=1)
        for i in range(2):
            charts.rowconfigure(i, weight=1)
        ttk.Label(self.root, text=f'日志：{monitor.log_path}', padding=8).pack(fill='x')
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.bind('<Escape>', lambda event: self.stop())
        self.root.after(100, self.refresh)

    def start(self):
        self.monitor.commands.put(('start', float(self.cap.get())))
        self.start_button.configure(state='disabled')

    def stop(self):
        self.monitor.commands.put(('stop', None))

    def close(self):
        self.closing = True
        self.monitor.quit.set()
        self.status.set('正在释放串口并关闭从臂扭矩…')

    def refresh(self):
        if self.closing and not self.monitor.thread.is_alive():
            self.root.destroy()
            return
        with self.monitor.lock:
            sample = self.monitor.latest
            history = list(self.monitor.history)
        self.status.set(self.monitor.message)
        alive = self.monitor.thread.is_alive()
        running = bool(sample and sample['running'] and alive)
        self.start_button.configure(state='normal' if alive and sample and not running else 'disabled')
        self.cap_input.configure(state='disabled' if running else 'readonly')
        if sample:
            ms = sample['timings_ms']
            cycle = sample['cycle_ms'] or 0
            write = f'{ms["write"]:.2f}' if 'write' in ms else '—'
            self.timing.set(f'主臂位置读取 {ms["leader_read"]:.2f} ms    从臂位置读取 {ms["follower_read"]:.2f} ms    '
                            f'目标下发 {write} ms    目标回读 {ms["goal_read"]:.2f} ms    '
                            f'采样 {1000/cycle if cycle else 0:.1f} Hz    界面独立刷新')
            for name, label in zip(JOINTS, NAMES):
                health = sample['health']['follower'][name]
                lh = sample['health']['leader'][name]
                diff = sample['leader'][name] - sample['follower'][name]
                sent = sample['sent']
                values = (f'{label} / {name}', f'{sample["leader"][name]:.2f}', f'{sent[name]:.2f}' if sent else '—',
                          f'{sample["goal"][name]:.2f}', f'{sample["follower"][name]:.2f}', f'{diff:+.2f}',
                          '开' if health['Torque_Enable'] else '关', health['Present_Load'],
                          f'{health["Present_Voltage"]/10:.1f}', health['Present_Temperature'],
                          f'{lh["Status"]}/{health["Status"]}')
                tag = 'fault' if lh['Status'] or health['Status'] else 'lag' if running and abs(diff) > 5 else ''
                self.table.item(name, values=values, tags=(tag,))
            recent = [s for s in history if s['t'] > sample['t'] - 12]
            for name, canvas in self.canvases.items():
                self.draw(canvas, name, recent, sample['t'])
        self.root.after(100, self.refresh)

    @staticmethod
    def draw(canvas, name, samples, now):
        canvas.delete('all')
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if width < 100 or height < 50 or not samples:
            return
        values = [s[key][name] for s in samples for key in COLORS if s[key] is not None]
        low, high = min(values), max(values)
        margin = max(1, (high - low) * 0.1)
        low, high = low - margin, high + margin
        canvas.create_text(4, 10, text=f'{high:.1f}', anchor='w', fill='#666')
        canvas.create_text(4, height - 22, text=f'{low:.1f}', anchor='w', fill='#666')
        canvas.create_line(48, 12, 48, height - 25, fill='#ddd')
        for key, color in COLORS.items():
            points = []
            for sample in samples:
                if sample[key] is None:
                    if len(points) >= 4:
                        canvas.create_line(*points, fill=color, width=2)
                    points = []
                    continue
                x = 48 + (sample['t'] - now + 12) / 12 * (width - 55)
                y = 12 + (high - sample[key][name]) / (high - low) * (height - 37)
                points.extend((x, y))
            if len(points) >= 4:
                canvas.create_line(*points, fill=color, width=2, dash=(4, 3) if key == 'goal' else ())
        canvas.create_text(width - 6, height - 10, text='现在', anchor='e', fill='#666')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, help='Optional maximum monitoring duration in seconds')
    parser.add_argument('--headless', action='store_true', help='Read-only bounded capture without GUI')
    args = parser.parse_args()
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error('--duration must be positive and finite')
    if args.headless and not args.duration:
        parser.error('--headless requires --duration')
    monitor = Monitor(args.duration)
    signal.signal(signal.SIGTERM, lambda *_: monitor.quit.set())
    signal.signal(signal.SIGINT, lambda *_: monitor.quit.set())
    if args.headless:
        monitor.thread.start()
        monitor.thread.join()
    else:
        window = Window(monitor)
        monitor.thread.start()
        window.root.mainloop()
        monitor.quit.set()
        monitor.thread.join()
    print('Log:', monitor.log_path, flush=True)
    return int(monitor.failed)


if __name__ == '__main__':
    raise SystemExit(main())
