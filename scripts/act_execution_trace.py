"""Observe the existing ACT dispatch path without predicting or replacing actions."""
from contextlib import ExitStack, contextmanager
from datetime import datetime
from fractions import Fraction
import json
import logging
from pathlib import Path
import threading
import time


@contextmanager
def replaced(obj, name, value):
    old, own = getattr(obj, name), name in vars(obj)
    setattr(obj, name, value)
    try:
        yield
    finally:
        if own:
            setattr(obj, name, old)
        else:
            delattr(obj, name)


class VideoJournal:
    """Encode cached camera frames off the control thread, including inference stalls."""
    def __init__(self, cameras, directory, origin):
        self.cameras, self.directory, self.origin = cameras, directory, origin
        self.stop = threading.Event()
        self.thread = None
        self.report = {}

    def start(self):
        self.thread = threading.Thread(target=self._run, name="trace-video", daemon=True)
        self.thread.start()

    def _run(self):
        import av
        outputs = {}
        try:
            for name in self.cameras:
                container = av.open(str(self.directory / f"{name}.mkv"), mode="w")
                outputs[name] = {"container": container, "stream": None, "first": None, "last": None,
                                 "journal": (self.directory / f"{name}-frames.jsonl").open("x")}
                self.report[name] = {"frames": 0, "max_frame_interval_s": 0, "error": None}
            while not self.stop.is_set():
                for name, camera in self.cameras.items():
                    out, status = outputs[name], self.report[name]
                    with camera.frame_lock:
                        timestamp, picture = camera.latest_timestamp, camera.latest_frame
                    if picture is None or timestamp is None or timestamp == out["last"]:
                        continue
                    if out["first"] is None:
                        out["first"] = timestamp
                        status["video_zero_t_s"] = timestamp - self.origin
                        stream = out["container"].add_stream("libx264", rate=30)
                        stream.width, stream.height = picture.shape[1], picture.shape[0]
                        stream.pix_fmt = "yuv420p"
                        stream.options = {"preset": "ultrafast", "tune": "zerolatency", "crf": "20"}
                        stream.codec_context.thread_count = 1
                        stream.time_base = stream.codec_context.time_base = Fraction(1, 1_000_000)
                        out["stream"] = stream
                    if out["last"] is not None:
                        status["max_frame_interval_s"] = max(status["max_frame_interval_s"], timestamp - out["last"])
                    frame = av.VideoFrame.from_ndarray(picture, format="rgb24")
                    frame.pts = round((timestamp - out["first"]) * 1_000_000)
                    frame.time_base = Fraction(1, 1_000_000)
                    for packet in out["stream"].encode(frame):
                        out["container"].mux(packet)
                    out["journal"].write(json.dumps({"frame": status["frames"], "capture_t_s": timestamp - self.origin,
                                                      "pts_s": frame.pts / 1_000_000}) + "\n")
                    status["frames"] += 1
                    out["last"] = timestamp
                self.stop.wait(.005)
        except Exception as exc:
            self.report["error"] = repr(exc)
            logging.exception("Trace video recording failed; see video-status.json")
        finally:
            for name, out in outputs.items():
                try:
                    if out["stream"] is not None:
                        for packet in out["stream"].encode():
                            out["container"].mux(packet)
                    out["container"].close()
                except Exception as exc:
                    self.report[name]["error"] = repr(exc)
                finally:
                    out["journal"].close()

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=10)
            if self.thread.is_alive():
                self.report["error"] = "Video writer did not finish within 10 seconds"
        (self.directory / "video-status.json").write_text(json.dumps(self.report, indent=2) + "\n")


class ExecutionTrace:
    def __init__(self, ctx, directory, video=True):
        self.ctx, self.directory, self.video_enabled = ctx, Path(directory), video
        self.policy = ctx.policy.policy
        cfg = ctx.runtime.cfg
        if cfg.policy.type != "act" or cfg.policy.temporal_ensemble_coeff is not None:
            raise ValueError("Execution tracing currently requires ACT with temporal ensembling disabled.")
        if cfg.inference.type != "sync" or cfg.interpolation_multiplier != 1:
            raise ValueError("Execution tracing requires sync inference and interpolation_multiplier=1.")
        if len(self.policy._action_queue):
            raise ValueError("Reset ACT before attaching the execution trace.")
        self.robot = ctx.hardware.robot_wrapper.inner
        self.origin = time.perf_counter()
        self.chunk_id = 0
        self.chunk = None
        self.chunk_t = None
        self.current = None
        self.index = 0
        self.camera_reads = {}
        self.stack = ExitStack()

    def stamp(self):
        return time.perf_counter() - self.origin

    @staticmethod
    def vector(tensor):
        return tensor.detach().float().cpu().reshape(-1).tolist()

    def write_row(self):
        if self.current is not None:
            self.steps.write(json.dumps(self.current) + "\n")
            if self.index % 30 == 0:
                self.steps.flush()
            self.current = None

    def __enter__(self):
        self.directory.mkdir(parents=True, exist_ok=False)
        self.steps = self.stack.enter_context((self.directory / "steps.jsonl").open("x"))
        self.chunks = self.stack.enter_context((self.directory / "chunks.jsonl").open("x"))
        self.videos = VideoJournal(self.robot.cameras, self.directory, self.origin)
        cfg = self.ctx.runtime.cfg
        metadata = {"wall_started_at": datetime.now().astimezone().isoformat(), "clock": "time.perf_counter",
                    "origin": self.origin, "action_keys": self.ctx.data.ordered_action_keys,
                    "checkpoint": str(cfg.policy.pretrained_path), "n_action_steps": cfg.policy.n_action_steps,
                    "chunk_size": cfg.policy.chunk_size, "fps": cfg.fps,
                    "temporal_ensemble_coeff": None, "gripper_id": self.robot.bus.motors["gripper"].id,
                    "timestamp_note": "Camera time is host frame availability, not hardware exposure time. Feedback is sampled before the action in the same row.",
                    "raw_goal_note": "Successful broadcast send, not register readback or an acknowledgement from each servo.",
                    "policy_postprocessor": [type(s).__name__ for s in self.ctx.policy.postprocessor.steps]}
        (self.directory / "trace.json").write_text(json.dumps(metadata, indent=2) + "\n")
        try:
            self._attach()
            if self.video_enabled:
                self.videos.start()
            return self
        except BaseException:
            self.stack.close()
            raise

    def _attach(self):
        original_predict = self.policy.predict_action_chunk
        def predict(*args, **kwargs):
            started = self.stamp()
            result = original_predict(*args, **kwargs)
            values = result.detach().float().cpu().tolist()[0]
            self.chunk_id += 1
            self.chunk, self.chunk_t = values, started
            self.chunks.write(json.dumps({"chunk_id": self.chunk_id, "observation_index": self.current["index"],
                                          "started_t_s": started, "finished_t_s": self.stamp(),
                                          "actions_normalized": values}) + "\n")
            return result
        self.stack.enter_context(replaced(self.policy, "predict_action_chunk", predict))

        original_select = self.policy.select_action
        def select(*args, **kwargs):
            before = len(self.policy._action_queue)
            result = original_select(*args, **kwargs)
            after = len(self.policy._action_queue)
            step = min(len(self.chunk), self.policy.config.n_action_steps) - after - 1
            selected = self.vector(result)
            self.current.update(chunk_id=self.chunk_id, chunk_step=step, queue_before=before, queue_after=after,
                                chunk_started_t_s=self.chunk_t, selected_normalized=selected,
                                selected_matches_chunk=(selected == self.chunk[step]))
            return result
        self.stack.enter_context(replaced(self.policy, "select_action", select))

        engine = self.ctx.policy.inference
        original_action = engine.get_action
        def get_action(*args, **kwargs):
            started = self.stamp()
            result = original_action(*args, **kwargs)
            self.current["inference_ms"] = (self.stamp() - started) * 1000
            if result is not None:
                self.current["after_policy_postprocess"] = self.vector(result)
            return result
        self.stack.enter_context(replaced(engine, "get_action", get_action))

        for name, camera in self.robot.cameras.items():
            original_read = camera.read_latest
            def read_latest(*args, _camera=camera, _original=original_read, _name=name, **kwargs):
                frame = _original(*args, **kwargs)
                with _camera.frame_lock:
                    capture = _camera.latest_timestamp if _camera.latest_frame is frame else None
                self.camera_reads[_name] = {"capture_t_s": capture - self.origin if capture is not None else None,
                                            "read_t_s": self.stamp()}
                return frame
            self.stack.enter_context(replaced(camera, "read_latest", read_latest))

        original_observe = self.robot.get_observation
        def observe(*args, **kwargs):
            self.write_row()
            started = self.stamp()
            result = original_observe(*args, **kwargs)
            self.current = {"index": self.index, "observe_started_t_s": started, "observe_finished_t_s": self.stamp(),
                            "feedback": {k: float(v) for k, v in result.items() if k.endswith(".pos")},
                            "cameras": dict(self.camera_reads), "sent": False}
            self.index += 1
            return result
        self.stack.enter_context(replaced(self.robot, "get_observation", observe))

        original_send = self.robot.send_action
        def send(action, *args, **kwargs):
            self.current["after_robot_processors"] = {k: float(v) for k, v in action.items()}
            self.current["send_started_t_s"] = self.stamp()
            result = original_send(action, *args, **kwargs)
            self.current["send_finished_t_s"] = self.stamp()
            self.current["sent"] = True
            self.current["robot_returned_target"] = {k: float(v) for k, v in result.items()}
            return result
        self.stack.enter_context(replaced(self.robot, "send_action", send))

        bus = self.robot.bus
        original_write = bus._sync_write
        def sync_write(addr, length, ids_values, *args, **kwargs):
            if addr == 42 and length == 2:
                self.current["raw_goal_packet"] = dict(ids_values)
                decoded = bus._decode_sign("Goal_Position", dict(ids_values))
                normalized = bus._normalize(decoded)
                self.current["sent_gripper_target"] = normalized[bus.motors["gripper"].id]
                self.current["packet_started_t_s"] = self.stamp()
            result = original_write(addr, length, ids_values, *args, **kwargs)
            if addr == 42 and length == 2:
                self.current["packet_finished_t_s"] = self.stamp()
                self.current["packet_comm_result"] = result
            return result
        self.stack.enter_context(replaced(bus, "_sync_write", sync_write))

    def __exit__(self, exc_type, exc, tb):
        try:
            if exc is not None and self.current is not None:
                self.current["error"] = repr(exc)
            self.write_row()
        finally:
            self.stack.close()
            self.videos.close()
