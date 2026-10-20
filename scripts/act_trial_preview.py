"""Live display of existing RGB camera caches; no device opens or motion calls."""

import logging
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time


class CameraPreview:
    """Each player has its own writer; slow/closed windows never block control."""

    def __init__(self, cameras, directory, fps=15):
        self.cameras = cameras
        self.directory = Path(directory)
        self.fps = fps
        self.stop = threading.Event()
        self.players = []

    def start(self):
        player = Path(sys.executable).resolve().parent / "ffplay"
        player = str(player) if player.is_file() else shutil.which("ffplay")
        if not player or not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            logging.warning("Camera preview unavailable (ffplay/display missing); terminal control remains active.")
            return
        for column, (name, camera) in enumerate(self.cameras.items()):
            log = None
            process = None
            try:
                with camera.frame_lock:
                    frame = camera.latest_frame
                if frame is None:
                    logging.warning("No cached frame for %s preview; skipping window.", name)
                    continue
                height, width = frame.shape[:2]
                log = (self.directory / f"preview-{name}.log").open("wb")
                process = subprocess.Popen([
                    player, "-hide_banner", "-loglevel", "warning", "-f", "rawvideo",
                    "-pixel_format", "rgb24", "-video_size", f"{width}x{height}",
                    "-framerate", str(self.fps), "-probesize", "32", "-analyzeduration", "0",
                    "-fflags", "nobuffer", "-flags", "low_delay", "-framedrop", "-sync", "ext",
                    "-i", "pipe:0", "-an", "-autoexit",
                    "-window_title", f"SO101 {name} LIVE | controls in terminal",
                    "-x", str(width), "-y", str(height),
                    "-left", str(100 + column * (width + 20)), "-top", "50",
                ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=log, bufsize=0)
                thread = threading.Thread(target=self._feed, args=(name, camera, process),
                                          name=f"preview-{name}", daemon=True)
                self.players.append((process, thread, log))
                thread.start()
                logging.info("Live %s preview started (%s fps); terminal controls remain active.", name, self.fps)
            except Exception:
                logging.exception("Could not start %s preview; policy/control unchanged.", name)
                if process is not None and process.poll() is None:
                    process.terminate()
                if log is not None:
                    log.close()

    def _feed(self, name, camera, process):
        last = None
        try:
            while not self.stop.is_set() and process.poll() is None:
                started = time.monotonic()
                with camera.frame_lock:
                    timestamp, frame = camera.latest_timestamp, camera.latest_frame
                if frame is not None and timestamp != last:
                    # Never mutate model input or hold the camera lock while writing.
                    data = memoryview(frame.tobytes())
                    while data and not self.stop.is_set():
                        count = process.stdin.write(data)
                        if not count:
                            return
                        data = data[count:]
                    last = timestamp
                self.stop.wait(max(.001, 1 / self.fps - (time.monotonic() - started)))
        except (BrokenPipeError, OSError, ValueError):
            if not self.stop.is_set():
                logging.info("%s preview closed; terminal trial continues.", name)
        except Exception:
            logging.exception("%s preview failed; terminal trial continues.", name)

    def close(self):
        self.stop.set()
        for process, thread, log in self.players:
            try:
                # Stop readers first to unblock any pipe write in the display thread.
                if process.poll() is None:
                    process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
                if thread.ident is not None:
                    thread.join(timeout=1)
                if process.stdin is not None:
                    process.stdin.close()
            except Exception:
                logging.exception("Preview cleanup failed; motor cleanup is independent.")
            finally:
                log.close()
        self.players.clear()
