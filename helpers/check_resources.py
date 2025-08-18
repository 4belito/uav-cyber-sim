import logging
import os
import threading
import time
from contextlib import contextmanager
from typing import Iterable, Optional

import psutil


class PsutilResourceLogger:
    """
    Logs CPU% and memory (RSS/USS) for the current process, optionally
    including all child processes. Uses Python logging.

    Parameters
    ----------
    logfile : str
        Path to log file.
    interval : float
        Seconds between samples.
    include_children : bool
        If True, aggregate CPU% and memory across all child processes.
    logger_name : Optional[str]
        Name for the logger (so you can attach your own handlers).

    """

    def __init__(
        self,
        logfile: str = "usage.log",
        interval: float = 1.0,
        include_children: bool = True,
        logger_name: Optional[str] = None,
    ):
        self.interval = interval
        self.include_children = include_children
        self.proc = psutil.Process(os.getpid())
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Logger
        self.log = logging.getLogger(logger_name or __name__ + ".resource")
        self.log.setLevel(logging.INFO)
        if not self.log.handlers:  # avoid duplicate handlers if reused
            fh = logging.FileHandler(logfile)
            fmt = logging.Formatter(
                "%(asctime)s - %(levelname)s - %(message)s", datefmt="%H:%M:%S"
            )
            fh.setFormatter(fmt)
            self.log.addHandler(fh)

        # Prime cpu_percent for accurate deltas
        self.proc.cpu_percent(None)
        for c in self._children():
            try:
                c.cpu_percent(None)
            except Exception:
                pass

    def _children(self) -> Iterable[psutil.Process]:
        if not self.include_children:
            return []
        try:
            return self.proc.children(recursive=True)
        except Exception:
            return []

    @staticmethod
    def _safe_mem_info(p: psutil.Process):
        try:
            mem = p.memory_full_info()  # has USS on most platforms
        except Exception:
            mem = p.memory_info()  # fallback (RSS only)
        return mem

    def _sample(self):
        # Per-process
        cpu = self.proc.cpu_percent(None)  # since last call
        mem = self._safe_mem_info(self.proc)
        rss = mem.rss / (1024**2)  # MB
        uss = getattr(mem, "uss", None)
        uss_mb = (uss / (1024**2)) if uss is not None else None

        total_cpu = cpu
        total_rss = rss
        total_uss = uss_mb if uss_mb is not None else 0.0

        # Children aggregate
        if self.include_children:
            for ch in self._children():
                try:
                    total_cpu += ch.cpu_percent(None)
                    m = self._safe_mem_info(ch)
                    total_rss += m.rss / (1024**2)
                    if hasattr(m, "uss"):
                        total_uss += m.uss / (1024**2)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue

        # Compose message
        if uss_mb is not None:
            msg = (
                f"PID {self.proc.pid} | CPU {total_cpu:5.1f}% | "
                f"RSS {total_rss:8.2f} MB | USS {total_uss:8.2f} MB"
            )
        else:
            msg = (
                f"PID {self.proc.pid} | CPU {total_cpu:5.1f}% | RSS {total_rss:8.2f} MB"
            )
        return msg

    def _run(self):
        while self._running:
            try:
                self.log.info(self._sample())
            except Exception as e:
                self.log.warning(f"resource-log error: {e}")
            time.sleep(self.interval)

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()

    @contextmanager
    def running(self):
        """Context manager: start logging on enter, stop on exit."""
        self.start()
        try:
            yield self
        finally:
            self.stop()
