"""Machine-wide limit on concurrent LEGO bench simulator runs.

Every simulator launch (``scripts/run_lego_bench.py``) takes one of ``LEGO_BENCH_SLOTS`` slots (default 6): an
exclusive ``flock`` on ``results/lego_bench/slots/slot<i>.lock``, held until the process exits. It also waits while
the machine is busy: less than ``LEGO_BENCH_MIN_FREE_GB`` (default 32) GB of available memory, or a 1-minute load
average above ``LEGO_BENCH_MAX_LOAD`` (default 22, about 70 % of 32 cores). A caller that already holds a slot for
the child passes ``LEGO_BENCH_SLOT_HELD=<i>`` in the child's environment (``scripts/lego_bench_iter.py`` does).
Slot i renders on GPU i % 2.
"""

from __future__ import annotations

import contextlib
import fcntl
import os
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
SLOTS_DIR = os.path.join(ROOT, "results", "lego_bench", "slots")
POLL_S = 15.0


def max_slots() -> int:
    return int(os.environ.get("LEGO_BENCH_SLOTS", "6"))


def mem_available_gb() -> float:
    with open("/proc/meminfo") as f:
        for line in f:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) / 1024**2
    return 1e9


def machine_busy() -> str | None:
    """Why a new simulator run should wait (None: go)."""
    free = mem_available_gb()
    if free < float(os.environ.get("LEGO_BENCH_MIN_FREE_GB", "32")):
        return f"only {free:.0f} GB memory available"
    load = os.getloadavg()[0]
    if load > float(os.environ.get("LEGO_BENCH_MAX_LOAD", "22")):
        return f"load average {load:.1f}"
    return None


def _try_lock(i: int):
    f = open(os.path.join(SLOTS_DIR, f"slot{i}.lock"), "w")
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        f.close()
        return None
    f.write(f"{os.getpid()}\n")
    f.flush()
    return f


def acquire(log=print):
    """Block until a slot is free and the machine is not busy; returns (slot index, open lock file). The lock is held
    while the file stays open (until the process exits)."""
    os.makedirs(SLOTS_DIR, exist_ok=True)
    waited, said = 0.0, ""
    while True:
        why = machine_busy()
        if why is None:
            for i in range(max_slots()):
                f = _try_lock(i)
                if f is not None:
                    if waited:
                        log(f"[SLOTS] got simulator slot {i} after waiting {waited:.0f} s")
                    return i, f
            why = f"all {max_slots()} simulator slots busy"
        if why != said:
            log(f"[SLOTS] waiting: {why}")
            said = why
        time.sleep(POLL_S)
        waited += POLL_S


@contextlib.contextmanager
def sim_slot(log=print):
    """Hold a slot for the duration of the block (for a caller that launches the simulator as a child process)."""
    i, f = acquire(log)
    try:
        yield i
    finally:
        fcntl.flock(f, fcntl.LOCK_UN)
        f.close()


_HELD = None


def hold_for_this_process(log=print) -> int:
    """Take a slot for the rest of this process (the simulator launcher), unless the parent holds one for us. Also
    pins the render GPU (slot % 2) when the caller did not choose one, and lowers this process's priority."""
    global _HELD
    held = os.environ.get("LEGO_BENCH_SLOT_HELD")
    if held is not None:
        return int(held)
    i, _HELD = acquire(log)
    os.environ.setdefault("CUDA_VISIBLE_DEVICES", str(i % 2))
    with contextlib.suppress(OSError):
        os.nice(5)
    return i
