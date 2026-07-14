"""Tests for pyhulax.video.AsyncDetector (skipped without the video deps)."""

import time

import pytest

np = pytest.importorskip("numpy")

from pyhulax.video import AsyncDetector  # noqa: E402  (needs numpy)


class _FakeDetector:
    """Slow, duck-typed detector standing in for a real YOLO model."""

    def __init__(self, delay=0.05):
        self.calls = 0
        self._delay = delay

    def detect(self, image):
        time.sleep(self._delay)  # simulate slow inference
        self.calls += 1
        return [("obj", self.calls)]

    @property
    def avg_inference_time(self):
        return 12.3


class _FakeFrame:
    def __init__(self, number, image):
        self.frame_number = number
        self.image = image
        self.detections = []


def _img():
    return np.zeros((4, 4, 3), dtype=np.uint8)


def test_call_is_non_blocking_and_detection_runs_off_thread():
    det = _FakeDetector(delay=0.05)
    adet = AsyncDetector(det)
    try:
        # First frame: the callback must return immediately (not wait ~0.05s
        # for inference), and no detections are available yet.
        t0 = time.perf_counter()
        f0 = adet(_FakeFrame(0, _img()))
        assert (time.perf_counter() - t0) < 0.03
        assert f0.detections == []

        # The worker processes the latest frame off-thread.
        deadline = time.time() + 2.0
        while det.calls == 0 and time.time() < deadline:
            time.sleep(0.01)
        assert det.calls >= 1

        # A later frame carries the worker's most recent detections.
        f1 = adet(_FakeFrame(1, _img()))
        assert f1.detections
        assert adet.latest_detections
        assert adet.avg_inference_time == 12.3
    finally:
        adet.stop()


def test_stop_halts_the_worker():
    det = _FakeDetector(delay=0.01)
    adet = AsyncDetector(det)
    adet(_FakeFrame(0, _img()))
    time.sleep(0.1)
    adet.stop()
    calls_after_stop = det.calls
    # Feed more frames after stop; without a running worker they aren't processed.
    for n in range(1, 4):
        adet(_FakeFrame(n, _img()))
    time.sleep(0.1)
    assert det.calls == calls_after_stop
