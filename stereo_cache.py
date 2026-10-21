"""Bounded exact-stamp stereo pairing with one asynchronous encoding worker."""
import copy
import queue
import threading
import time


class StereoCache:
    def __init__(self, encode, sim_clock, wall_clock=time.monotonic):
        self.encode, self.sim_clock, self.wall_clock = encode, sim_clock, wall_clock
        self.lock = threading.Lock()
        self.pending = {'left': {}, 'right': {}}
        self.jobs = queue.Queue(maxsize=1)
        self.closed = threading.Event()
        self.frame = None
        self.last_attempt = -1e9
        self.error = None
        threading.Thread(target=self.run, name='stereo_encoder', daemon=True).start()

    def push(self, side, message):
        stamp = message.header.stamp.to_nsec()
        with self.lock:
            own = self.pending[side]
            own[stamp] = message
            while len(own) > 8:
                del own[min(own)]
            other = self.pending['right' if side == 'left' else 'left']
            if stamp not in other:
                return
            pair = {side: own.pop(stamp), 'right' if side == 'left' else 'left': other.pop(stamp)}
            now = self.wall_clock()
            if now-self.last_attempt < .25:
                return
            self.last_attempt = now
        try:
            self.jobs.put_nowait((pair, now))
        except queue.Full:
            pass

    def run(self):
        while not self.closed.is_set():
            try:
                pair, received = self.jobs.get(timeout=.1)
            except queue.Empty:
                continue
            try:
                stamp = pair['left'].header.stamp.to_sec()
                if not -.05 <= self.sim_clock()-stamp <= .5:
                    raise ValueError('source stereo image is stale')
                frame = {side: self.encode(message) for side, message in pair.items()}
                with self.lock:
                    self.frame = {'stamp_sim_s': stamp, 'received': received, **frame}
                    self.error = None
            except Exception as exc:
                with self.lock:
                    self.error = str(exc)

    def snapshot(self):
        with self.lock:
            if not self.frame:
                raise ValueError(self.error or 'no exact-stamp stereo pair is available')
            result = copy.copy(self.frame)
        if not 0 <= self.wall_clock()-result['received'] <= 2 or not -.05 <= self.sim_clock()-result['stamp_sim_s'] <= .5:
            raise ValueError('synchronized stereo pair is stale')
        return result

    def close(self):
        self.closed.set()
