"""Small HTTP client; does not need ROS or third-party HTTP packages."""
import json
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen


class BridgeClient:
    supports_stereo_localization = True
    def __init__(self, url="http://127.0.0.1:8765"):
        self.url = url.rstrip("/")

    def request(self, path, payload=None, timeout=5):
        request = Request(self.url + path,
                          data=None if payload is None else json.dumps(payload).encode(),
                          headers={"Content-Type": "application/json"},
                          method="GET" if payload is None else "POST")
        try:
            with urlopen(request, timeout=timeout) as response:
                return json.load(response)
        except HTTPError as exc:
            try:
                detail = json.load(exc)
            except (ValueError, OSError):
                detail = {"error": "HTTP {}".format(exc.code)}
            raise RuntimeError(detail.get("error", str(detail))) from None

    def state(self):
        return self.request("/state")

    def capture_camera(self, side="left"):
        if side not in ("left", "right"):
            raise ValueError("camera side must be left or right")
        return self.request("/camera/snapshot?side=" + side)

    def capture_stereo(self):
        return self.request("/camera/stereo")

    def localize_stereo(self, pair_id, bboxes):
        return self.request("/camera/stereo/localize", {"pair_id": pair_id, "bboxes": bboxes})

    def execute(self, steps, description="", on_progress=None):
        accepted = self.request("/mission", {"steps": steps, "description": description})
        return self.wait_for(accepted, on_progress)

    def wait_for(self, accepted, on_progress=None):
        mission_id = accepted["mission"]["id"]
        last_step = None
        deadline = time.monotonic() + 1000
        while time.monotonic() < deadline:
            state = self.state()
            mission = state.get("mission", {})
            if mission.get("id") != mission_id:
                raise RuntimeError("mission was replaced before its result could be read")
            if mission.get("step") != last_step:
                last_step = mission.get("step")
                if on_progress:
                    on_progress(mission)
            if mission["status"] != "running":
                return state
            time.sleep(0.5)
        self.request("/stop", {})
        raise RuntimeError("mission polling timed out; requested hold")
