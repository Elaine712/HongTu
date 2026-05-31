#!/usr/bin/env python3
"""Small HTTP client used by the PC GUI to control the G1 body service."""

import json
import urllib.error
import urllib.request


class G1RemoteError(RuntimeError):
    pass


class G1RemoteClient:
    def __init__(self, base_url, timeout=2.0):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def _request(self, method, path, payload=None, timeout=None):
        data = None
        headers = {"Accept": "application/json"}
        if payload is not None:
            data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            headers["Content-Type"] = "application/json; charset=utf-8"
        req = urllib.request.Request(
            self.base_url + path,
            data=data,
            headers=headers,
            method=method,
        )
        try:
            with self.opener.open(req, timeout=timeout or self.timeout) as resp:
                raw = resp.read().decode("utf-8")
        except urllib.error.HTTPError as e:
            try:
                body = e.read().decode("utf-8")
                detail = json.loads(body).get("error", body)
            except Exception:
                detail = str(e)
            raise G1RemoteError(detail)
        except Exception as e:
            raise G1RemoteError(str(e))
        try:
            result = json.loads(raw) if raw else {}
        except json.JSONDecodeError as e:
            raise G1RemoteError(f"bad JSON response: {e}")
        if not result.get("ok", False):
            raise G1RemoteError(result.get("error", "remote request failed"))
        return result

    def status(self):
        return self._request("GET", "/api/status")

    def connect(self):
        return self._request("POST", "/api/connect", {}, timeout=8.0)

    def move(self, vx, vy, wz, continuous=False):
        return self._request("POST", "/api/move", {
            "vx": float(vx),
            "vy": float(vy),
            "wz": float(wz),
            "continuous": bool(continuous),
        })

    def stop(self):
        return self._request("POST", "/api/stop", {})

    def stand(self):
        return self._request("POST", "/api/stand", {}, timeout=8.0)

    def fsm(self, fsm_id):
        return self._request("POST", "/api/fsm", {"id": int(fsm_id)}, timeout=8.0)

    def action(self, name=None, action_id=None):
        payload = {}
        if name is not None:
            payload["name"] = name
        if action_id is not None:
            payload["id"] = int(action_id)
        return self._request("POST", "/api/action", payload, timeout=12.0)

    def coordinated_action(self, name):
        return self._request("POST", "/api/coordinated", {"name": name}, timeout=12.0)

    def speak(self, text, speaker_id=0):
        return self._request("POST", "/api/speak", {
            "text": text,
            "speaker_id": int(speaker_id),
        }, timeout=12.0)

    def volume(self, value):
        return self._request("POST", "/api/volume", {"value": int(value)})

    def led(self, r, g, b):
        return self._request("POST", "/api/led", {
            "r": int(r),
            "g": int(g),
            "b": int(b),
        })

    def hand_preset(self, lr, preset):
        return self._request("POST", "/api/hand/preset", {
            "lr": lr,
            "preset": preset,
        })

    def hand_angles(self, lr, angles):
        return self._request("POST", "/api/hand/angles", {
            "lr": lr,
            "angles": [int(v) for v in angles],
        })

    def arm_activate(self):
        return self._request("POST", "/api/arm/activate", {}, timeout=8.0)

    def arm_release(self):
        return self._request("POST", "/api/arm/release", {}, timeout=8.0)

    def arm_joints(self, joints):
        return self._request("POST", "/api/arm/joints", {
            "joints": [float(v) for v in joints],
        })

    def arm_current(self):
        return self._request("GET", "/api/arm/current", timeout=4.0)
