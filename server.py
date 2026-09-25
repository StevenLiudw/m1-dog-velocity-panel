#!/usr/bin/env python3
"""Loopback-only control panel for the M1 DDS velocity interface."""

import argparse
import json
import math
import os
import secrets
import shlex
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


HERE = Path(__file__).resolve().parent
LIMITS = (2.0, 0.6, 0.8)  # deployed robot's DDS subscriber clamps
MODES = {"damping": 1, "sit": 2, "stand": 3, "walk": 4}


class Publisher:
    def __init__(self, iface: str, simulate: bool, transport: str, robot: str):
        self.simulate = simulate
        self.transport = transport
        self.robot = robot
        self.lock = threading.Lock()
        self.ready = simulate
        self.message = ("Simulation: no DDS messages are sent" if simulate else
                        "Set --robot USER@ROBOT_IP when starting the panel" if transport == "ssh" and not robot else
                        "Enter the robot SSH password to connect" if transport == "ssh" else
                        "Starting local DDS publisher")
        self.process = None
        self.ready_event = threading.Event()
        if not simulate and transport == "local":
            command = [str(HERE / "dds_bridge")]
            if iface:
                command += ["--iface", iface]
            self.process = subprocess.Popen(
                command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE, text=True, bufsize=1,
            )
            threading.Thread(target=self._read_status, daemon=True).start()

    def _status_line(self, line):
        with self.lock:
            if "DOG_READY" in line:
                self.ready = True
                self.message = "Robot-side DDS publisher ready; no live telemetry acknowledgement"
                self.ready_event.set()
            elif "DOG_ERROR" in line:
                self.message = line.split("DOG_ERROR", 1)[1].strip()
            elif "DOG_WATCHDOG" in line:
                self.message = "DDS publisher stopped an expired velocity command"
            elif "DOG_OK" in line:
                self.message = line.split("DOG_OK", 1)[1].strip()

    def _read_status(self):
        for raw in self.process.stderr:
            line = raw.strip()
            if "DOG_" in line:
                self._status_line(line)
        with self.lock:
            self.ready = False
            self.message = "DDS publisher exited"
            self.ready_event.set()

    def _read_ssh(self, password: str):
        pending = ""
        password_sent = False
        try:
            while True:
                chunk = os.read(self.process.stdout.fileno(), 1024).decode(errors="replace")
                if not chunk:
                    break
                pending += chunk
                if "password:" in pending.lower():
                    if password_sent:
                        with self.lock:
                            self.message = "Robot SSH authentication failed"
                        break
                    self.process.stdin.write((password + "\n").encode())
                    self.process.stdin.flush()
                    password = ""
                    password_sent = True
                    pending = ""
                while "\n" in pending:
                    line, pending = pending.split("\n", 1)
                    if "DOG_" in line:
                        self._status_line(line.strip())
                    elif "Permission denied" in line:
                        with self.lock:
                            self.message = "Robot SSH authentication failed"
                if len(pending) > 4096:
                    pending = pending[-4096:]
        except OSError:
            pass
        finally:
            password = ""
            if self.process.poll() is None:
                self.process.terminate()
            with self.lock:
                self.ready = False
                if self.message not in ("Robot SSH authentication failed",):
                    self.message = "Robot SSH publisher disconnected"
                self.ready_event.set()

    def connect(self, password: str):
        if self.simulate or self.transport != "ssh":
            raise ValueError("SSH connection is unavailable in this mode")
        if not self.robot:
            raise ValueError("Start the panel with --robot USER@ROBOT_IP")
        if not isinstance(password, str) or not password:
            raise ValueError("Enter the robot SSH password")
        with self.lock:
            if self.ready and self.process and self.process.poll() is None:
                return
            if self.process and self.process.poll() is None:
                raise RuntimeError("Robot SSH connection is already starting")
            command = ["ssh", "-tt", "-o", "StrictHostKeyChecking=yes",
                       "-o", "PreferredAuthentications=password",
                       "-o", "PubkeyAuthentication=no", "-o", "ConnectTimeout=5",
                       self.robot,
                       "dg_fsm_m1/build/test_repo/dog_dds_bridge", "--iface", "lo"]
            wrapper = ["script", "-q", "-f", "-c", shlex.join(command), "/dev/null"]
            self.process = subprocess.Popen(wrapper, stdin=subprocess.PIPE,
                                            stdout=subprocess.PIPE,
                                            stderr=subprocess.STDOUT, bufsize=0)
            self.message = "Connecting to robot over SSH"
            self.ready_event.clear()
        threading.Thread(target=self._read_ssh, args=(password,), daemon=True).start()
        if not self.ready_event.wait(12):
            self.process.terminate()
            raise RuntimeError("Timed out waiting for the robot DDS publisher")
        if not self.snapshot()["ready"]:
            raise RuntimeError(self.snapshot()["message"])

    def send(self, line: str):
        with self.lock:
            if self.simulate:
                self.message = f"Simulation: {line}"
                return
            if not self.ready or self.process is None or self.process.poll() is not None:
                raise RuntimeError("DDS publisher is not running")
            try:
                if self.transport == "ssh":
                    self.process.stdin.write((line + "\n").encode())
                    self.process.stdin.flush()
                else:
                    self.process.stdin.write(line + "\n")
                    self.process.stdin.flush()
            except (BrokenPipeError, OSError) as error:
                self.ready = False
                raise RuntimeError("DDS publisher connection failed") from error

    def snapshot(self):
        with self.lock:
            alive = self.simulate or (self.process is not None and self.process.poll() is None)
            return {"ready": self.ready and alive, "message": self.message,
                    "simulate": self.simulate, "transport": self.transport}

    def close(self):
        if self.simulate or self.process is None or self.process.poll() is not None:
            return
        try:
            self.send("STOP")
            self.send("QUIT")
            self.process.wait(timeout=2)
        except (RuntimeError, subprocess.TimeoutExpired):
            self.process.terminate()


class Control:
    def __init__(self, publisher: Publisher):
        self.publisher = publisher
        self.lock = threading.Lock()
        self.mode = "none"
        self.armed = False
        self.velocity = (0.0, 0.0, 0.0)
        self.last_drive = 0.0
        self.last_seq = -1
        threading.Thread(target=self._watchdog, daemon=True).start()

    def _watchdog(self):
        while True:
            time.sleep(0.05)
            with self.lock:
                if (self.velocity != (0.0, 0.0, 0.0) and
                        time.monotonic() - self.last_drive > 0.4):
                    self.velocity = (0.0, 0.0, 0.0)
                    self.armed = False
                    try:
                        self.publisher.send("STOP")
                    except RuntimeError:
                        pass

    def snapshot(self):
        with self.lock:
            publisher = self.publisher.snapshot()
            if not publisher["ready"]:
                self.armed = False
                self.velocity = (0.0, 0.0, 0.0)
            return {**publisher, "mode": self.mode,
                    "armed": self.armed, "velocity": self.velocity,
                    "limits": LIMITS}

    def command(self, payload):
        action = payload.get("action")
        seq = payload.get("seq")
        if type(seq) is not int or seq < 0:
            raise ValueError("Missing command sequence")
        if action == "connect":
            with self.lock:
                if seq <= self.last_seq:
                    return self.snapshot_unlocked()
                self.last_seq = seq
                self.mode = "none"
                self.armed = False
                self.velocity = (0.0, 0.0, 0.0)
            self.publisher.connect(payload.get("password", ""))
            return self.snapshot()
        with self.lock:
            if seq <= self.last_seq:
                return self.snapshot_unlocked()
            self.last_seq = seq
            if action == "stop":
                self.velocity = (0.0, 0.0, 0.0)
                self.armed = False
                self.publisher.send("STOP")
            elif action == "release":
                self.velocity = (0.0, 0.0, 0.0)
                self.publisher.send("STOP")
            elif action == "mode":
                mode = payload.get("mode")
                if mode not in MODES:
                    raise ValueError("Unknown robot mode")
                self.velocity = (0.0, 0.0, 0.0)
                self.armed = False
                self.publisher.send("STOP")
                self.publisher.send(f"MODE {MODES[mode]}")
                self.mode = mode
            elif action == "arm":
                if self.mode != "walk" or not self.publisher.snapshot()["ready"]:
                    raise ValueError("Select Walk and wait for the DDS publisher first")
                self.velocity = (0.0, 0.0, 0.0)
                self.armed = True
                self.publisher.send("STOP")
            elif action == "drive":
                if not self.armed or self.mode != "walk":
                    raise ValueError("Movement controls are disarmed")
                values = payload.get("velocity")
                if not isinstance(values, list) or len(values) != 3:
                    raise ValueError("Velocity must have vx, vy, wz")
                try:
                    velocity = tuple(float(value) for value in values)
                except (TypeError, ValueError) as error:
                    raise ValueError("Invalid velocity") from error
                if not all(math.isfinite(value) and abs(value) <= limit
                           for value, limit in zip(velocity, LIMITS)):
                    raise ValueError("Velocity exceeds deployed robot limits")
                self.publisher.send("VEL " + " ".join(f"{value:.4f}" for value in velocity))
                self.velocity = velocity
                self.last_drive = time.monotonic()
            else:
                raise ValueError("Unknown command")
            return self.snapshot_unlocked()

    def snapshot_unlocked(self):
        return {**self.publisher.snapshot(), "mode": self.mode,
                "armed": self.armed, "velocity": self.velocity,
                "limits": LIMITS}


def handler_factory(control: Control, token: str, port: int):
    html = (HERE / "index.html").read_text().replace("__TOKEN__", token)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            return

        def respond(self, code, payload):
            body = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except BrokenPipeError:
                pass

        def do_GET(self):
            if self.path == "/":
                body = html.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/api/status":
                self.respond(200, control.snapshot())
            else:
                self.respond(404, {"error": "Not found"})

        def do_POST(self):
            if self.path != "/api/command":
                return self.respond(404, {"error": "Not found"})
            if self.headers.get("Origin") not in (None, f"http://127.0.0.1:{port}"):
                return self.respond(403, {"error": "Invalid origin"})
            if self.headers.get("X-Dog-Control-Token") != token:
                return self.respond(403, {"error": "Invalid control token"})
            if self.headers.get("Content-Type", "").split(";", 1)[0] != "application/json":
                return self.respond(415, {"error": "JSON required"})
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if size < 1 or size > 2048:
                    raise ValueError("Invalid request size")
                payload = json.loads(self.rfile.read(size))
                if not isinstance(payload, dict):
                    raise ValueError("JSON object required")
                state = control.command(payload)
                self.respond(200, state)
            except (ValueError, RuntimeError) as error:
                self.respond(400, {"error": str(error), **control.snapshot()})

    return Handler


def main():
    parser = argparse.ArgumentParser(description="M1 dog velocity control panel")
    parser.add_argument("--iface", default="", help="Host network interface connected to the dog")
    parser.add_argument("--transport", choices=("ssh", "local"), default="ssh",
                        help="SSH publisher on robot loopback (default), or direct host DDS")
    parser.add_argument("--robot", default=os.environ.get("DOG_ROBOT_SSH_TARGET", ""),
                        help="Robot SSH target, USER@ROBOT_IP (or DOG_ROBOT_SSH_TARGET)")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--simulate", action="store_true", help="Run the panel without sending DDS")
    args = parser.parse_args()
    if not args.simulate and args.transport == "local" and not (HERE / "dds_bridge").is_file():
        parser.error("DDS publisher missing; run ./build.sh first")
    token = secrets.token_urlsafe(32)
    publisher = Publisher(args.iface, args.simulate, args.transport, args.robot)
    control = Control(publisher)
    server = ThreadingHTTPServer(("127.0.0.1", args.port),
                                 handler_factory(control, token, args.port))
    print(f"Dog control panel: http://127.0.0.1:{args.port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        publisher.close()


if __name__ == "__main__":
    main()
