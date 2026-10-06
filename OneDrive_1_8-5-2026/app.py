"""
Integrated Lab Automation Controller
Stage (Ender 3 V2 / Marlin) + Pipette (ESP32)
Flask backend — two independent serial connections
"""

from flask import Flask, render_template, jsonify, request
import serial
import serial.tools.list_ports
import threading
import time
import json
import re
import os

app = Flask(__name__)

# ─── Global State ─────────────────────────────────────────────────────────────

stage   = None   # PrinterConnection
pipette = None   # PipetteConnection

keyframes = []

playback_state = {
    "playing": False,
    "current_frame": -1,
    "stop_requested": False,
}

pipette_status = {
    "is_moving": False,
    "contained_volume": 0.0,
    "factor": 4.5669,
    "capacity": 1000.0,
    "last_msg": "",
    "calibration": {"enabled": False, "points": []}
}

protocol_state = {
    "running": False,
    "paused": False,
    "current_step": -1,
    "iteration": 0,
    "total_steps": 0,
    "total_iterations": 1,
    "stop_requested": False,
    "log": [],
}

CONFIG_FILE = "config.json"
stage_tube_cal = {"tube1_x": 0.0, "tube1_y": 0.0, "tube1_z": 0.0}
stage_tip_cal = {"tip1_x": 0.0, "tip1_y": 0.0, "tip1_z": 0.0}
stage_eject_cal = {"eject_x": 0.0, "eject_y": 0.0, "eject_z": 0.0}
stage_mag_tube_cal = {"mag_x": 0.0, "mag_y": 0.0, "mag_z": 0.0, "x_pitch": 10.0, "y_pitch": 13.0}
next_tip_index = 0
tips_inventory = 20

def load_config():
    global stage_tube_cal, stage_tip_cal, stage_eject_cal, next_tip_index, tips_inventory
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                data = json.load(f)
                if "tube_cal" in data:
                    stage_tube_cal.update(data["tube_cal"])
                if "tip_cal" in data:
                    stage_tip_cal.update(data["tip_cal"])
                if "eject_cal" in data:
                    stage_eject_cal.update(data["eject_cal"])
                if "mag_tube_cal" in data:
                    stage_mag_tube_cal.update(data["mag_tube_cal"])
                if "next_tip_index" in data:
                    next_tip_index = int(data["next_tip_index"])
                if "tips_inventory" in data:
                    tips_inventory = int(data["tips_inventory"])
        except Exception as e:
            print("Error loading config:", e)

def save_config():
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump({
                "tube_cal": stage_tube_cal,
                "tip_cal": stage_tip_cal,
                "eject_cal": stage_eject_cal,
                "mag_tube_cal": stage_mag_tube_cal,
                "next_tip_index": next_tip_index,
                "tips_inventory": tips_inventory
            }, f)
    except Exception as e:
        print("Error saving config:", e)

load_config()

# ─── Stage Serial Connection (Marlin G-code) ──────────────────────────────────

class PrinterConnection:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=10)
        self.lock = threading.Lock()
        time.sleep(2)
        self.ser.flushInput()

    def send(self, cmd):
        with self.lock:
            self.ser.write(f"{cmd.strip()}\n".encode())
            import time, re
            deadline = time.time() + 90
            while time.time() < deadline:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                if line == "ok" or line.startswith("ok "):
                    return True
                if "error" in line.lower():
                    raise RuntimeError(f"Marlin: {line}")
            raise TimeoutError(f"No ok for: {cmd}")

    def get_position(self):
        with self.lock:
            if self.ser.in_waiting:
                self.ser.read(self.ser.in_waiting)
            self.ser.write(b"M114\n")
            import time, re
            deadline = time.time() + 5
            ret = None
            while time.time() < deadline:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                m = re.search(r"X:([\d.-]+)\s+Y:([\d.-]+)\s+Z:([\d.-]+)", line)
                if m:
                    ret = {
                        "x": round(float(m.group(1)), 2),
                        "y": round(float(m.group(2)), 2),
                        "z": round(float(m.group(3)), 2),
                    }
                if line == "ok" or line.startswith("ok "):
                    break
            return ret

    def close(self):
        try: self.ser.close()
        except: pass


# ─── Pipette Serial Connection (ESP32 text protocol) ─────────────────────────

class PipetteConnection:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self._lock  = threading.Lock()
        self._done  = threading.Event()
        self._alive = True
        self._buf   = ""
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        time.sleep(2)
        self.ser.flushInput()
        self._thread.start()
        self._write("GET_FACTOR\n")
        self._write("GET_STATE\n")

    def _write(self, data):
        with self._lock:
            self.ser.write(data.encode())

    def send(self, cmd):
        """Fire-and-forget: send command, clear done event, mark moving."""
        cmd = cmd.strip()
        self._done.clear()
        pipette_status["is_moving"] = True
        pipette_status["last_msg"] = f"→ {cmd}"
        self._write(cmd + "\n")

    def wait_done(self, timeout=60):
        return self._done.wait(timeout=timeout)

    def _read_loop(self):
        while self._alive:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if chunk:
                    self._buf += chunk.decode("utf-8", errors="ignore")
                    while "\n" in self._buf:
                        line, self._buf = self._buf.split("\n", 1)
                        line = line.strip()
                        if line:
                            self._handle(line)
            except Exception:
                time.sleep(0.05)

    def _handle(self, line):
        pipette_status["last_msg"] = line
        if "DONE" in line:
            pipette_status["is_moving"] = False
            self._done.set()
        elif line.startswith("FACTOR:") or line.startswith("FACTOR_SAVED:"):
            try:
                pipette_status["factor"] = float(line.split(":")[1])
            except: pass
        elif line.startswith("CAPACITY_SAVED:"):
            try:
                pipette_status["capacity"] = float(line.split(":")[1])
            except: pass
        elif line.startswith("STATE:"):
            parts = line[6:].split(",")
            if len(parts) >= 5:
                try:
                    self.stop1 = int(parts[1])
                    self.stop2 = int(parts[2])
                    self.factor = float(parts[3])
                    self.capacity = float(parts[4])
                    pipette_status["factor"]   = self.factor
                    pipette_status["capacity"] = self.capacity
                except: pass
            pipette_status["is_moving"] = False
            self._done.set()
        elif line in ("HOME_OK", "HOME_ABORTED"):
            pipette_status["is_moving"] = False
            self._done.set()
        elif line.startswith("CAL_DATA:"):
            parts = line[9:].split(",")
            if parts:
                pipette_status["calibration"]["enabled"] = (parts[0] == "ON")
                pts = []
                for p in parts[1:]:
                    if ":" in p:
                        t, m = p.split(":")
                        try: pts.append({"target": float(t), "measured": float(m)})
                        except: pass
                pipette_status["calibration"]["points"] = pts

    def close(self):
        self._alive = False
        try: self.ser.close()
        except: pass


# ─── Protocol Runner ──────────────────────────────────────────────────────────

def proto_log(msg):
    ts = time.strftime("%H:%M:%S")
    protocol_state["log"].append({"ts": ts, "msg": msg})
    if len(protocol_state["log"]) > 300:
        protocol_state["log"] = protocol_state["log"][-150:]


def _exec_step(step, travel_speed, safe_lift):
    stype = step.get("type", "")

    if stype == "move":
        if not stage: raise RuntimeError("Stage not connected")
        x = float(step.get("x", 0))
        y = float(step.get("y", 0))
        z = float(step.get("z", 0))
        stage.send("G90")
        stage.send(f"G1 Z{z + safe_lift:.3f} F1200")
        stage.send("M400")
        time.sleep(0.2)
        stage.send(f"G1 X{x:.3f} Y{y:.3f} F{travel_speed}")
        stage.send("M400")
        time.sleep(0.2)
        stage.send(f"G1 Z{z:.3f} F1200")
        stage.send("M400")
        time.sleep(0.2)
        proto_log(f"  -> Absolute Move X={x:.2f} Y={y:.2f} Z={z:.2f}")

    elif stype == "move_tube":
        if not stage: raise RuntimeError("Stage not connected")
        tube = int(step.get("tube", 1))
        tube_idx = tube - 1
        row = tube_idx // 10
        col = tube_idx % 10
        x = stage_tube_cal["tube1_x"] + (col * 19.44)
        y = stage_tube_cal["tube1_y"] - (row * 32.0)
        z = stage_tube_cal["tube1_z"] - (row * 7.0)
        
        proto_log(f"  -> Moving to Tube {tube}: X={x:.2f} Y={y:.2f} Z={z:.2f} (Base Y={stage_tube_cal['tube1_y']})")
        
        stage.send("G90")
        stage.send(f"G1 Z{stage_tube_cal['tube1_z'] + safe_lift:.3f} F1200")
        stage.send("M400")
        time.sleep(0.2)
        stage.send(f"G1 X{x:.3f} Y{y:.3f} F{travel_speed}")
        stage.send("M400")
        time.sleep(0.2)
        stage.send(f"G1 Z{z:.3f} F1200")
        stage.send("M400")
        time.sleep(0.2)

    elif stype == "move_mag_tube":
        if not stage: raise RuntimeError("Stage not connected")
        tube = int(step.get("tube", 1))
        tube_idx = tube - 1
        row = tube_idx // 5
        col = tube_idx % 5
        x_pitch = stage_mag_tube_cal.get("x_pitch", 19.44)
        y_pitch = stage_mag_tube_cal.get("y_pitch", 32.0)
        x = stage_mag_tube_cal["mag_x"] + (col * x_pitch)
        y = stage_mag_tube_cal["mag_y"] - (row * y_pitch)
        z = stage_mag_tube_cal["mag_z"]
        
        proto_log(f"  -> Moving to Mag Tube {tube}: X={x:.2f} Y={y:.2f} Z={z:.2f}")
        
        stage.send("G90")
        stage.send(f"G1 Z{stage_mag_tube_cal['mag_z'] + safe_lift:.3f} F1200")
        stage.send("M400")
        time.sleep(0.2)
        stage.send(f"G1 X{x:.3f} Y{y:.3f} F{travel_speed}")
        stage.send("M400")
        time.sleep(0.2)
        stage.send(f"G1 Z{z:.3f} F1200")
        stage.send("M400")
        time.sleep(0.2)

    elif stype == "prep":
        if not pipette: raise RuntimeError("Pipette not connected")
        if stage:
            max_z = max(
                stage_tube_cal.get("tube1_z", 0.0),
                stage_tip_cal.get("tip1_z", 0.0),
                stage_eject_cal.get("eject_z", 0.0)
            )
            travel_z = max_z + safe_lift
            proto_log(f"  -> Lifting Z to safe height {travel_z:.2f}mm before PREP")
            stage.send("G90")
            stage.send(f"G1 Z{travel_z:.3f} F1200")
            stage.send("M400")
            time.sleep(0.2)
        pipette.send("PREP")
        if not pipette.wait_done(60): raise TimeoutError("Prep timeout")
    elif stype == "pause":
        app.logger.info("Protocol paused by pause step")
        state.protocol_paused = True
        while state.protocol_paused and state.protocol_running:
            time.sleep(0.5)
    elif stype == "mix":
        if not pipette: raise RuntimeError("Pipette not connected")
        vol = float(step.get("volume", 50))
        times = int(step.get("times", 3))
        for _ in range(times):
            pipette.send(f"VOL:{vol:.3f}")
            if not pipette.wait_done(60): raise TimeoutError("Mix asp timeout")
            time.sleep(0.5)
            pipette.send(f"DISP_VOL:{vol:.3f}")
            if not pipette.wait_done(60): raise TimeoutError("Mix disp timeout")
            time.sleep(0.5)
    elif stype == "shake_y":
        if not stage: raise RuntimeError("Stage not connected")
        amp = float(step.get("amplitude", 0.2))
        duration = float(step.get("duration", 5.0))
        speed = int(step.get("speed", 15000))
        cycles = max(1, int(duration * 50))
        proto_log(f"  -> High-frequency Y shake: amp={amp:.2f}mm, duration={duration:.1f}s ({cycles} cycles)")
        
        # Boost acceleration to maximum for fast reversals
        stage.send("M201 Y8000")
        stage.send("M204 P8000 T8000")
        
        stage.send("G91")
        stage.send(f"G1 Y{amp:.3f} F{speed}")
        for _ in range(cycles):
            if protocol_state["stop_requested"]: break
            stage.send(f"G1 Y{-2*amp:.3f} F{speed}")
            stage.send(f"G1 Y{2*amp:.3f} F{speed}")
        stage.send(f"G1 Y{-amp:.3f} F{speed}")
        stage.send("M400")
        
        # Restore normal operation acceleration
        stage.send("M201 X2000 Y2000 Z250")
        stage.send("M204 P1500 T1500")
        stage.send("G90")
    elif stype == "aspirate":
        if not pipette: raise RuntimeError("Pipette not connected")
        vol = step.get("volume")
        pipette.send(f"VOL:{float(vol):.3f}" if vol else "ASP")
        if not pipette.wait_done(60): raise TimeoutError("Aspirate timeout")
        if vol:
            pipette_status["contained_volume"] = float(vol)
        else:
            pipette_status["contained_volume"] = pipette_status["capacity"]

    elif stype == "dispense":
        if not pipette: raise RuntimeError("Pipette not connected")
        vol = step.get("volume")
        if vol:
            pipette.send(f"DISP_VOL:{float(vol):.3f}")
            if not pipette.wait_done(60): raise TimeoutError("Dispense timeout")
            pipette_status["contained_volume"] = max(0,
                pipette_status["contained_volume"] - float(vol))
        else:
            pipette.send("DISP")
            if not pipette.wait_done(60): raise TimeoutError("Dispense timeout")
            pipette_status["contained_volume"] = 0.0
    elif stype == "wait":
        secs = float(step.get("seconds", 1))
        t0 = time.time()
        while time.time() - t0 < secs:
            if protocol_state["stop_requested"]: break
            time.sleep(0.1)

    elif stype == "home_stage":
        if not stage: raise RuntimeError("Stage not connected")
        stage.send("G28 X Y")
        stage.send("G28 Z")
        stage.send("M84 S0") # Keep holding torque on indefinitely
        stage.send("G90")

    elif stype == "get_tip":
        global next_tip_index, tips_inventory
        if not stage: raise RuntimeError("Stage not connected")
        if tips_inventory <= 0:
            raise RuntimeError("No tips left in inventory!")
        
        col = next_tip_index // 5
        row = next_tip_index % 5
        x = stage_tip_cal["tip1_x"] + (col * 9.0)
        y = stage_tip_cal["tip1_y"] + (row * 9.0)
        z = stage_tip_cal["tip1_z"]
        
        proto_log(f"  -> Picking up Tip {next_tip_index + 1} at X={x:.2f} Y={y:.2f} (Inventory: {tips_inventory})")
        
        stage.send("G90")
        stage.send(f"G1 Z{z + safe_lift:.3f} F1200")
        stage.send("M400")
        time.sleep(0.2)
        stage.send(f"G1 X{x:.3f} Y{y:.3f} F{travel_speed}")
        stage.send("M400")
        time.sleep(0.2)
        # First press
        stage.send(f"G1 Z{z - 8.500:.3f} F1200")
        # Back off slightly
        stage.send(f"G1 Z{z:.3f} F1200")
        # Second press
        stage.send(f"G1 Z{z - 8.500:.3f} F1200")
        # Lift back to travel height
        stage.send(f"G1 Z{z + safe_lift:.3f} F1200")
        stage.send("M400")
        time.sleep(0.2)
        
        next_tip_index = (next_tip_index + 1) % 20
        tips_inventory = max(0, tips_inventory - 1)
        save_config()

    elif stype == "eject_tip":
        if not stage: raise RuntimeError("Stage not connected")
        step_x = float(step.get("x", 0))
        step_y = float(step.get("y", 0))
        step_z = float(step.get("z", 0))
        
        x = step_x if step_x != 0 else stage_eject_cal["eject_x"]
        y = step_y if step_y != 0 else stage_eject_cal["eject_y"]
        z = step_z if step_z != 0 else stage_eject_cal["eject_z"]
        
        proto_log(f"  -> Ejecting Tip: slide in, pull up to travel height")
        
        stage.send("G90")
        # 1. Move to 20mm left of slot at safe height
        stage.send(f"G1 Z{z + safe_lift:.3f} F1200")
        stage.send("M400")
        time.sleep(0.2)
        stage.send(f"G1 X{x - 20.0:.3f} Y{y:.3f} F{travel_speed}")
        stage.send("M400")
        time.sleep(0.2)
        # 2. Lower Z to slot height
        stage.send(f"G1 Z{z:.3f} F1200")
        stage.send("M400")
        time.sleep(0.2)
        # 3. Slide right 20mm into the slot
        stage.send(f"G1 X{x:.3f} F1500")
        stage.send("M400")
        time.sleep(0.2)
        # 4. Pull up to travel height (z + safe_lift) to strip tip
        stage.send(f"G1 Z{z + safe_lift:.3f} F1200")
        stage.send("M400")
        time.sleep(0.2)
        # 5. Move back left 20mm to exit the slot
        stage.send(f"G1 X{x - 20.0:.3f} F{travel_speed}")
        stage.send("M400")
        time.sleep(0.2)




def _run_protocol(steps, loop_count, travel_speed, safe_lift):
    protocol_state["running"] = True
    protocol_state["stop_requested"] = False
    protocol_state["paused"] = False
    protocol_state["total_steps"] = len(steps)
    protocol_state["total_iterations"] = loop_count
    
    try:
        # Auto-home pipette before starting protocol if connected
        if pipette:
            proto_log("Auto-homing pipette before protocol start...")
            pipette.send("AUTO_HOME")
            if not pipette.wait_done(60):
                raise TimeoutError("Pipette auto-home timeout")

        for iteration in range(loop_count):
            if protocol_state["stop_requested"]: break
            protocol_state["iteration"] = iteration + 1
            proto_log(f"▶ Iteration {iteration+1}/{loop_count}")
            
            for i, step in enumerate(steps):
                if protocol_state["stop_requested"]: break
                while protocol_state["paused"] and not protocol_state["stop_requested"]:
                    time.sleep(0.5)
                if protocol_state["stop_requested"]: break
                
                protocol_state["current_step"] = i
                stype = step.get("type", "").upper()
                proto_log(f"[{i+1}/{len(steps)}] {stype}")
                
                try:
                    _exec_step(step, travel_speed, safe_lift)
                except Exception as e:
                    proto_log(f"Error at step {i+1}: {str(e)}")
                    protocol_state["stop_requested"] = True
                    break
                
        if not protocol_state["stop_requested"]:
            proto_log("✓ Protocol complete")
        else:
            proto_log("⏹ Protocol stopped")
            
    except Exception as e:
        proto_log(f"Protocol crashed: {str(e)}")
    finally:
        protocol_state["running"] = False
        protocol_state["current_step"] = -1

# ==========================================
# ── Keyframes ──────────────────────────────────────────────────────────────────


# ─── Flask Routes ──────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/ports")
def api_ports():
    ports = [{"device": p.device, "description": p.description}
             for p in serial.tools.list_ports.comports()]
    return jsonify(ports)


@app.route("/api/status")
def api_status():
    return jsonify({
        "stage_connected":   stage   is not None,
        "pipette_connected": pipette is not None,
        "playback": {
            "playing":       playback_state["playing"],
            "current_frame": playback_state["current_frame"],
            "total_frames":  len(keyframes),
        },
        "pipette": {**pipette_status, "connected": pipette is not None},
        "protocol": {k: protocol_state[k] for k in
            ["running","paused","current_step","iteration",
             "total_steps","total_iterations","stop_requested"]},
        "tube_cal": stage_tube_cal,
        "mag_tube_cal": stage_mag_tube_cal,
        "tip_cal": stage_tip_cal,
        "eject_cal": stage_eject_cal,
        "next_tip_index": next_tip_index,
        "tips_inventory": tips_inventory
    })

@app.route("/api/keyframes", methods=["GET"])
def api_kf_get():
    return jsonify(keyframes)


@app.route("/api/keyframes", methods=["POST"])
def api_kf_add():
    global keyframes
    if not stage: return jsonify({"success": False, "error": "Stage not connected"})
    data  = request.json or {}
    label = data.get("label") or f"Pos {len(keyframes) + 1}"
    try:
        pos = stage.get_position()
        if not pos: return jsonify({"success": False, "error": "Can't read position"})
        kf = {"id": len(keyframes), "label": label, **pos}
        keyframes.append(kf)
        return jsonify({"success": True, "keyframes": keyframes})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route("/api/keyframes/<int:idx>", methods=["DELETE"])
def api_kf_delete(idx):
    global keyframes
    if 0 <= idx < len(keyframes):
        keyframes.pop(idx)
        for i, kf in enumerate(keyframes): kf["id"] = i
    return jsonify({"success": True, "keyframes": keyframes})



# ── Stage ──────────────────────────────────────────────────────────────────────

@app.route("/api/stage/connect", methods=["POST"])
def api_stage_connect():
    global stage
    data = request.json or {}
    try:
        if stage: stage.close()
        stage = PrinterConnection(data["port"], int(data.get("baud", 115200)))
        stage.send("M211 S0") # Disable Marlin software endstops
        stage.send("M203 X400 Y400 Z20") # Raise maximum feedrate limits (X/Y=400, Z=20 mm/s)
        stage.send("M201 X2000 Y2000 Z250") # Set higher acceleration limits
        stage.send("M204 P800 T800") # Set default travel acceleration to 800 mm/s^2
        stage.send("G90")
        return jsonify({"success": True})
    except Exception as e:
        stage = None
        return jsonify({"success": False, "error": str(e)})


@app.route("/api/stage/disconnect", methods=["POST"])
def api_stage_disconnect():
    global stage
    if stage: stage.close(); stage = None
    return jsonify({"success": True})


@app.route("/api/stage/position")
def api_stage_position():
    if not stage: return jsonify({"success": False, "error": "Not connected"})
    try:
        pos = stage.get_position()
        return jsonify({"success": True, "position": pos} if pos
                       else {"success": False, "error": "No response"})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route("/api/stage/home_xy", methods=["POST"])
def api_stage_home_xy():
    if not stage: return jsonify({"success": False, "error": "Not connected"})
    try:
        stage.send("G28 X Y")
        stage.send("G28 Z")
        stage.send("M84 S0") # Keep holding torque on indefinitely
        stage.send("G90")
        pos = stage.get_position()
        return jsonify({"success": True, "position": pos})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route("/api/stage/set_z_origin", methods=["POST"])
def api_stage_set_z_origin():
    if not stage: return jsonify({"success": False, "error": "Not connected"})
    try:
        stage.send("G92 Z0")
        pos = stage.get_position()
        return jsonify({"success": True, "position": pos})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route("/api/stage/jog", methods=["POST"])
def api_stage_jog():
    if not stage: return jsonify({"success": False, "error": "Not connected"})
    data   = request.json or {}
    axis   = data.get("axis", "X").upper()
    amount = float(data.get("amount", 1.0))
    speed  = int(data.get("speed", 300 if axis == "Z" else 3000))
    try:
        stage.send("G91")
        stage.send(f"G1 {axis}{amount:.3f} F{speed}")
        stage.send("M400")
        stage.send("G90")
        pos = stage.get_position()
        return jsonify({"success": True, "position": pos})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route("/api/stage/move_to", methods=["POST"])
def api_stage_move_to():
    """Absolute XY move (with optional Z). Used by virtual bed click."""
    if not stage: return jsonify({"success": False, "error": "Not connected"})
    data      = request.json or {}
    print("MOVE TO RECEIVED:", data)
    x         = float(data["x"])
    y         = float(data["y"])
    z         = data.get("z")
    speed     = int(data.get("speed", 3000))
    safe_lift = float(data.get("safe_lift", 5.0))
    try:
        stage.send("G90")
        stage.send("M211 S0") # Ensure endstops are disabled
        if z is not None:
            stage.send(f"G1 Z{float(z) + safe_lift:.3f} F1200")
        else:
            stage.send("G91")
            stage.send(f"G1 Z{safe_lift:.3f} F1200")
            stage.send("G90")
        stage.send(f"G1 X{x:.3f} Y{y:.3f} F{speed}")
        if z is not None:
            stage.send(f"G1 Z{float(z):.3f} F1200")
        else:
            stage.send("G91")
            stage.send(f"G1 Z{-safe_lift:.3f} F1200")
            stage.send("G90")
        stage.send("M400")
        pos = stage.get_position()
        return jsonify({"success": True, "position": pos})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route("/api/stage/tube_cal", methods=["GET", "POST"])
def api_stage_tube_cal():
    global stage_tube_cal
    if request.method == "POST":
        data = request.json or {}
        if "tube1_x" in data: stage_tube_cal["tube1_x"] = float(data["tube1_x"])
        if "tube1_y" in data: stage_tube_cal["tube1_y"] = float(data["tube1_y"])
        if "tube1_z" in data: stage_tube_cal["tube1_z"] = float(data["tube1_z"])
        save_config()
        return jsonify({"success": True, "tube_cal": stage_tube_cal})
    return jsonify(stage_tube_cal)

@app.route("/api/stage/set_tube_origin_here", methods=["POST"])
def api_stage_set_tube_origin_here():
    global stage_tube_cal
    if not stage: return jsonify({"success": False, "error": "Not connected"})
    try:
        pos = stage.get_position()
        if pos:
            stage_tube_cal["tube1_x"] = pos["x"]
            stage_tube_cal["tube1_y"] = pos["y"]
            stage_tube_cal["tube1_z"] = pos["z"]
            save_config()
            return jsonify({"success": True, "tube_cal": stage_tube_cal})
        return jsonify({"success": False, "error": "No response from M114"})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})

@app.route("/api/stage/set_tip_origin_here", methods=["POST"])
def api_stage_set_tip_origin_here():
    global stage_tip_cal
    if not stage: return jsonify({"success": False, "error": "Not connected"})
    try:
        pos = stage.get_position()
        if pos:
            stage_tip_cal["tip1_x"] = pos["x"]
            stage_tip_cal["tip1_y"] = pos["y"]
            stage_tip_cal["tip1_z"] = pos["z"]
            save_config()
            return jsonify({"success": True, "tip_cal": stage_tip_cal})
        return jsonify({"success": False, "error": "No response from M114"})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})

@app.route("/api/stage/set_tips_state", methods=["POST"])
def api_stage_set_tips_state():
    global next_tip_index, tips_inventory
    data = request.json or {}
    if "next_tip_index" in data:
        next_tip_index = int(data["next_tip_index"])
    if "tips_inventory" in data:
        tips_inventory = int(data["tips_inventory"])
    save_config()
    return jsonify({
        "success": True,
        "next_tip_index": next_tip_index,
        "tips_inventory": tips_inventory
    })

@app.route("/api/stage/set_eject_origin_here", methods=["POST"])
def api_stage_set_eject_origin_here():
    global stage_eject_cal
    if not stage: return jsonify({"success": False, "error": "Not connected"})
    try:
        pos = stage.get_position()
        if pos:
            stage_eject_cal["eject_x"] = pos["x"]
            stage_eject_cal["eject_y"] = pos["y"]
            stage_eject_cal["eject_z"] = pos["z"]
            save_config()
            return jsonify({"success": True, "eject_cal": stage_eject_cal})
        return jsonify({"success": False, "error": "No response from M114"})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


# ==========================================── Keyframes ──────────────────────────────────────────────────────────────────

@app.route("/api/keyframes/<int:idx>/label", methods=["PUT"])
def api_kf_label(idx):
    global keyframes
    label = (request.json or {}).get("label", "").strip()
    if 0 <= idx < len(keyframes) and label:
        keyframes[idx]["label"] = label
    return jsonify({"success": True, "keyframes": keyframes})


@app.route("/api/keyframes/clear", methods=["POST"])
def api_kf_clear():
    global keyframes
    keyframes = []
    return jsonify({"success": True, "keyframes": keyframes})


@app.route("/api/keyframes/import", methods=["POST"])
def api_kf_import():
    global keyframes
    data = request.json
    if isinstance(data, list):
        keyframes = data
        for i, kf in enumerate(keyframes): kf["id"] = i
    return jsonify({"success": True, "keyframes": keyframes})


# ── Stage Playback ─────────────────────────────────────────────────────────────

@app.route("/api/playback/start", methods=["POST"])
def api_play_start():
    global playback_state
    if not stage:      return jsonify({"success": False, "error": "Stage not connected"})
    if not keyframes:  return jsonify({"success": False, "error": "No keyframes"})
    if playback_state["playing"]: return jsonify({"success": False, "error": "Already playing"})
    data      = request.json or {}
    speed     = int(data.get("speed", 3000))
    dwell     = float(data.get("dwell", 1.0))
    safe_lift = float(data.get("safe_lift", 5.0))

    def run():
        playback_state.update({"playing": True, "stop_requested": False, "current_frame": -1})
        try:
            stage.send("G90")
            for i, kf in enumerate(keyframes):
                if playback_state["stop_requested"]: break
                playback_state["current_frame"] = i
                stage.send(f"G1 Z{kf['z'] + safe_lift:.3f} F1200")
                stage.send(f"G1 X{kf['x']:.3f} Y{kf['y']:.3f} F{speed}")
                stage.send(f"G1 Z{kf['z']:.3f} F1200")
                stage.send("M400")
                time.sleep(dwell)
        except Exception as e:
            print(f"[Playback] {e}")
        finally:
            playback_state.update({"playing": False, "current_frame": -1})

    threading.Thread(target=run, daemon=True).start()
    return jsonify({"success": True})


@app.route("/api/playback/stop", methods=["POST"])
def api_play_stop():
    playback_state["stop_requested"] = True
    return jsonify({"success": True})


# ── Pipette ────────────────────────────────────────────────────────────────────

@app.route("/api/pipette/connect", methods=["POST"])
def api_pip_connect():
    global pipette
    data = request.json or {}
    try:
        if pipette: pipette.close()
        pipette = PipetteConnection(data["port"], int(data.get("baud", 115200)))
        return jsonify({"success": True})
    except Exception as e:
        pipette = None
        return jsonify({"success": False, "error": str(e)})


@app.route("/api/pipette/disconnect", methods=["POST"])
def api_pip_disconnect():
    global pipette
    if pipette: pipette.close(); pipette = None
    return jsonify({"success": True})


@app.route("/api/pipette/cmd", methods=["POST"])
def api_pip_cmd():
    """Send a raw command to the pipette (non-blocking)."""
    if not pipette: return jsonify({"success": False, "error": "Not connected"})
    cmd = (request.json or {}).get("cmd", "").strip()
    if not cmd: return jsonify({"success": False, "error": "Empty command"})
    try:
        pipette.send(cmd)
        return jsonify({"success": True})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route("/api/pipette/status")
def api_pip_status():
    return jsonify({**pipette_status, "connected": pipette is not None})


@app.route("/api/pipette/calibration")
def api_pip_cal_get():
    if not pipette: return jsonify({"success": False, "error": "Not connected"})
    pipette.send("GET_CAL")
    return jsonify({"success": True, "calibration": pipette_status["calibration"]})

@app.route("/api/pipette/calibration/enable", methods=["POST"])
def api_pip_cal_enable():
    if not pipette: return jsonify({"success": False, "error": "Not connected"})
    enable = (request.json or {}).get("enable", False)
    pipette.send(f"SET_CAL_ENABLE:{1 if enable else 0}")
    pipette.send("GET_CAL")
    return jsonify({"success": True})

@app.route("/api/pipette/calibration/clear", methods=["POST"])
def api_pip_cal_clear():
    if not pipette: return jsonify({"success": False, "error": "Not connected"})
    pipette.send("CLEAR_CAL")
    pipette.send("GET_CAL")
    return jsonify({"success": True})

@app.route("/api/pipette/calibration/add", methods=["POST"])
def api_pip_cal_add():
    if not pipette: return jsonify({"success": False, "error": "Not connected"})
    data = request.json or {}
    t, m = data.get("target"), data.get("measured")
    if t is None or m is None: return jsonify({"success": False, "error": "Missing params"})
    pipette.send(f"ADD_CAL:{t},{m}")
    pipette.send("GET_CAL")
    return jsonify({"success": True})


# ── Protocol ───────────────────────────────────────────────────────────────────

@app.route("/api/protocol/start", methods=["POST"])
def api_proto_start():
    if protocol_state["running"]:
        return jsonify({"success": False, "error": "Already running"})
    data      = request.json or {}
    steps     = data.get("steps", [])
    loop_count  = int(data.get("loop_count", 1))
    travel_speed = int(data.get("travel_speed", 3000))
    safe_lift  = float(data.get("safe_lift", 5.0))
    if not steps: return jsonify({"success": False, "error": "No steps"})
    protocol_state["log"] = []
    threading.Thread(
        target=_run_protocol,
        args=(steps, loop_count, travel_speed, safe_lift),
        daemon=True
    ).start()
    return jsonify({"success": True})


@app.route("/api/protocol/stop", methods=["POST"])
def api_proto_stop():
    protocol_state["stop_requested"] = True
    protocol_state["paused"] = False
    return jsonify({"success": True})


@app.route("/api/protocol/pause", methods=["POST"])
def api_proto_pause():
    if protocol_state["running"]:
        protocol_state["paused"] = not protocol_state.get("paused", False)
    return jsonify({"success": True, "paused": protocol_state.get("paused", False)})


@app.route("/api/protocol/log")
def api_proto_log():
    since = int(request.args.get("since", 0))
    return jsonify(protocol_state["log"][since:])


@app.route("/api/protocol/status")
def api_proto_status():
    return jsonify({k: protocol_state[k] for k in
        ["running","paused","current_step","iteration",
         "total_steps","total_iterations","stop_requested"]})


# ─── Entry Point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("Lab Automation Controller -> http://127.0.0.1:5000")
    app.run(host="127.0.0.1", port=5000, debug=False)
