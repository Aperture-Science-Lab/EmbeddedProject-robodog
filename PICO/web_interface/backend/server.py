from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
import serial
import serial.tools.list_ports
import time
import threading
import queue
import cv2
import numpy as np
import os
import sys
import json
import requests
import socket

# Import modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
try:
    from ekf_module import SpotEKF
except ImportError:
    SpotEKF = None

try:
    from stereo_utils import get_detections, get_cost, get_tracks 
    STEREO_AVAILABLE = True
except ImportError:
    STEREO_AVAILABLE = False


app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ================= Global State =================
class RobotState:
    def __init__(self):
        self.ekf = SpotEKF() if SpotEKF else None
        self.ekf_active = False
        self.latest_ekf_state = {
            "position": [0,0,0],
            "velocity": [0,0,0],
            "orientation": [1,0,0,0],
            "imu": {"ax":0,"ay":0,"az":0},
            "angles": [0,0,0,0],
            "depth": 0.0
        }
        self.serial_port = None
        self.serial_connected = False
        self.serial_lock = threading.Lock()
        
        # Camera Defaults
        # Camera Defaults (Empty = discovered on startup)
        self.cam_url_left = "" 
        self.cam_url_right = ""
        
        self.stereo_enabled = True
        self.base_length = 0.04 # 4cm in meters
        self.focal_length = 700 # pixels (Estimate for 640x480)
        
robot = RobotState()

# ================= Serial Manager =================
def auto_connect_serial():
    ports = list(serial.tools.list_ports.comports())
    target_port = None
    for p in ports:
        if "Pico" in p.description or "USB Serial" in p.description:
            target_port = p.device
            break
    
    if not target_port and len(ports) > 0:
        target_port = ports[0].device 
        
    if target_port:
        try:
            robot.serial_port = serial.Serial(target_port, 115200, timeout=0.1)
            robot.serial_connected = True
            print(f"Connected to {target_port}")
            # Start listener
            threading.Thread(target=serial_listener, daemon=True).start()
        except Exception as e:
            print(f"Failed to connect to {target_port}: {e}")

def serial_listener():
    """Reads lines from serial and parses JSON telemetry"""
    while robot.serial_connected and robot.serial_port:
        try:
            if robot.serial_port.in_waiting:
                line = robot.serial_port.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('{') and line.endswith('}'):
                    try:
                        data = json.loads(line)
                        process_telemetry(data)
                    except json.JSONDecodeError:
                        pass 
                elif len(line) > 0: 
                    # print(f"PICO: {line}") # Reduce log noise
                    pass
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"Serial Read Error: {e}")
            robot.serial_connected = False
            break

def process_telemetry(data):
    robot.latest_ekf_state["angles"] = data.get("angles", [0,0,0,0])
    imu = data.get("imu", {})
    robot.latest_ekf_state["imu"] = imu
    
    if robot.ekf and "ax" in imu:
        imu_f = np.array([imu.get("ax", 0), imu.get("ay", 0), imu.get("az", -9.81)])
        imu_w = np.array([imu.get("gx", 0), imu.get("gy", 0), imu.get("gz", 0)])
        dt = 0.1
        robot.ekf.predict(imu_f, imu_w, dt)
        state = robot.ekf.get_state()
        robot.latest_ekf_state.update(state)

def send_serial_cmd_raw(cmd: str):
    if not robot.serial_connected or not robot.serial_port:
        return False
    try:
        with robot.serial_lock:
            robot.serial_port.write((cmd + "\n").encode())
            robot.serial_port.flush()
        return True
    except Exception as e:
        robot.serial_connected = False
        return False

# ================= Logic Processor =================
def process_high_level_command(cmd: str):
    cmd_map = {
        "WALK": "W", "BACK": "B", "STOP": "S", 
        "LEFT": "A", "RIGHT": "E", "NEUTRAL": "N", "SAVE": "SAVE"
    }
    primitive = cmd_map.get(cmd, cmd)
    send_serial_cmd_raw(primitive)
    return primitive

# ================= Camera & Stereo =================

# Scan Network
def scan_network_for_cameras():
    """Scans 192.168.1.x for devices on port 81 (stream)"""
    found = []
    print("Starting Camera Scan...")
    
    # List of subnets to scan
    subnets = ["192.168.1.", "192.168.0.", "192.168.137.", "192.168.4."]
    
    try:
        hostname = socket.gethostname()
        local_ips = socket.gethostbyname_ex(hostname)[2]
        for ip in local_ips:
            if ip.startswith("192.168."):
                part = ".".join(ip.split(".")[:3]) + "."
                if part not in subnets:
                    subnets.insert(0, part)
    except: pass
    
    print(f"Scanning subnets: {subnets}")
    
    def check_ip(ip):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.3)  # Slightly longer timeout
        try:
            # ESP32-CAM serves the index on port 80, stream on port 81
            result = s.connect_ex((ip, 80))
            if result == 0:
                print(f"Found ESP32-CAM at {ip}")
                # The stream is at port 81
                found.append(f"http://{ip}:81/stream")
        except:
            pass
        finally:
            s.close()
            
    threads = []
    for base_ip in subnets:
        # Scan common DHCP ranges, skip .1 (usually router)
        ranges = list(range(100, 200)) + list(range(2, 21))
        for i in ranges:
            t = threading.Thread(target=check_ip, args=(base_ip + str(i),))
            threads.append(t)
            t.start()
            
    for t in threads:
        t.join()
        
    return found

class VideoStream:
    def __init__(self, src):
        self.src = src
        self.frame = None
        self.running = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        
    def _update(self):
        # Wait for valid source
        while self.running and not (isinstance(self.src, str) and self.src.startswith("http")):
            time.sleep(1)
            
        cap = cv2.VideoCapture(self.src)
        error_count = 0
        while self.running:
            if cap.isOpened():
                ret, frame = cap.read()
                if ret: 
                    with self.lock:
                        self.frame = frame
                    error_count = 0
                else:
                    error_count += 1
                    if error_count > 50: # Restart if stuck
                        cap.release()
                        time.sleep(1)
            else:
                # Disable retrying on local indices 0 or 1. Only retry if it looks like a URL
                if isinstance(self.src, str) and self.src.startswith("http"):
                    try:
                        time.sleep(1)
                        cap.open(self.src)
                    except: pass
                else:
                    # Invalid source, stop trying
                    self.running = False
            time.sleep(0.01) # Yield

streams = {}

def get_stream_frame(src):
    if src not in streams:
        streams[src] = VideoStream(src)
    with streams[src].lock:
        frame = streams[src].frame
        if frame is None:
            return None
        return frame.copy()

def process_stereo():
    """Background task to compute depth from latest left/right frames"""
    while True:
        if robot.stereo_enabled and STEREO_AVAILABLE and robot.cam_url_left != 0 and robot.cam_url_right != 1:
             left = get_stream_frame(robot.cam_url_left)
             right = get_stream_frame(robot.cam_url_right)
             
             if left is not None and right is not None:
                 try:
                     # Very simplified stereo logic for realtime speed
                     # 1. Scale down for performance
                     l_small = cv2.resize(left, (320, 240))
                     r_small = cv2.resize(right, (320, 240))
                     
                     # 2. Simple template matching for center point to find disparity
                     # Convert to gray
                     gray_l = cv2.cvtColor(l_small, cv2.COLOR_BGR2GRAY)
                     gray_r = cv2.cvtColor(r_small, cv2.COLOR_BGR2GRAY)
                     
                     # Define center region of interest in Left
                     h, w = gray_l.shape
                     cx, cy = w//2, h//2
                     patch_size = 20 # 40x40 patch
                     
                     # Safety check
                     if cy-patch_size < 0 or cx-patch_size < 0: continue
                     
                     patch = gray_l[cy-patch_size:cy+patch_size, cx-patch_size:cx+patch_size]
                     
                     # Search stripe in Right (assume rectified, searchable on same Y line)
                     # Search range: from 0 to cx (since disparity should be positive)
                     strip = gray_r[cy-patch_size:cy+patch_size, :]
                     
                     res = cv2.matchTemplate(strip, patch, cv2.TM_CCOEFF_NORMED)
                     min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
                     
                     # Max correlation location in strip
                     match_x = max_loc[0] + patch_size
                     
                     disparity = cx - match_x
                     
                     if disparity > 2: # Min disparity threshold
                         # Z = (f * B) / d
                         # f ~ 700px (for 640 width), scaled to 320 it's 350
                         f = 350.0 
                         B = robot.base_length
                         Z = (f * B) / disparity
                         robot.latest_ekf_state["depth"] = float(Z)
                     else:
                         robot.latest_ekf_state["depth"] = 0.0
                         
                 except Exception as e:
                     print(f"Stereo Error: {e}")
        time.sleep(0.1) # 10Hz stereo calculation

# Start Stereo Loop
threading.Thread(target=process_stereo, daemon=True).start()

def gen_frames(src, is_stereo_feed=False):
    while True:
        frame = get_stream_frame(src)
        if frame is None:
            # Placeholder
            img = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(img, "CONNECTING...", (80, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
            ret, buffer = cv2.imencode('.jpg', img)
        else:
             if is_stereo_feed and robot.latest_ekf_state["depth"] > 0:
                 d = robot.latest_ekf_state["depth"]
                 cv2.putText(frame, f"Depth: {d:.2f}m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                 # Draw center crosshair
                 h, w, _ = frame.shape
                 cv2.line(frame, (w//2-10, h//2), (w//2+10, h//2), (0,0,255), 1)
                 cv2.line(frame, (w//2, h//2-10), (w//2, h//2+10), (0,0,255), 1)
                 
             ret, buffer = cv2.imencode('.jpg', frame)
             
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.05)

# ================= API Endpoints =================

@app.get("/api/status")
def get_status():
    return {
        "serial_connected": robot.serial_connected,
        "ekf_state": robot.latest_ekf_state
    }

@app.post("/api/connect")
def connect_serial_endpoint(port: str = ""):
    if not port:
        auto_connect_serial()
    return {"connected": robot.serial_connected}

@app.post("/api/command")
def send_command(cmd: str):
    primitive = process_high_level_command(cmd)
    return {"status": "sent", "primitive": primitive}

@app.get("/api/video_feed_left")
def video_feed_left():
    return StreamingResponse(gen_frames(robot.cam_url_left, False), media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/api/video_feed_right")
def video_feed_right():
    return StreamingResponse(gen_frames(robot.cam_url_right, True), media_type="multipart/x-mixed-replace; boundary=frame")

@app.post("/api/config/cameras")
def set_cameras(left: str, right: str):
    global streams
    # Stop and clear existing streams so new ones will be created
    for key in list(streams.keys()):
        streams[key].running = False
    streams.clear()
    
    robot.cam_url_left = left 
    robot.cam_url_right = right
    print(f"Cameras configured: LEFT={left}, RIGHT={right}")
    return {"status": "updated", "left": robot.cam_url_left, "right": robot.cam_url_right}

@app.get("/api/scan_cameras")
def scan_api():
    found = scan_network_for_cameras()
    return {"cameras": found}

# Auto-Discovery on Startup
def auto_discover_cameras():
    print("\n=== AUTO-DISCOVERING CAMERAS ===")
    cams = scan_network_for_cameras()
    if len(cams) >= 1:
        robot.cam_url_left = cams[0]
        print(f"LEFT CAM: {cams[0]}")
    if len(cams) >= 2:
        robot.cam_url_right = cams[1]
        print(f"RIGHT CAM: {cams[1]}")
    if len(cams) == 0:
        print("No cameras found. Use the UI to scan again or enter IPs manually.")
    print("=================================\n")

# Initial Auto Connect
try:
    auto_connect_serial()
except:
    pass

# Run camera discovery in background (non-blocking)
threading.Thread(target=auto_discover_cameras, daemon=True).start()
