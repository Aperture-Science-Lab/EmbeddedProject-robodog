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
import asyncio

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
        
        # Serial (USB) connection
        self.serial_port = None
        self.serial_connected = False
        self.serial_lock = threading.Lock()
        
        # TCP (WiFi) connection to Pico
        self.tcp_socket = None
        self.tcp_connected = False
        self.tcp_lock = threading.Lock()
        self.pico_ip = ""  # Will be discovered or set manually
        self.pico_port = 8080
        
        # Latest telemetry from Pico
        self.pico_telemetry = {
            "state": "UNKNOWN",
            "head": {"angle": 90, "swing": False},
            "imu": {"roll": 0, "pitch": 0, "yaw": 0, "ax": 0, "ay": 0, "az": 0},
            "ir": {"front": False, "back": False},
            "pir": {"front": False, "back": False},
            "ldr": {"raw": 0, "percent": 0},
            "gps": {"lat": 0, "lon": 0, "alt": 0, "sats": 0, "fix": False},
            "us": {"left": 0, "right": 0},
            "calibration": {
                "shoulder": [60, 130, 120, 50],
                "elbow": [90, 90, 90, 90],
                "wrist": [50, 130, 50, 130]
            },
            "wifi": False,
            "nano": False
        }
        
        # Camera Defaults (Empty = discovered on startup)
        self.cam_url_left = "" 
        self.cam_url_right = ""
        
        self.stereo_enabled = True
        self.base_length = 0.04 # 4cm in meters
        self.focal_length = 700 # pixels (Estimate for 640x480)
        
robot = RobotState()

# ================= TCP Connection to Pico =================
def tcp_connect_pico(ip: str, port: int = 8080):
    """Connect to Pico's TCP server"""
    try:
        if robot.tcp_socket:
            try:
                robot.tcp_socket.close()
            except: pass
            
        robot.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        robot.tcp_socket.settimeout(5.0)
        robot.tcp_socket.connect((ip, port))
        robot.tcp_socket.settimeout(0.1)  # Non-blocking reads
        robot.tcp_connected = True
        robot.pico_ip = ip
        print(f"[TCP] Connected to Pico at {ip}:{port}")
        
        # Start TCP listener thread
        threading.Thread(target=tcp_listener, daemon=True).start()
        return True
    except Exception as e:
        print(f"[TCP] Failed to connect to {ip}:{port}: {e}")
        robot.tcp_connected = False
        return False

def tcp_listener():
    """Reads data from Pico TCP connection"""
    buffer = ""
    while robot.tcp_connected and robot.tcp_socket:
        try:
            data = robot.tcp_socket.recv(1024)
            if not data:
                print("[TCP] Connection closed by Pico")
                robot.tcp_connected = False
                break
                
            buffer += data.decode('utf-8', errors='ignore')
            
            # Process complete JSON lines
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                line = line.strip()
                if line.startswith('{') and line.endswith('}'):
                    try:
                        telemetry = json.loads(line)
                        if telemetry.get("type") == "telemetry":
                            robot.pico_telemetry = telemetry
                            # Also update EKF state for compatibility
                            imu = telemetry.get("imu", {})
                            robot.latest_ekf_state["imu"] = imu
                        elif "status" in telemetry:
                            print(f"[PICO] Status: {telemetry['status']}")
                    except json.JSONDecodeError:
                        pass
        except socket.timeout:
            pass
        except Exception as e:
            print(f"[TCP] Read error: {e}")
            robot.tcp_connected = False
            break
        time.sleep(0.01)

def tcp_send_command(cmd: str):
    """Send command to Pico via TCP"""
    if not robot.tcp_connected or not robot.tcp_socket:
        return False
    try:
        with robot.tcp_lock:
            robot.tcp_socket.sendall((cmd + "\n").encode())
        return True
    except Exception as e:
        print(f"[TCP] Send error: {e}")
        robot.tcp_connected = False
        return False

def scan_for_pico():
    """Scan network for Pico's TCP server on port 8080"""
    found = []
    print("[SCAN] Looking for Pico on network...")
    
    # Get local IP subnets
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
    
    def check_pico(ip):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.3)
        try:
            result = s.connect_ex((ip, 8080))
            if result == 0:
                print(f"[SCAN] Found Pico at {ip}")
                found.append(ip)
        except: pass
        finally:
            s.close()
    
    threads = []
    for base_ip in subnets:
        for i in list(range(100, 200)) + list(range(2, 21)):
            t = threading.Thread(target=check_pico, args=(base_ip + str(i),))
            threads.append(t)
            t.start()
    
    for t in threads:
        t.join()
    
    return found

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
    """Send command to Pico via TCP (preferred) or Serial"""
    cmd_map = {
        "WALK": "W", "BACK": "B", "STOP": "S", 
        "LEFT": "A", "RIGHT": "D", "NEUTRAL": "N", "SAVE": "SAVE",
        "STAND": "NEUTRAL", "FORWARD": "W", "BACKWARD": "B"
    }
    primitive = cmd_map.get(cmd, cmd)
    
    # Try TCP first (WiFi), then Serial (USB)
    if robot.tcp_connected:
        tcp_send_command(primitive)
    elif robot.serial_connected:
        send_serial_cmd_raw(primitive)
    else:
        print(f"[WARN] No connection to Pico, command '{primitive}' not sent")
        return None
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
    """Get full robot status including TCP connection and telemetry"""
    return {
        "serial_connected": robot.serial_connected,
        "tcp_connected": robot.tcp_connected,
        "pico_ip": robot.pico_ip,
        "ekf_state": robot.latest_ekf_state,
        "telemetry": robot.pico_telemetry
    }

@app.post("/api/connect")
def connect_serial_endpoint(port: str = ""):
    """Connect via USB Serial"""
    if not port:
        auto_connect_serial()
    return {"connected": robot.serial_connected}

@app.post("/api/connect_tcp")
def connect_tcp_endpoint(ip: str = ""):
    """Connect to Pico via TCP/WiFi"""
    if not ip:
        # Auto-scan for Pico
        picos = scan_for_pico()
        if picos:
            ip = picos[0]
        else:
            return {"connected": False, "error": "No Pico found on network"}
    
    success = tcp_connect_pico(ip, 8080)
    return {"connected": success, "ip": ip if success else ""}

@app.get("/api/scan_pico")
def scan_pico_endpoint():
    """Scan network for Pico devices"""
    picos = scan_for_pico()
    return {"picos": picos}

@app.post("/api/command")
def send_command(cmd: str):
    """Send command to robot (via TCP or Serial)"""
    primitive = process_high_level_command(cmd)
    return {"status": "sent" if primitive else "no_connection", "primitive": primitive}

@app.post("/api/servo")
def set_servo(channel: int, angle: int):
    """Set individual servo angle"""
    cmd = f"SERVO_{channel}_{angle}"
    process_high_level_command(cmd)
    return {"status": "sent", "channel": channel, "angle": angle}

@app.post("/api/calibration")
def set_calibration(leg: str, shoulder: int, elbow: int, wrist: int):
    """Set calibration for a specific leg (FR, FL, RR, RL)"""
    cmd = f"CAL_{leg}_{shoulder}_{elbow}_{wrist}"
    process_high_level_command(cmd)
    return {"status": "sent", "leg": leg}

@app.post("/api/set_home")
def set_home():
    """Set current servo positions as home/neutral"""
    process_high_level_command("SET_HOME")
    return {"status": "sent"}

@app.post("/api/save")
def save_calibration():
    """Save calibration to flash"""
    process_high_level_command("SAVE")
    return {"status": "sent"}

@app.get("/api/telemetry")
def get_telemetry():
    """Get latest telemetry from Pico"""
    return robot.pico_telemetry

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

def auto_discover_pico():
    print("\n=== AUTO-DISCOVERING PICO ===")
    picos = scan_for_pico()
    if picos:
        print(f"Found Pico at: {picos[0]}")
        tcp_connect_pico(picos[0])
    else:
        print("No Pico found on network. Will try USB serial...")
    print("==============================\n")

# Initial Auto Connect
print("\n" + "="*50)
print("SpotMicro Backend Server Starting...")
print("="*50 + "\n")

# Try USB serial first
try:
    auto_connect_serial()
except:
    pass

# Run Pico TCP discovery in background (non-blocking)
threading.Thread(target=auto_discover_pico, daemon=True).start()

# Run camera discovery in background (non-blocking)
threading.Thread(target=auto_discover_cameras, daemon=True).start()
