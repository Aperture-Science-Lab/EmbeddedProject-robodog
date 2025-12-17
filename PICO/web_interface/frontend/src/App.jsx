
import { useCallback, useEffect, useState } from 'react';
import './App.css';
import logo from './assets/logo.png';
import './index.css';

const API_URL = "http://localhost:8000/api";

// Icon Components
const IconWifi = () => (
  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <path d="M5 12.55a11 11 0 0 1 14.08 0" /><path d="M1.42 9a16 16 0 0 1 21.16 0" /><path d="M8.53 16.11a6 6 0 0 1 6.95 0" /><circle cx="12" cy="20" r="1" />
  </svg>
);

const IconCamera = () => (
  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <path d="M23 19a2 2 0 0 1-2 2H3a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2h4l2-3h6l2 3h4a2 2 0 0 1 2 2z" /><circle cx="12" cy="13" r="4" />
  </svg>
);

const IconSettings = () => (
  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <circle cx="12" cy="12" r="3" /><path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1 0 2.83 2 2 0 0 1-2.83 0l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-2 2 2 2 0 0 1-2-2v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83 0 2 2 0 0 1 0-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1-2-2 2 2 0 0 1 2-2h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 0-2.83 2 2 0 0 1 2.83 0l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 2-2 2 2 0 0 1 2 2v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 0 2 2 0 0 1 0 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 2 2 2 2 0 0 1-2 2h-.09a1.65 1.65 0 0 0-1.51 1z" />
  </svg>
);

const IconTerminal = () => (
  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <polyline points="4 17 10 11 4 5" /><line x1="12" y1="19" x2="20" y2="19" />
  </svg>
);

const IconNavigation = () => (
  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <polygon points="3 11 22 2 13 21 11 13 3 11" />
  </svg>
);

const IconActivity = () => (
  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <polyline points="22 12 18 12 15 21 9 3 6 12 2 12" />
  </svg>
);

// No Signal Animation Component
const NoSignalAnimation = () => (
  <div className="no-signal-container">
    <div className="static-noise"></div>
    <div className="no-signal-overlay">
      <div className="glitch-icon">
        <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
          <path d="M23 19a2 2 0 0 1-2 2H3a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2h4l2-3h6l2 3h4a2 2 0 0 1 2 2z" />
          <circle cx="12" cy="13" r="4" />
          <line x1="4" y1="4" x2="20" y2="20" strokeWidth="2" />
        </svg>
      </div>
      <div className="no-signal-text">NO SIGNAL</div>
      <div className="signal-bars">
        <span></span><span></span><span></span><span></span><span></span>
      </div>
      <div className="reconnect-hint">Attempting to reconnect...</div>
    </div>
    <div className="scanlines"></div>
  </div>
);

function App() {
  const [status, setStatus] = useState({ serial_connected: false, ekf_state: {} });
  const [logs, setLogs] = useState([]);
  const [camConfig, setCamConfig] = useState({ left: "http://192.168.1.101:81/stream", right: "http://192.168.1.102:81/stream" });
  const [scanning, setScanning] = useState(false);
  const [connecting, setConnecting] = useState(false);

  // Camera Error States - Default to true to show no-signal animation until cameras connect
  const [camErrorL, setCamErrorL] = useState(true);
  const [camErrorR, setCamErrorR] = useState(true);

  useEffect(() => {
    const interval = setInterval(fetchStatus, 500);
    return () => clearInterval(interval);
  }, []);

  const fetchStatus = async () => {
    try {
      const res = await fetch(`${API_URL}/status`);
      const data = await res.json();
      setStatus(data);
    } catch (e) { }
  };

  const addLog = useCallback((msg, type = 'info') => {
    const timestamp = new Date().toLocaleTimeString('en-US', { hour12: false });
    setLogs(prev => [{ msg, type, timestamp }, ...prev].slice(0, 20));
  }, []);

  const sendCmd = async (cmd) => {
    addLog(`CMD → ${cmd}`, 'command');
    try {
      await fetch(`${API_URL}/command?cmd=${cmd}`, { method: 'POST' });
    } catch (e) {
      addLog(`Failed to send: ${cmd}`, 'error');
    }
  };

  const connectSerial = async () => {
    setConnecting(true);
    addLog("Establishing serial connection...", 'system');
    try {
      await fetch(`${API_URL}/connect`, { method: 'POST' });
      addLog("Connection established", 'success');
    } catch (e) {
      addLog("Connection failed", 'error');
    }
    setConnecting(false);
  };

  const scanCameras = async () => {
    setScanning(true);
    addLog("Scanning network for cameras...", 'system');
    try {
      const res = await fetch(`${API_URL}/scan_cameras`);
      const data = await res.json();
      if (data.cameras.length > 0) {
        const left = data.cameras[0];
        const right = data.cameras.length > 1 ? data.cameras[1] : camConfig.right;
        setCamConfig({ left, right });
        addLog(`Found cameras: ${data.cameras.length}`, 'success');
        await fetch(`${API_URL}/config/cameras?left=${left}&right=${right}`, { method: 'POST' });
        setCamErrorL(false); setCamErrorR(false);
      } else {
        addLog("No cameras found on network", 'warning');
      }
    } catch (e) {
      addLog("Camera scan failed", 'error');
    }
    setScanning(false);
  }

  const manualUpdateCams = async () => {
    addLog(`Updating camera config...`, 'system');
    await fetch(`${API_URL}/config/cameras?left=${camConfig.left}&right=${camConfig.right}`, { method: 'POST' });
    setCamErrorL(false); setCamErrorR(false);
    addLog(`Camera config updated`, 'success');
  }

  // EKF Viz Helpers
  const ekf = status.ekf_state || {};
  const pos = ekf.position || [0, 0, 0];
  const rot = ekf.orientation || [1, 0, 0, 0];
  const depth = ekf.depth || 0.0;

  const q0 = rot[0], q1 = rot[1], q2 = rot[2], q3 = rot[3];
  const yaw = Math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
  const arrowDeg = -yaw * 180 / Math.PI;

  return (
    <div className="app-container">
      <div className="clean-panel">

        {/* HEADER */}
        <header className="lab-header">
          <div className="brand-block">
            <img src={logo} className="logo-img" alt="Robot Logo" />
            <div className="title-text">
              <h1>HOT DOG V3</h1>
              <span className="version">SpotMicro Robotics</span>
            </div>
          </div>
          <div className={`status-indicator ${status.serial_connected ? 'online' : 'offline'}`}>
            {status.serial_connected ? 'SYSTEM ONLINE' : 'DISCONNECTED'}
            {!status.serial_connected &&
              <button 
                className="lab-btn connect-btn" 
                onClick={connectSerial}
                disabled={connecting}
              >
                {connecting ? 'CONNECTING...' : 'CONNECT'}
              </button>
            }
          </div>
        </header>

        <div className="lab-grid">

          {/* PRIMARY VISUALS */}
          <div className="lab-card vision-area">
            <h2><IconCamera /> Visual Feed</h2>

            <div className="camera-feeds">
              <div className={`feed-box ${camErrorL ? 'error' : ''}`}>
                {!camErrorL ? (
                  <img 
                    src={`${API_URL}/video_feed_left`} 
                    onError={() => setCamErrorL(true)} 
                    alt="Left Camera Feed"
                  />
                ) : (
                  <NoSignalAnimation />
                )}
                <div className="feed-label">CAM_L</div>
              </div>

              <div className={`feed-box ${camErrorR ? 'error' : ''}`}>
                {!camErrorR ? (
                  <img 
                    src={`${API_URL}/video_feed_right`} 
                    onError={() => setCamErrorR(true)} 
                    alt="Right Camera Feed"
                  />
                ) : (
                  <NoSignalAnimation />
                )}
                <div className="feed-label">CAM_R</div>
                {depth > 0 && <div className="depth-value">Z: {depth.toFixed(2)}m</div>}
              </div>
            </div>

            <div className="cam-config">
              <input 
                className="lab-input" 
                value={camConfig.left} 
                onChange={e => setCamConfig({ ...camConfig, left: e.target.value })} 
                placeholder="Left Camera IP" 
              />
              <input 
                className="lab-input" 
                value={camConfig.right} 
                onChange={e => setCamConfig({ ...camConfig, right: e.target.value })} 
                placeholder="Right Camera IP" 
              />
              <button className="lab-btn accent" onClick={manualUpdateCams}>
                <IconSettings /> SET
              </button>
              <button className="lab-btn scan-btn" onClick={scanCameras} disabled={scanning}>
                <IconWifi /> {scanning ? "SCANNING..." : "SCAN"}
              </button>
            </div>
          </div>

          {/* CONTROLS & DATA */}
          <div className="control-area">

            <div className="lab-card">
              <h2><IconNavigation /> Manual Override</h2>
              <div className="nav-grid">
                <button className="lab-btn" disabled></button>
                <button className="lab-btn" onMouseDown={() => sendCmd('WALK')}>↑ FWD</button>
                <button className="lab-btn" disabled></button>

                <button className="lab-btn" onMouseDown={() => sendCmd('LEFT')}>← LEFT</button>
                <button className="lab-btn stop" onMouseDown={() => sendCmd('STOP')}>⬤ STOP</button>
                <button className="lab-btn" onMouseDown={() => sendCmd('RIGHT')}>RIGHT →</button>

                <button className="lab-btn" disabled></button>
                <button className="lab-btn" onMouseDown={() => sendCmd('BACK')}>↓ BWD</button>
                <button className="lab-btn" disabled></button>
              </div>
              <div style={{ display: 'flex', gap: 10 }}>
                <button className="lab-btn" style={{ flex: 1 }} onClick={() => sendCmd('NEUTRAL')}>
                  STAND
                </button>
                <button className="lab-btn accent" style={{ flex: 1 }} onClick={() => sendCmd('SAVE')}>
                  SAVE
                </button>
              </div>
            </div>

            <div className="lab-card">
              <h2><IconActivity /> Telemetry</h2>
              <div className="ekf-viz-container">
                <div className="map-view">
                  <div className="robot-arrow" style={{
                    transform: `translate(${pos[1] * 40}px, ${-pos[0] * 40}px) rotate(${arrowDeg}deg)`
                  }}></div>
                </div>
                <div className="telemetry-list">
                  <div>POS_X: {pos[0]?.toFixed(3)} m</div>
                  <div>POS_Y: {pos[1]?.toFixed(3)} m</div>
                  <div>HDG: {(arrowDeg).toFixed(1)}°</div>
                  <div>ACC_Z: {ekf.imu?.az?.toFixed(2) || '0.00'} g</div>
                </div>
              </div>
            </div>

            <div className="lab-card" style={{ flex: 1 }}>
              <h2><IconTerminal /> System Log</h2>
              <div className="log-terminal">
                {logs.length === 0 ? (
                  <div className="log-empty">Waiting for events...</div>
                ) : (
                  logs.map((l, i) => (
                    <div key={i} className={`log-entry log-${l.type}`}>
                      <span className="log-time">[{l.timestamp}]</span> {l.msg}
                    </div>
                  ))
                )}
              </div>
            </div>

          </div>
        </div>
      </div>
    </div>
  )
}

export default App
