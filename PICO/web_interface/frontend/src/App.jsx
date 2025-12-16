
import { useState, useEffect } from 'react'
import logo from './assets/logo.png'
import './index.css'

const API_URL = "http://localhost:8000/api";

function App() {
  const [status, setStatus] = useState({ serial_connected: false, ekf_state: {} });
  const [logs, setLogs] = useState([]);
  const [camConfig, setCamConfig] = useState({ left: "http://192.168.1.101:81/stream", right: "http://192.168.1.102:81/stream" });
  const [scanning, setScanning] = useState(false);

  // Camera Error States
  const [camErrorL, setCamErrorL] = useState(false);
  const [camErrorR, setCamErrorR] = useState(false);

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

  const sendCmd = async (cmd) => {
    addLog(`CMD >> ${cmd}`);
    try {
      await fetch(`${API_URL}/command?cmd=${cmd}`, { method: 'POST' });
    } catch (e) {
      addLog(`ERR >> ${cmd} Failed`);
    }
  };

  const connectSerial = async () => {
    addLog("SYS >> Connecting Serial...");
    try {
      await fetch(`${API_URL}/connect`, { method: 'POST' });
    } catch (e) {
      addLog("ERR >> Connection Failed");
    }
  };

  const scanCameras = async () => {
    setScanning(true);
    addLog("SYS >> Scanning Network...");
    try {
      const res = await fetch(`${API_URL}/scan_cameras`);
      const data = await res.json();
      if (data.cameras.length > 0) {
        const left = data.cameras[0];
        const right = data.cameras.length > 1 ? data.cameras[1] : camConfig.right;
        setCamConfig({ left, right });
        addLog(`SYS >> Found Cams: ${left}, ${right}`);
        await fetch(`${API_URL}/config/cameras?left=${left}&right=${right}`, { method: 'POST' });
        setCamErrorL(false); setCamErrorR(false);
      } else {
        addLog("SYS >> No Cameras Found.");
      }
    } catch (e) {
      addLog("ERR >> Scan Failed");
    }
    setScanning(false);
  }

  const manualUpdateCams = async () => {
    addLog(`SYS >> Config Cams: ${camConfig.left}, ${camConfig.right}`);
    await fetch(`${API_URL}/config/cameras?left=${camConfig.left}&right=${camConfig.right}`, { method: 'POST' });
    setCamErrorL(false); setCamErrorR(false);
    // Force reload images by appending timestamp (react trick) or just let state update do it if src changes
  }

  const addLog = (msg) => {
    setLogs(prev => [`${msg}`, ...prev].slice(0, 15));
  };

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
            <img src={logo} className="logo-img" alt="Aperture Logo" />
            <div className="title-text">
              <h1>HOT DOG V3</h1>
              <span className="version">SPOTMICRO ROBOTICS</span>
            </div>
          </div>
          <div className={`status-indicator ${status.serial_connected ? 'online' : 'offline'}`}>
            {status.serial_connected ? 'SYSTEM ONLINE' : 'DISCONNECTED'}
            {!status.serial_connected &&
              <button className="lab-btn connect-btn" onClick={connectSerial}>CONNECT</button>
            }
          </div>
        </header>

        <div className="lab-grid">

          {/* PRIMARY VISUALS */}
          <div className="lab-card vision-area">
            <h2>VISUAL FEED</h2>

            <div className="camera-feeds">
              <div className={`feed-box ${camErrorL ? 'error' : ''}`}>
                {!camErrorL ? (
                  <img src={`${API_URL}/video_feed_left`} onError={() => setCamErrorL(true)} />
                ) : <span style={{ color: 'white' }}>NO SIGNAL (LEFT)</span>}
                <div className="feed-label">CAM_L</div>
              </div>

              <div className={`feed-box ${camErrorR ? 'error' : ''}`}>
                {!camErrorR ? (
                  <img src={`${API_URL}/video_feed_right`} onError={() => setCamErrorR(true)} />
                ) : <span style={{ color: 'white' }}>NO SIGNAL (RIGHT)</span>}
                <div className="feed-label">CAM_R</div>
                {depth > 0 && <div className="depth-value">Z: {depth.toFixed(2)}m</div>}
              </div>
            </div>

            <div className="cam-config">
              <input className="lab-input" value={camConfig.left} onChange={e => setCamConfig({ ...camConfig, left: e.target.value })} placeholder="Left IP" />
              <input className="lab-input" value={camConfig.right} onChange={e => setCamConfig({ ...camConfig, right: e.target.value })} placeholder="Right IP" />
              <button className="lab-btn accent" onClick={manualUpdateCams}>SET</button>
              <button className="lab-btn scan-btn" onClick={scanCameras} disabled={scanning}>
                {scanning ? "SCANNING..." : "SCAN"}
              </button>
            </div>
          </div>

          {/* CONTROLS & DATA */}
          <div className="control-area">

            <div className="lab-card">
              <h2>MANUAL OVERRIDE</h2>
              <div className="nav-grid">
                <button></button>
                <button className="lab-btn" onMouseDown={() => sendCmd('WALK')}>FWD</button>
                <button></button>

                <button className="lab-btn" onMouseDown={() => sendCmd('LEFT')}>◀</button>
                <button className="lab-btn stop" onMouseDown={() => sendCmd('STOP')}>STOP</button>
                <button className="lab-btn" onMouseDown={() => sendCmd('RIGHT')}>▶</button>

                <button></button>
                <button className="lab-btn" onMouseDown={() => sendCmd('BACK')}>BWD</button>
                <button></button>
              </div>
              <div style={{ display: 'flex', gap: 10 }}>
                <button className="lab-btn" style={{ flex: 1 }} onClick={() => sendCmd('NEUTRAL')}>STAND</button>
                <button className="lab-btn accent" style={{ flex: 1 }} onClick={() => sendCmd('SAVE')}>SAVE</button>
              </div>
            </div>

            <div className="lab-card">
              <h2>TELEMETRY</h2>
              <div className="ekf-viz-container">
                <div className="map-view">
                  <div className="robot-arrow" style={{
                    transform: `translate(${pos[1] * 40}px, ${-pos[0] * 40}px) rotate(${arrowDeg}deg)`
                  }}></div>
                </div>
                <div className="telemetry-list">
                  <div>POS_X: {pos[0]?.toFixed(3)} m</div>
                  <div>POS_Y: {pos[1]?.toFixed(3)} m</div>
                  <div>HDG:   {(arrowDeg).toFixed(1)} deg</div>
                  <div>ACC_Z: {ekf.imu?.az?.toFixed(1)} g</div>
                </div>
              </div>
            </div>

            <div className="lab-card" style={{ flex: 1 }}>
              <h2>SYSTEM LOG</h2>
              <div className="log-terminal">
                {logs.map((l, i) => <div key={i}>{l}</div>)}
              </div>
            </div>

          </div>
        </div>
      </div>
    </div>
  )
}

export default App
