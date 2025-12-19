
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
  const [status, setStatus] = useState({ 
    serial_connected: false, 
    tcp_connected: false,
    pico_ip: "",
    ekf_state: {},
    telemetry: {}
  });
  const [logs, setLogs] = useState([]);
  const [camConfig, setCamConfig] = useState({ left: "http://192.168.1.101:81/stream", right: "http://192.168.1.102:81/stream" });
  const [scanning, setScanning] = useState(false);
  const [scanningPico, setScanningPico] = useState(false);
  const [connecting, setConnecting] = useState(false);
  const [picoIp, setPicoIp] = useState("");
  const [activeTab, setActiveTab] = useState("control");

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
    setLogs(prev => [{ msg, type, timestamp }, ...prev].slice(0, 30));
  }, []);

  const sendCmd = async (cmd) => {
    addLog(`CMD → ${cmd}`, 'command');
    try {
      await fetch(`${API_URL}/command?cmd=${cmd}`, { method: 'POST' });
    } catch (e) {
      addLog(`Failed: ${cmd}`, 'error');
    }
  };

  const connectSerial = async () => {
    setConnecting(true);
    addLog("Connecting USB Serial...", 'system');
    try {
      const res = await fetch(`${API_URL}/connect`, { method: 'POST' });
      const data = await res.json();
      addLog(data.connected ? "USB connected" : "USB failed", data.connected ? 'success' : 'error');
    } catch (e) {
      addLog("USB failed", 'error');
    }
    setConnecting(false);
  };

  const connectTcp = async () => {
    setConnecting(true);
    addLog(picoIp ? `Connecting to ${picoIp}...` : "Scanning for Pico...", 'system');
    try {
      const res = await fetch(`${API_URL}/connect_tcp?ip=${picoIp}`, { method: 'POST' });
      const data = await res.json();
      if (data.connected) {
        addLog(`WiFi connected: ${data.ip}`, 'success');
        setPicoIp(data.ip);
      } else {
        addLog(data.error || "TCP failed", 'error');
      }
    } catch (e) {
      addLog("TCP failed", 'error');
    }
    setConnecting(false);
  };

  const scanPico = async () => {
    setScanningPico(true);
    addLog("Scanning for Pico...", 'system');
    try {
      const res = await fetch(`${API_URL}/scan_pico`);
      const data = await res.json();
      if (data.picos?.length > 0) {
        setPicoIp(data.picos[0]);
        addLog(`Found: ${data.picos.join(", ")}`, 'success');
      } else {
        addLog("No Pico found", 'warning');
      }
    } catch (e) {
      addLog("Scan failed", 'error');
    }
    setScanningPico(false);
  };

  const scanCameras = async () => {
    setScanning(true);
    addLog("Scanning for cameras...", 'system');
    try {
      const res = await fetch(`${API_URL}/scan_cameras`);
      const data = await res.json();
      if (data.cameras.length > 0) {
        setCamConfig({ left: data.cameras[0], right: data.cameras[1] || camConfig.right });
        addLog(`Found ${data.cameras.length} camera(s)`, 'success');
        await fetch(`${API_URL}/config/cameras?left=${data.cameras[0]}&right=${data.cameras[1] || camConfig.right}`, { method: 'POST' });
        setCamErrorL(false); setCamErrorR(false);
      } else {
        addLog("No cameras found", 'warning');
      }
    } catch (e) {
      addLog("Scan failed", 'error');
    }
    setScanning(false);
  };

  const manualUpdateCams = async () => {
    await fetch(`${API_URL}/config/cameras?left=${camConfig.left}&right=${camConfig.right}`, { method: 'POST' });
    setCamErrorL(false); setCamErrorR(false);
    addLog("Cameras updated", 'success');
  };

  const telem = status.telemetry || {};
  const imu = telem.imu || {};
  const ir = telem.ir || {};
  const pir = telem.pir || {};
  const ldr = telem.ldr || {};
  const gps = telem.gps || {};
  const cal = telem.calibration || {};
  const head = telem.head || {};
  const ekf = status.ekf_state || {};
  const depth = ekf.depth || 0.0;
  const isConnected = status.serial_connected || status.tcp_connected;

  return (
    <div className="app-container">
      <div className="clean-panel">
        <header className="lab-header">
          <div className="brand-block">
            <img src={logo} className="logo-img" alt="Logo" />
            <div className="title-text">
              <h1>SPOT MICRO</h1>
              <span className="version">FreeRTOS Control</span>
            </div>
          </div>
          <div className={`status-indicator ${isConnected ? 'online' : 'offline'}`}>
            {isConnected ? (status.tcp_connected ? `WiFi: ${status.pico_ip}` : 'USB') + ` | ${telem.state || 'OK'}` : 'DISCONNECTED'}
          </div>
          <div className="connection-controls">
            <input className="lab-input" value={picoIp} onChange={e => setPicoIp(e.target.value)} placeholder="Pico IP" style={{width:'120px'}}/>
            <button className="lab-btn" onClick={scanPico} disabled={scanningPico}><IconWifi/>{scanningPico?'...':'SCAN'}</button>
            <button className="lab-btn accent" onClick={connectTcp} disabled={connecting}>WiFi</button>
            <button className="lab-btn" onClick={connectSerial} disabled={connecting}>USB</button>
          </div>
        </header>

        <div className="tab-nav">
          <button className={activeTab==='control'?'active':''} onClick={()=>setActiveTab('control')}><IconNavigation/> Control</button>
          <button className={activeTab==='sensors'?'active':''} onClick={()=>setActiveTab('sensors')}><IconActivity/> Sensors</button>
          <button className={activeTab==='calibration'?'active':''} onClick={()=>setActiveTab('calibration')}><IconSettings/> Calibration</button>
        </div>

        <div className="lab-grid">
          <div className="lab-card vision-area">
            <h2><IconCamera /> Cameras</h2>
            <div className="camera-feeds">
              <div className={`feed-box ${camErrorL?'error':''}`}>
                {!camErrorL ? <img class="scaleY(-1)" src={`${API_URL}/video_feed_left`} onError={()=>setCamErrorL(true)} alt="L"/> : <NoSignalAnimation/>}
                <div className="feed-label">LEFT</div>
              </div>
              <div className={`feed-box ${camErrorR?'error':''}`}>
                {!camErrorR ? <img src={`${API_URL}/video_feed_right`} onError={()=>setCamErrorR(true)} alt="R"/> : <NoSignalAnimation/>}
                <div className="feed-label">RIGHT</div>
                {depth > 0 && <div className="depth-value">Z:{depth.toFixed(2)}m</div>}
              </div>
            </div>
            <div className="cam-config">
              <input className="lab-input" value={camConfig.left} onChange={e=>setCamConfig({...camConfig,left:e.target.value})} placeholder="Left IP"/>
              <input className="lab-input" value={camConfig.right} onChange={e=>setCamConfig({...camConfig,right:e.target.value})} placeholder="Right IP"/>
              <button className="lab-btn accent" onClick={manualUpdateCams}><IconSettings/>SET</button>
              <button className="lab-btn" onClick={scanCameras} disabled={scanning}><IconWifi/>{scanning?'...':'SCAN'}</button>
            </div>
          </div>

          <div className="control-area">
            {activeTab==='control' && <>
              <div className="lab-card">
                <h2><IconNavigation /> Movement</h2>
                <div className="nav-grid">
                  <button className="lab-btn" disabled></button>
                  <button className="lab-btn" onClick={()=>sendCmd('WALK')}>↑ FWD</button>
                  <button className="lab-btn" disabled></button>
                  <button className="lab-btn" onClick={()=>sendCmd('LEFT')}>← LEFT</button>
                  <button className="lab-btn stop" onClick={()=>sendCmd('STOP')}>⬤ STOP</button>
                  <button className="lab-btn" onClick={()=>sendCmd('RIGHT')}>RIGHT →</button>
                  <button className="lab-btn" disabled></button>
                  <button className="lab-btn" onClick={()=>sendCmd('BACK')}>↓ BWD</button>
                  <button className="lab-btn" disabled></button>
                </div>
                <div style={{display:'flex',gap:10,marginTop:10}}>
                  <button className="lab-btn" style={{flex:1}} onClick={()=>sendCmd('NEUTRAL')}>STAND</button>
                  <button className="lab-btn danger" style={{flex:1}} onClick={()=>sendCmd('ESTOP')}>E-STOP</button>
                </div>
              </div>
              <div className="lab-card">
                <h2>🐕 Actions</h2>
                <div style={{display:'flex',gap:8,flexWrap:'wrap'}}>
                  <button className="lab-btn accent" onClick={()=>sendCmd('TAIL_WAG')}>🐕 Wag Tail</button>
                  <button className="lab-btn" onClick={()=>sendCmd('HEAD_ON')}>👀 Scan</button>
                  <button className="lab-btn" onClick={()=>sendCmd('HEAD_OFF')}>Stop Scan</button>
                  <button className="lab-btn" onClick={()=>sendCmd('HEAD_LEFT')}>Look L</button>
                  <button className="lab-btn" onClick={()=>sendCmd('HEAD_CENTER')}>Center</button>
                  <button className="lab-btn" onClick={()=>sendCmd('HEAD_RIGHT')}>Look R</button>
                </div>
              </div>
              <div className="lab-card">
                <h2>Status</h2>
                <div className="telemetry-grid">
                  <div className="telem-item"><span>State</span><span>{telem.state||'N/A'}</span></div>
                  <div className="telem-item"><span>Head</span><span>{head.angle?.toFixed(0)||90}° {head.swing?'🔄':''}</span></div>
                  <div className="telem-item"><span>WiFi</span><span className={telem.wifi?'ok':'error'}>{telem.wifi?'✓':'✗'}</span></div>
                  <div className="telem-item"><span>Nano</span><span className={telem.nano?'ok':'error'}>{telem.nano?'✓':'✗'}</span></div>
                </div>
              </div>
            </>}

            {activeTab==='sensors' && <>
              <div className="lab-card">
                <h2>IMU</h2>
                <div className="sensor-grid">
                  <div className="sensor-item"><span>Roll</span><span>{imu.roll?.toFixed(1)||0}°</span></div>
                  <div className="sensor-item"><span>Pitch</span><span>{imu.pitch?.toFixed(1)||0}°</span></div>
                  <div className="sensor-item"><span>Yaw</span><span>{imu.yaw?.toFixed(1)||0}°</span></div>
                </div>
              </div>
              <div className="lab-card">
                <h2>🚨 Obstacles</h2>
                <div className="sensor-grid">
                  <div className={`sensor-item ${ir.front?'alert':''}`}><span>IR Front</span><span>{ir.front?'⚠️BLOCKED':'✓'}</span></div>
                  <div className={`sensor-item ${ir.back?'alert':''}`}><span>IR Back</span><span>{ir.back?'⚠️BLOCKED':'✓'}</span></div>
                  <div className={`sensor-item ${pir.front?'alert':''}`}><span>PIR Front</span><span>{pir.front?'👤MOTION':'—'}</span></div>
                  <div className={`sensor-item ${pir.back?'alert':''}`}><span>PIR Back</span><span>{pir.back?'👤MOTION':'—'}</span></div>
                </div>
              </div>
              <div className="lab-card">
                <h2>Environment</h2>
                <div className="sensor-grid">
                  <div className="sensor-item"><span>Light</span><span>{ldr.percent?.toFixed(0)||0}%</span></div>
                  <div className="sensor-item"><span>US Left</span><span>{telem.us?.left?.toFixed(0)||0}cm</span></div>
                  <div className="sensor-item"><span>US Right</span><span>{telem.us?.right?.toFixed(0)||0}cm</span></div>
                </div>
              </div>
              <div className="lab-card">
                <h2>📍 GPS</h2>
                <div className="sensor-grid">
                  <div className="sensor-item"><span>Fix</span><span className={gps.fix?'ok':'error'}>{gps.fix?'✓':'✗'}</span></div>
                  <div className="sensor-item"><span>Sats</span><span>{gps.sats||0}</span></div>
                  <div className="sensor-item"><span>Lat</span><span>{gps.lat?.toFixed(5)||0}</span></div>
                  <div className="sensor-item"><span>Lon</span><span>{gps.lon?.toFixed(5)||0}</span></div>
                </div>
              </div>
            </>}

            {activeTab==='calibration' && <>
              <div className="lab-card">
                <h2><IconSettings /> Calibration</h2>
                <p style={{fontSize:'0.8em',opacity:0.7}}>Adjust robot, then SET HOME + SAVE</p>
                <div style={{display:'flex',gap:10,marginTop:10}}>
                  <button className="lab-btn accent" onClick={()=>sendCmd('SET_HOME')}>📐 SET HOME</button>
                  <button className="lab-btn success" onClick={()=>sendCmd('SAVE')}>💾 SAVE</button>
                  <button className="lab-btn" onClick={()=>sendCmd('NEUTRAL')}>Reset</button>
                </div>
              </div>
              <div className="lab-card">
                <h2>Current Values</h2>
                <div className="calibration-display">
                  <div><b>FR:</b> S:{cal.shoulder?.[0]||60} E:{cal.elbow?.[0]||90} W:{cal.wrist?.[0]||50}</div>
                  <div><b>FL:</b> S:{cal.shoulder?.[1]||130} E:{cal.elbow?.[1]||90} W:{cal.wrist?.[1]||130}</div>
                  <div><b>RR:</b> S:{cal.shoulder?.[2]||120} E:{cal.elbow?.[2]||90} W:{cal.wrist?.[2]||50}</div>
                  <div><b>RL:</b> S:{cal.shoulder?.[3]||50} E:{cal.elbow?.[3]||90} W:{cal.wrist?.[3]||130}</div>
                </div>
              </div>
              <div className="lab-card">
                <h2>Servo Sliders</h2>
                <div className="servo-control">
                  {[...Array(13)].map((_,i)=>(
                    <div key={i} className="servo-slider">
                      <label>CH{i}</label>
                      <input type="range" min="0" max="180" defaultValue="90" onChange={e=>sendCmd(`SERVO_${i}_${e.target.value}`)}/>
                    </div>
                  ))}
                </div>
              </div>
            </>}

            <div className="lab-card log-card">
              <h2><IconTerminal /> Log</h2>
              <div className="log-terminal">
                {logs.length===0 ? <div className="log-empty">Waiting...</div> : logs.map((l,i)=>(
                  <div key={i} className={`log-entry log-${l.type}`}><span className="log-time">[{l.timestamp}]</span> {l.msg}</div>
                ))}
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}

export default App
