# Extended Kalman Filter for Robodog State Estimation
#
# Adapted from Coursera SDC Course 2 final project
# Original Authors: Trevor Ablett and Jonathan Kelly
# University of Toronto Institute for Aerospace Studies
#
# Modified for robodog with IMU and GPS (Neo S3 v2 GPS module - required)
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from rotations import angle_normalize, rpy_jacobian_axis_angle, skew_symmetric, Quaternion

#### 1. Data Loading ############################################################################

################################################################################################
# Data loading for robodog sensors
# Expected data format:
#   imu_data: Dictionary or object with:
#     - roll, pitch, yaw: Orientation in degrees
#     - accel_x, accel_y, accel_z: Accelerometer readings in g
#     - gyro_x, gyro_y, gyro_z: Gyroscope readings in degrees per second (dps)
#     - t: Timestamps in ms
#   gps_data: Required StampedData object with GPS position data (Neo S3 v2 GPS)
#     - data: Position data [N, 3] in meters
#     - t: Timestamps in ms
#   gt: Optional ground truth data for comparison
################################################################################################

################################################################################################
# GPS Data Loading for Neo S3 v2 GPS Module
# 
# The Neo S3 v2 GPS module outputs NMEA format sentences. GPS data is REQUIRED.
# 
# Option 1: Load from NMEA file (Recommended for Neo S3 v2)
#   gps = load_gps_from_nmea('neo_s3_v2_gps.nmea', 
#                           reference_position=[40.7128, -74.0060, 0.0])
#
# Option 2: Load from CSV file (if you've converted NMEA to CSV)
#   gps = load_gps_from_csv('neo_s3_v2_gps.csv', 
#                          reference_position=[40.7128, -74.0060, 0.0])
#
# Option 3: Convert from parsed GPS readings
#   gps_readings = [
#       {'latitude': 40.7128, 'longitude': -74.0060, 'altitude': 10.0, 't': 0},
#       {'latitude': 40.7129, 'longitude': -74.0061, 'altitude': 10.5, 't': 1000},
#       # ... more readings from Neo S3 v2 GPS
#   ]
#   gps = convert_gps_to_ekf_format(gps_readings, 
#                                   reference_position=[40.7128, -74.0060, 0.0])
#
# Option 4: Load from pickle (for testing with existing data)
#   import pickle
#   with open('data/pt1_data.pkl', 'rb') as file:
#       data = pickle.load(file)
#   gps = data.get('gnss', None)  # If already in ENU format
################################################################################################

# Example data loading - adapt to your actual data source
# This assumes data is provided as a dictionary or loaded from a file
# Replace this section with your actual data loading code

# For now, we'll assume data is provided in a compatible format
# You may need to adapt this based on your actual data source
try:
    # Try to load from pickle if available (for testing)
    import pickle
    with open('data/pt1_data.pkl', 'rb') as file:
        data = pickle.load(file)
    gt = data.get('gt', None)
    imu_f = data.get('imu_f', None)
    imu_w = data.get('imu_w', None)
    gps = data.get('gnss', None)  # Assumes already in ENU format
except:
    # If pickle file doesn't exist, expect data to be provided directly
    # You must provide: imu_data and gps (required - Neo S3 v2 GPS)
    gt = None
    imu_f = None
    imu_w = None
    gps = None

# Example: Load GPS from Neo S3 v2 GPS module (uncomment and adapt as needed)
# Option 1: Load from NMEA file (Neo S3 v2 outputs NMEA format)
# gps = load_gps_from_nmea('neo_s3_v2_gps.nmea', 
#                          reference_position=[40.7128, -74.0060, 0.0])
#
# Option 2: Load from CSV file
# gps = load_gps_from_csv('neo_s3_v2_gps.csv',
#                        reference_position=[40.7128, -74.0060, 0.0])
#
# Option 3: Convert from list of GPS readings (if you parse NMEA yourself)
# gps_readings = [
#     {'latitude': 40.7128, 'longitude': -74.0060, 'altitude': 10.0, 't': 0},
#     {'latitude': 40.7129, 'longitude': -74.0061, 'altitude': 10.5, 't': 1000},
#     # ... more readings from Neo S3 v2 GPS
# ]
# gps = convert_gps_to_ekf_format(gps_readings,
#                                 reference_position=[40.7128, -74.0060, 0.0])

################################################################################################
# GPS Data Conversion Functions
################################################################################################

def lat_lon_alt_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """
    Convert GPS coordinates (latitude, longitude, altitude) to local ENU (East-North-Up) coordinates.
    
    Args:
        lat: Latitude in degrees
        lon: Longitude in degrees
        alt: Altitude in meters
        lat0: Reference latitude in degrees (origin of local frame)
        lon0: Reference longitude in degrees (origin of local frame)
        alt0: Reference altitude in meters (origin of local frame)
        
    Returns:
        [east, north, up]: Position in meters in ENU frame
    """
    # WGS84 ellipsoid parameters
    a = 6378137.0  # Semi-major axis (m)
    e2 = 6.69437999014e-3  # First eccentricity squared
    
    # Convert to radians
    lat_rad = np.deg2rad(lat)
    lon_rad = np.deg2rad(lon)
    lat0_rad = np.deg2rad(lat0)
    lon0_rad = np.deg2rad(lon0)
    
    # Radius of curvature in prime vertical
    N = a / np.sqrt(1 - e2 * np.sin(lat_rad)**2)
    N0 = a / np.sqrt(1 - e2 * np.sin(lat0_rad)**2)
    
    # ECEF coordinates
    x = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
    y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
    z = (N * (1 - e2) + alt) * np.sin(lat_rad)
    
    x0 = (N0 + alt0) * np.cos(lat0_rad) * np.cos(lon0_rad)
    y0 = (N0 + alt0) * np.cos(lat0_rad) * np.sin(lon0_rad)
    z0 = (N0 * (1 - e2) + alt0) * np.sin(lat0_rad)
    
    # Relative position
    dx = x - x0
    dy = y - y0
    dz = z - z0
    
    # Rotation matrix from ECEF to ENU
    sin_lat = np.sin(lat0_rad)
    cos_lat = np.cos(lat0_rad)
    sin_lon = np.sin(lon0_rad)
    cos_lon = np.cos(lon0_rad)
    
    # ENU = [East, North, Up]
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    
    return np.array([east, north, up])

def convert_gps_to_ekf_format(gps_data_list, reference_position=None):
    """
    Convert GPS data (lat/lon/alt) to EKF-compatible format (ENU coordinates).
    
    The EKF works in a local ENU (East-North-Up) coordinate frame. This function
    converts GPS coordinates (latitude, longitude, altitude) to this local frame.
    The reference_position defines the origin of the local frame - typically the
    starting position of the robodog.
    
    Args:
        gps_data_list: List of GPS readings, each with:
            - latitude: Latitude in degrees
            - longitude: Longitude in degrees
            - altitude: Altitude in meters
            - t: Timestamp in ms
        reference_position: Optional [lat0, lon0, alt0] for ENU origin.
                            If None, uses first GPS reading as reference.
                            This should be your starting position or a fixed
                            reference point near your operating area.
                            
    Returns:
        gps: StampedData-like object with position in ENU frame [N, 3] in meters
             Format: [[east, north, up], ...] where:
             - east: meters east of reference
             - north: meters north of reference  
             - up: meters above reference altitude
    """
    class StampedData:
        def __init__(self, data, t):
            self.data = np.array(data)
            self.t = np.array(t)
    
    if len(gps_data_list) == 0:
        return None
    
    # Use first GPS reading as reference if not provided
    if reference_position is None:
        lat0 = gps_data_list[0]['latitude']
        lon0 = gps_data_list[0]['longitude']
        alt0 = gps_data_list[0]['altitude']
    else:
        lat0, lon0, alt0 = reference_position
    
    gps_positions = []
    timestamps = []
    
    for gps_reading in gps_data_list:
        lat = gps_reading['latitude']
        lon = gps_reading['longitude']
        alt = gps_reading['altitude']
        
        # Convert to ENU coordinates
        enu_pos = lat_lon_alt_to_enu(lat, lon, alt, lat0, lon0, alt0)
        gps_positions.append(enu_pos)
        timestamps.append(gps_reading['t'])
    
    return StampedData(gps_positions, timestamps)

def load_gps_from_nmea(nmea_file_path, reference_position=None):
    """
    Load GPS data from NMEA file (e.g., .nmea, .txt with NMEA sentences).
    
    Args:
        nmea_file_path: Path to NMEA file
        reference_position: Optional [lat0, lon0, alt0] for ENU origin
        
    Returns:
        gps: StampedData-like object with position in ENU frame
    """
    import re
    from datetime import datetime
    
    gps_readings = []
    
    # Regex patterns for NMEA sentences
    gga_pattern = re.compile(r'\$GPGGA,(\d+\.?\d*),(\d+\.?\d*),([NS]),(\d+\.?\d*),([EW]),(\d+),(\d+),([\d.]+),([\d.]+),M')
    
    with open(nmea_file_path, 'r') as f:
        for line in f:
            if line.startswith('$GPGGA'):
                # Parse GGA sentence
                match = gga_pattern.match(line)
                if match:
                    time_str = match.group(1)
                    lat_deg = float(match.group(2))
                    lat_dir = match.group(3)
                    lon_deg = float(match.group(4))
                    lon_dir = match.group(5)
                    quality = int(match.group(6))
                    alt = float(match.group(9))
                    
                    # Convert to decimal degrees
                    lat = lat_deg if lat_dir == 'N' else -lat_deg
                    lon = lon_deg if lon_dir == 'E' else -lon_deg
                    
                    # Only use valid GPS fixes (quality > 0)
                    if quality > 0:
                        # Convert time to timestamp (simplified - you may need to adjust)
                        # Assuming time is in HHMMSS.SSS format
                        hours = int(time_str[:2])
                        minutes = int(time_str[2:4])
                        seconds = float(time_str[4:])
                        timestamp_ms = int((hours * 3600 + minutes * 60 + seconds) * 1000)
                        
                        gps_readings.append({
                            'latitude': lat,
                            'longitude': lon,
                            'altitude': alt,
                            't': timestamp_ms
                        })
    
    return convert_gps_to_ekf_format(gps_readings, reference_position)

def load_gps_from_csv(csv_file_path, reference_position=None, 
                     lat_col='latitude', lon_col='longitude', alt_col='altitude', 
                     time_col='timestamp_ms'):
    """
    Load GPS data from CSV file.
    
    Args:
        csv_file_path: Path to CSV file
        reference_position: Optional [lat0, lon0, alt0] for ENU origin
        lat_col, lon_col, alt_col, time_col: Column names in CSV
        
    Returns:
        gps: StampedData-like object with position in ENU frame
    """
    import csv
    
    gps_readings = []
    
    with open(csv_file_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                gps_readings.append({
                    'latitude': float(row[lat_col]),
                    'longitude': float(row[lon_col]),
                    'altitude': float(row[alt_col]),
                    't': int(float(row[time_col]))
                })
            except (KeyError, ValueError) as e:
                print(f"Warning: Skipping invalid GPS row: {e}")
                continue
    
    return convert_gps_to_ekf_format(gps_readings, reference_position)

# Convert robodog IMU data format to EKF format
# Robodog IMU provides: roll/pitch/yaw (degrees), accel (g), gyro (dps)
# EKF needs: specific force (m/s²), angular velocity (rad/s)

def convert_robodog_imu_to_ekf_format(imu_data_list):
    """
    Convert robodog IMU data format to EKF-compatible format.
    
    Args:
        imu_data_list: List of IMU readings, each with:
            - roll, pitch, yaw: Orientation in degrees
            - accel_x, accel_y, accel_z: Accelerometer in g
            - gyro_x, gyro_y, gyro_z: Gyroscope in dps
            - t: Timestamp in ms
            
    Returns:
        imu_f: StampedData-like object with specific force (m/s²)
        imu_w: StampedData-like object with angular velocity (rad/s)
    """
    class StampedData:
        def __init__(self, data, t):
            self.data = np.array(data)
            self.t = np.array(t)
    
    imu_f_data = []
    imu_w_data = []
    timestamps = []
    
    g_vec = np.array([0, 0, -9.81])  # gravity in navigation frame
    
    for imu_reading in imu_data_list:
        # Convert orientation (degrees -> radians)
        roll_rad = np.deg2rad(imu_reading['roll'])
        pitch_rad = np.deg2rad(imu_reading['pitch'])
        yaw_rad = np.deg2rad(imu_reading['yaw'])
        
        # Convert accelerometer (g -> m/s²)
        accel_g = np.array([imu_reading['accel_x'], 
                           imu_reading['accel_y'], 
                           imu_reading['accel_z']])
        accel_ms2 = accel_g * 9.81
        
        # Convert gyroscope (dps -> rad/s)
        gyro_dps = np.array([imu_reading['gyro_x'], 
                            imu_reading['gyro_y'], 
                            imu_reading['gyro_z']])
        gyro_rads = np.deg2rad(gyro_dps)
        
        # Convert to quaternion and rotation matrix
        q = Quaternion(euler=[roll_rad, pitch_rad, yaw_rad])
        C_ns = q.to_mat()  # Rotation from sensor/vehicle frame to navigation frame
        
        # Compute specific force (remove gravity component)
        # The accelerometer measures: a = f + C_ns^T @ g
        # So specific force in vehicle frame: f = a - C_ns^T @ g
        # But since accel is already in vehicle frame, we need to account for gravity
        # In vehicle frame, gravity appears as: C_ns^T @ g
        specific_force = accel_ms2 - C_ns.T @ g_vec
        
        imu_f_data.append(specific_force)
        imu_w_data.append(gyro_rads)
        timestamps.append(imu_reading['t'])
    
    return StampedData(imu_f_data, timestamps), StampedData(imu_w_data, timestamps)

# If data is provided in robodog format, convert it
# This assumes you have a list of IMU readings in robodog format
# Uncomment and adapt the following if your data is in robodog format:
# if imu_f is None or imu_w is None:
#     # Assuming robodog_imu_data is a list of dictionaries with IMU readings
#     imu_f, imu_w = convert_robodog_imu_to_ekf_format(robodog_imu_data)

# GPS is required (Neo S3 v2 GPS module)
if gps is None:
    raise ValueError("GPS data is required but not provided. Please provide GPS data from Neo S3 v2 GPS module.")
elif not hasattr(gps, 'data') or not hasattr(gps, 't'):
    raise ValueError("GPS data must have 'data' and 't' attributes. Use convert_gps_to_ekf_format() or load_gps_from_nmea() to format GPS data.")
elif len(gps.data) == 0:
    raise ValueError("GPS data is empty. Please provide valid GPS readings from Neo S3 v2 GPS module.")
else:
    print(f"GPS data loaded: {len(gps.data)} readings from Neo S3 v2 GPS module")

################################################################################################
# Plot ground truth trajectory if available
################################################################################################
if gt is not None:
    gt_fig = plt.figure()
    ax = gt_fig.add_subplot(111, projection='3d')
    ax.plot(gt.p[:,0], gt.p[:,1], gt.p[:,2])
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.set_title('Ground Truth Trajectory')
    plt.show()

################################################################################################

#### 2. Constants ##############################################################################

################################################################################################
# Sensor variances - tune these for your robodog sensors
################################################################################################

var_imu_f = 0.10      # IMU accelerometer variance (m/s²)²
var_imu_w = 0.25      # IMU gyroscope variance (rad/s)²
var_gps = 0.01        # GPS position variance (m)²

################################################################################################
# Constants for EKF
################################################################################################
g = np.array([0, 0, -9.81])  # gravity vector (m/s²)
l_jac = np.zeros([9, 6])
l_jac[3:, :] = np.eye(6)  # motion model noise jacobian
h_jac = np.zeros([3, 9])
h_jac[:, :3] = np.eye(3)  # measurement model jacobian for GPS

#### 3. Initial Values #########################################################################

################################################################################################
# Initialize state estimates
################################################################################################

# Determine number of timesteps from IMU data
if imu_f is not None and hasattr(imu_f, 'data') and len(imu_f.data) > 0:
    num_timesteps = imu_f.data.shape[0]
elif imu_w is not None and hasattr(imu_w, 'data') and len(imu_w.data) > 0:
    num_timesteps = imu_w.data.shape[0]
else:
    raise ValueError("IMU data not available. Cannot initialize EKF. Please provide IMU data in the expected format.")

p_est = np.zeros([num_timesteps, 3])  # position estimates
v_est = np.zeros([num_timesteps, 3])  # velocity estimates
q_est = np.zeros([num_timesteps, 4])  # orientation estimates as quaternions
p_cov = np.zeros([num_timesteps, 9, 9])  # covariance matrices at each timestep

# Set initial values
if gt is not None:
    p_est[0] = gt.p[0]
    v_est[0] = gt.v[0]
    q_est[0] = Quaternion(euler=gt.r[0]).to_numpy()
elif gps is not None and len(gps.data) > 0:
    # Initialize position from first GPS reading (Neo S3 v2 GPS)
    p_est[0] = gps.data[0]
    v_est[0] = np.array([0.0, 0.0, 0.0])  # Assume stationary at start
    q_est[0] = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion (level orientation)
    print(f"Initialized position from GPS: {p_est[0]}")
else:
    # Initialize with zeros (should not happen if GPS validation passed)
    p_est[0] = np.array([0.0, 0.0, 0.0])
    v_est[0] = np.array([0.0, 0.0, 0.0])
    q_est[0] = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion

p_cov[0] = np.eye(9) * 0.1  # Initial covariance (small uncertainty)
gps_i = 0

#### 4. Measurement Update Functions ##########################################################

################################################################################################
# GPS measurement update (position)
################################################################################################
def measurement_update(sensor_var, p_cov_check, y_k, p_check, v_check, q_check):
    """
    GPS position measurement update.
    
    Args:
        sensor_var: Measurement variance
        p_cov_check: Predicted covariance
        y_k: GPS position measurement [3]
        p_check: Predicted position [3]
        v_check: Predicted velocity [3]
        q_check: Predicted orientation quaternion [4]
        
    Returns:
        Updated state and covariance
    """
    # Compute Kalman Gain
    R = np.eye(3) * sensor_var
    S = h_jac @ p_cov_check @ h_jac.T + R
    S += np.eye(3) * 1e-8  # Numerical stability
    K = p_cov_check @ h_jac.T @ np.linalg.inv(S)

    # Compute error state
    delta_x = K @ ((y_k - p_check).reshape(3))

    # Correct predicted state
    p_hat = p_check + delta_x[:3]
    v_hat = v_check + delta_x[3:6]
    q_hat = Quaternion(axis_angle=angle_normalize(delta_x[6:])).quat_mult_left(q_check)

    # Compute corrected covariance
    p_cov_hat = (np.eye(9) - K @ h_jac) @ p_cov_check

    return p_hat, v_hat, q_hat, p_cov_hat

#### 5. Main Filter Loop #######################################################################

################################################################################################
# EKF main loop
################################################################################################

for k in range(1, num_timesteps):
    # Time delta (convert ms to seconds)
    if hasattr(imu_f, 't'):
        delta_t = (imu_f.t[k] - imu_f.t[k - 1]) / 1000.0  # Convert ms to s
    elif hasattr(imu_w, 't'):
        delta_t = (imu_w.t[k] - imu_w.t[k - 1]) / 1000.0
    else:
        delta_t = 0.01  # Default 100 Hz
    
    # Previous state
    p_prev = p_est[k-1]
    v_prev = v_est[k-1]
    q_prev = q_est[k-1]
    p_cov_prev = p_cov[k-1]

    # Get IMU measurements
    if imu_f is not None and hasattr(imu_f, 'data') and k-1 < len(imu_f.data):
        imu_f_in = imu_f.data[k-1]
    else:
        imu_f_in = np.array([0.0, 0.0, 0.0])
        print(f"Warning: IMU specific force data not available at timestep {k}")
    
    if imu_w is not None and hasattr(imu_w, 'data') and k-1 < len(imu_w.data):
        imu_w_in = imu_w.data[k-1]
    else:
        imu_w_in = np.array([0.0, 0.0, 0.0])
        print(f"Warning: IMU angular velocity data not available at timestep {k}")
    
    # 1. Predict state with IMU inputs
    C_ns = Quaternion(*q_prev).to_mat()

    # State prediction
    p_check = p_prev + delta_t * v_prev + 0.5 * (C_ns @ imu_f_in + g) * (delta_t**2)
    v_check = v_prev + (C_ns @ imu_f_in + g) * delta_t
    q_check = Quaternion(*q_prev).quat_mult_left(
        Quaternion(axis_angle=angle_normalize(imu_w_in * delta_t))
    )

    # 1.1 Linearize the motion model and compute Jacobians
    F = np.identity(9)
    F[0:3, 3:6] = np.identity(3) * delta_t
    F[3:6, 6:9] = -skew_symmetric(C_ns @ (imu_f_in.reshape(3,1))) * delta_t
    
    # 2. Propagate uncertainty
    Q = np.diag([var_imu_f, var_imu_f, var_imu_f, var_imu_w, var_imu_w, var_imu_w])
    p_cov_check = F @ p_cov_prev @ F.T + l_jac @ (delta_t**2 * Q) @ l_jac.T

    # 3. GPS update (required - Neo S3 v2 GPS)
    # Get current time from IMU data
    if imu_f is not None and hasattr(imu_f, 't'):
        current_time = imu_f.t[k]
    elif imu_w is not None and hasattr(imu_w, 't'):
        current_time = imu_w.t[k]
    else:
        current_time = k * 10  # Default: assume 100Hz (10ms intervals)
    
    # Apply GPS update when timestamp matches
    if gps_i < gps.data.shape[0]:
        if current_time >= gps.t[gps_i]:
            p_check, v_check, q_check, p_cov_check = measurement_update(
                var_gps, p_cov_check, gps.data[gps_i], p_check, v_check, q_check
            )
            gps_i += 1
    elif gps_i >= gps.data.shape[0] and k < num_timesteps - 1:
        # Warn if we've run out of GPS data but still have IMU data
        if k == num_timesteps - 1 or (k % 100 == 0):  # Only warn occasionally
            print(f"Warning: GPS data exhausted at timestep {k}, continuing with IMU-only prediction")

    # Update states (save)
    p_est[k] = p_check
    v_est[k] = v_check
    q_est[k] = q_check
    p_cov[k] = p_cov_check

#### 6. Results and Analysis ###################################################################

################################################################################################
# Plot estimated trajectory
################################################################################################
est_traj_fig = plt.figure()
ax = est_traj_fig.add_subplot(111, projection='3d')
ax.plot(p_est[:,0], p_est[:,1], p_est[:,2], label='Estimated', linewidth=2)
if gt is not None:
    ax.plot(gt.p[:,0], gt.p[:,1], gt.p[:,2], label='Ground Truth', linewidth=2)
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.set_title('Robodog Estimated Trajectory')
ax.legend()
ax.grid(True)
plt.show()

################################################################################################
# Error plots (if ground truth available)
################################################################################################
if gt is not None:
    error_fig, ax = plt.subplots(2, 3)
    error_fig.suptitle('State Estimation Error')
    num_gt = min(gt.p.shape[0], p_est.shape[0])
    p_est_euler = []
    p_cov_euler_std = []

    # Convert estimated quaternions to euler angles
    for i in range(len(q_est)):
        qc = Quaternion(*q_est[i, :])
        p_est_euler.append(qc.to_euler())

        # First-order approximation of RPY covariance
        J = rpy_jacobian_axis_angle(qc.to_axis_angle())
        p_cov_euler_std.append(np.sqrt(np.diagonal(J @ p_cov[i, 6:, 6:] @ J.T)))

    p_est_euler = np.array(p_est_euler)
    p_cov_euler_std = np.array(p_cov_euler_std)

    # Get uncertainty estimates from P matrix
    p_cov_std = np.sqrt(np.diagonal(p_cov[:, :6, :6], axis1=1, axis2=2))

    titles = ['X Position', 'Y Position', 'Z Position', 'Roll', 'Pitch', 'Yaw']
    for i in range(3):
        ax[0, i].plot(range(num_gt), gt.p[:num_gt, i] - p_est[:num_gt, i], label='Error')
        ax[0, i].plot(range(num_gt),  3 * p_cov_std[:num_gt, i], 'r--', label='±3σ')
        ax[0, i].plot(range(num_gt), -3 * p_cov_std[:num_gt, i], 'r--')
        ax[0, i].set_title(titles[i])
        ax[0, i].grid(True)
        ax[0, i].legend()
    ax[0,0].set_ylabel('Error [m]')

    for i in range(3):
        ax[1, i].plot(range(num_gt), 
            angle_normalize(gt.r[:num_gt, i] - p_est_euler[:num_gt, i]), label='Error')
        ax[1, i].plot(range(num_gt),  3 * p_cov_euler_std[:num_gt, i], 'r--', label='±3σ')
        ax[1, i].plot(range(num_gt), -3 * p_cov_euler_std[:num_gt, i], 'r--')
        ax[1, i].set_title(titles[i+3])
        ax[1, i].grid(True)
        ax[1, i].legend()
    ax[1,0].set_ylabel('Error [rad]')
    plt.show()

print("EKF processing complete.")
