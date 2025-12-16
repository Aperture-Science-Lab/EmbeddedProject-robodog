import numpy as np
from rotations import Quaternion, skew_symmetric, angle_normalize

# Constants
VAR_IMU_F = 0.10      # IMU accelerometer variance (m/s²)^2
VAR_IMU_W = 0.25      # IMU gyroscope variance (rad/s)^2
VAR_GPS = 0.01        # GPS position variance (m)^2
G_VEC = np.array([0, 0, -9.81])  # gravity vector (m/s²)

class SpotEKF:
    def __init__(self, init_pos=None):
        # Initial State
        self.p = np.zeros(3) if init_pos is None else np.array(init_pos)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0]) # w, x, y, z
        
        # Covariance
        self.p_cov = np.eye(9) * 0.1
        
        # Jacobians
        self.l_jac = np.zeros([9, 6])
        self.l_jac[3:, :] = np.eye(6)
        
        self.h_jac = np.zeros([3, 9])
        self.h_jac[:, :3] = np.eye(3)

    def predict(self, imu_f, imu_w, dt):
        """
        Predict state based on IMU data.
        imu_f: Specific force [3] (m/s^2)
        imu_w: Angular velocity [3] (rad/s)
        dt: Time delta (s)
        """
        # Previous state
        p_prev = self.p
        v_prev = self.v
        q_prev = self.q
        p_cov_prev = self.p_cov
        
        # Rotation Matrix
        C_ns = Quaternion(*q_prev).to_mat()
        
        # State Prediction
        # p_check = p_prev + dt * v_prev + 0.5 * (C_ns @ imu_f + G_VEC) * (dt**2)
        # However, imu_f is usually in sensor frame. C_ns transforms sensor to nav.
        # But wait, es_ekf.py line 538: (C_ns @ imu_f_in + g)
        
        self.p = p_prev + dt * v_prev + 0.5 * (C_ns @ imu_f + G_VEC) * (dt**2)
        self.v = v_prev + (C_ns @ imu_f + G_VEC) * dt
        self.q = Quaternion(*q_prev).quat_mult_left(
             Quaternion(axis_angle=angle_normalize(imu_w * dt))
        ).to_numpy() # Store as numpy array
        
        # Linearize Motion Model (Jacobians)
        F = np.identity(9)
        F[0:3, 3:6] = np.identity(3) * dt
        F[3:6, 6:9] = -skew_symmetric(C_ns @ (imu_f.reshape(3,1))) * dt
        
        # Propagate Uncertainty
        Q = np.diag([VAR_IMU_F]*3 + [VAR_IMU_W]*3)
        self.p_cov = F @ p_cov_prev @ F.T + self.l_jac @ (dt**2 * Q) @ self.l_jac.T

    def update(self, gps_pos):
        """
        Update state based on GPS position.
        gps_pos: [x, y, z] in ENU frame (m)
        """
        y_k = np.array(gps_pos)
        
        # Kalman Gain
        R = np.eye(3) * VAR_GPS
        S = self.h_jac @ self.p_cov @ self.h_jac.T + R
        K = self.p_cov @ self.h_jac.T @ np.linalg.inv(S)
        
        # Error State
        # y_k is measurement, self.p is predicted position (first 3 elements of state)
        delta_x = K @ ((y_k - self.p).reshape(3))
        
        # Correct State
        self.p = self.p + delta_x[:3]
        self.v = self.v + delta_x[3:6]
        self.q = Quaternion(axis_angle=angle_normalize(delta_x[6:])).quat_mult_left(
            Quaternion(*self.q)
        ).to_numpy()
        
        # Correct Covariance
        self.p_cov = (np.eye(9) - K @ self.h_jac) @ self.p_cov

    def get_state(self):
        return {
            "position": self.p.tolist(),
            "velocity": self.v.tolist(),
            "orientation": self.q.tolist()
        }

# GPS Utils
def lat_lon_alt_to_enu(lat, lon, alt, lat0, lon0, alt0):
    a = 6378137.0
    e2 = 6.69437999014e-3
    lat_rad = np.deg2rad(lat); lon_rad = np.deg2rad(lon)
    lat0_rad = np.deg2rad(lat0); lon0_rad = np.deg2rad(lon0)
    
    def get_xyz(lat_r, lon_r, h):
        N = a / np.sqrt(1 - e2 * np.sin(lat_r)**2)
        x = (N + h) * np.cos(lat_r) * np.cos(lon_r)
        y = (N + h) * np.cos(lat_r) * np.sin(lon_r)
        z = (N * (1 - e2) + h) * np.sin(lat_r)
        return x, y, z
        
    x, y, z = get_xyz(lat_rad, lon_rad, alt)
    x0, y0, z0 = get_xyz(lat0_rad, lon0_rad, alt0)
    
    dx, dy, dz = x-x0, y-y0, z-z0
    
    sin_lat = np.sin(lat0_rad); cos_lat = np.cos(lat0_rad)
    sin_lon = np.sin(lon0_rad); cos_lon = np.cos(lon0_rad)
    
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    
    return np.array([east, north, up])
