/**
 * @file spot_kinematics_v2.cpp
 * @brief Improved Inverse Kinematics Implementation for SpotMicro
 * 
 * Based on: "Implementing Kinematics of a four-legged Robot"
 * https://github.com/mike4192/spotMicro (reference)
 */

#include "spot_kinematics_v2.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ============================================================================
// Calibrated Neutral Angles - EXTERN references to arrays in main_controller.cpp
// These are loaded from flash or set to defaults on boot by main_controller
// ============================================================================
extern int calibrated_shoulder[4];   // Defined in main_controller.cpp
extern int calibrated_elbow[4];      // Defined in main_controller.cpp  
extern int calibrated_wrist[4];      // Defined in main_controller.cpp

// ============================================================================
// Utility Functions
// ============================================================================

float deg2rad(float deg) {
    return deg * M_PI / 180.0f;
}

float rad2deg(float rad) {
    return rad * 180.0f / M_PI;
}

float clampf(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

float smoothstep(float t) {
    t = clampf(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

void vec3_lerp(const Vec3* a, const Vec3* b, float t, Vec3* result) {
    float st = smoothstep(t);
    result->x = lerp(a->x, b->x, st);
    result->y = lerp(a->y, b->y, st);
    result->z = lerp(a->z, b->z, st);
}

// ============================================================================
// Matrix Operations
// ============================================================================

void mat4_identity(Mat4* m) {
    memset(m, 0, sizeof(Mat4));
    m->m[0][0] = 1.0f;
    m->m[1][1] = 1.0f;
    m->m[2][2] = 1.0f;
    m->m[3][3] = 1.0f;
}

void mat4_multiply(const Mat4* a, const Mat4* b, Mat4* result) {
    Mat4 temp;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            temp.m[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                temp.m[i][j] += a->m[i][k] * b->m[k][j];
            }
        }
    }
    memcpy(result, &temp, sizeof(Mat4));
}

void mat4_transform_point(const Mat4* m, const Vec3* p, Vec3* result) {
    float w = m->m[3][0] * p->x + m->m[3][1] * p->y + m->m[3][2] * p->z + m->m[3][3];
    result->x = (m->m[0][0] * p->x + m->m[0][1] * p->y + m->m[0][2] * p->z + m->m[0][3]) / w;
    result->y = (m->m[1][0] * p->x + m->m[1][1] * p->y + m->m[1][2] * p->z + m->m[1][3]) / w;
    result->z = (m->m[2][0] * p->x + m->m[2][1] * p->y + m->m[2][2] * p->z + m->m[2][3]) / w;
}

// Simple 4x4 matrix inverse (assumes affine transformation)
void mat4_inverse(const Mat4* m, Mat4* result) {
    // For affine matrices: inverse of rotation is transpose, inverse of translation is -R^T * t
    Mat4 inv;
    mat4_identity(&inv);
    
    // Transpose the 3x3 rotation part
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            inv.m[i][j] = m->m[j][i];
        }
    }
    
    // Compute -R^T * t for translation
    inv.m[0][3] = -(inv.m[0][0] * m->m[0][3] + inv.m[0][1] * m->m[1][3] + inv.m[0][2] * m->m[2][3]);
    inv.m[1][3] = -(inv.m[1][0] * m->m[0][3] + inv.m[1][1] * m->m[1][3] + inv.m[1][2] * m->m[2][3]);
    inv.m[2][3] = -(inv.m[2][0] * m->m[0][3] + inv.m[2][1] * m->m[1][3] + inv.m[2][2] * m->m[2][3]);
    
    memcpy(result, &inv, sizeof(Mat4));
}

// ============================================================================
// Leg Inverse Kinematics
// Based on: Sen, Muhammed Arif & Bakircioglu, Veli & Kalyoncu, Mete. (2017)
// "Inverse Kinematic Analysis Of A Quadruped Robot"
// ============================================================================

bool leg_ik(float x4, float y4, float z4, LegAngles* angles, bool is_right_side) {
    /**
     * Leg IK calculates q1, q2, q3 for foot position (x4, y4, z4)
     * relative to leg origin (hip joint).
     * 
     * Using 3-link leg model:
     *   l1 = hip link (coxa)
     *   l2 = upper leg (femur) 
     *   l3 = lower leg (tibia)
     */
    
    float l1 = SPOT_L1;  // Hip/coxa length
    float l2 = SPOT_L3;  // Upper leg (using L3 as femur)
    float l3 = SPOT_L4;  // Lower leg (using L4 as tibia)
    
    // Supporting variable D for knee angle calculation
    float D = (x4*x4 + y4*y4 + z4*z4 - l1*l1 - l2*l2 - l3*l3) / (2.0f * l2 * l3);
    D = clampf(D, -0.999f, 0.999f);  // Clamp to avoid domain errors
    
    // q3 (knee/wrist angle) - different for left vs right side legs
    // Right side (legs 1,2): positive sqrt
    // Left side (legs 3,4): negative sqrt
    float sqrt_term = sqrtf(1.0f - D*D);
    if (is_right_side) {
        angles->theta3 = atan2f(sqrt_term, D);
    } else {
        angles->theta3 = atan2f(-sqrt_term, D);
    }
    
    // q2 (elbow/femur angle)
    float xy_dist_sq = x4*x4 + y4*y4 - l1*l1;
    if (xy_dist_sq < 0.01f) xy_dist_sq = 0.01f;  // Prevent negative sqrt
    float xy_term = sqrtf(xy_dist_sq);
    
    angles->theta2 = atan2f(z4, xy_term) - atan2f(l3 * sinf(angles->theta3), l2 + l3 * cosf(angles->theta3));
    
    // q1 (shoulder/hip angle)
    // Note: Using corrected formula from paper analysis
    float sqrt_xy = sqrtf(x4*x4 + y4*y4 - l1*l1);
    if (isnan(sqrt_xy)) sqrt_xy = 0.01f;
    angles->theta1 = atan2f(y4, x4) + atan2f(sqrt_xy, -l1);
    
    return true;
}

// Wrapper for backward compatibility
bool leg_ik_simple(float x, float y, float z, LegAngles* angles) {
    return leg_ik(x, y, z, angles, true);
}

// ============================================================================
// Body Inverse Kinematics
// Based on the article's bodyIK function
// ============================================================================

void body_ik(const BodyPose* pose, Mat4* Tlf, Mat4* Trf, Mat4* Tlb, Mat4* Trb) {
    float omega = pose->omega;  // Roll
    float phi = pose->phi;      // Pitch
    float psi = pose->psi;      // Yaw
    float xm = pose->x;
    float ym = pose->y;
    float zm = pose->z;
    
    float L = SPOT_BODY_LENGTH;
    float W = SPOT_BODY_WIDTH;
    
    // Rotation matrices
    float co = cosf(omega), so = sinf(omega);
    float cp = cosf(phi), sp = sinf(phi);
    float cy = cosf(psi), sy = sinf(psi);
    
    // Combined rotation matrix Rxyz = Rx * Ry * Rz
    Mat4 Rxyz;
    mat4_identity(&Rxyz);
    
    // Row 0
    Rxyz.m[0][0] = cp * cy;
    Rxyz.m[0][1] = -cp * sy;
    Rxyz.m[0][2] = sp;
    
    // Row 1
    Rxyz.m[1][0] = so * sp * cy + co * sy;
    Rxyz.m[1][1] = -so * sp * sy + co * cy;
    Rxyz.m[1][2] = -so * cp;
    
    // Row 2
    Rxyz.m[2][0] = -co * sp * cy + so * sy;
    Rxyz.m[2][1] = co * sp * sy + so * cy;
    Rxyz.m[2][2] = co * cp;
    
    // Add translation
    Rxyz.m[0][3] = xm;
    Rxyz.m[1][3] = ym;
    Rxyz.m[2][3] = zm;
    
    // Pre-computed sin/cos for 90 degree rotation
    float sHp = 1.0f;   // sin(pi/2)
    float cHp = 0.0f;   // cos(pi/2)
    
    // Leg offset matrices (position of each hip relative to body center)
    // Left Front: (+L/2, +W/2)
    Mat4 Mlf;
    mat4_identity(&Mlf);
    Mlf.m[0][0] = cHp;  Mlf.m[0][2] = sHp;  Mlf.m[0][3] = L / 2.0f;
    Mlf.m[2][0] = -sHp; Mlf.m[2][2] = cHp;  Mlf.m[2][3] = W / 2.0f;
    
    // Right Front: (+L/2, -W/2)
    Mat4 Mrf;
    mat4_identity(&Mrf);
    Mrf.m[0][0] = cHp;  Mrf.m[0][2] = sHp;  Mrf.m[0][3] = L / 2.0f;
    Mrf.m[2][0] = -sHp; Mrf.m[2][2] = cHp;  Mrf.m[2][3] = -W / 2.0f;
    
    // Left Back: (-L/2, +W/2)
    Mat4 Mlb;
    mat4_identity(&Mlb);
    Mlb.m[0][0] = cHp;  Mlb.m[0][2] = sHp;  Mlb.m[0][3] = -L / 2.0f;
    Mlb.m[2][0] = -sHp; Mlb.m[2][2] = cHp;  Mlb.m[2][3] = W / 2.0f;
    
    // Right Back: (-L/2, -W/2)
    Mat4 Mrb;
    mat4_identity(&Mrb);
    Mrb.m[0][0] = cHp;  Mrb.m[0][2] = sHp;  Mrb.m[0][3] = -L / 2.0f;
    Mrb.m[2][0] = -sHp; Mrb.m[2][2] = cHp;  Mrb.m[2][3] = -W / 2.0f;
    
    // Compute final transformation matrices
    mat4_multiply(&Rxyz, &Mlf, Tlf);
    mat4_multiply(&Rxyz, &Mrf, Trf);
    mat4_multiply(&Rxyz, &Mlb, Tlb);
    mat4_multiply(&Rxyz, &Mrb, Trb);
}

// ============================================================================
// Convert Joint Angles to Servo Angles
// ============================================================================

void joint_to_servo(LegIndex leg, const LegAngles* joint_angles, ServoAngles* servo_angles) {
    // Convert from radians to degrees
    float t2_deg = rad2deg(joint_angles->theta2);
    float t3_deg = rad2deg(joint_angles->theta3);
    
    // =========================================================================
    // Uses CALIBRATED neutral angles from main_controller.cpp
    // These are loaded from flash or set to defaults on boot
    // Shoulder angles are LOCKED to calibrated neutral
    // Elbow and wrist move relative to their calibrated neutral
    // =========================================================================
    
    // Movement multipliers - increase for more visible motion
    const float elbow_multiplier = 1.5f;
    const float wrist_multiplier = 3.0f;  // Larger for visible wrist movement
    
    // Get calibrated neutral angles for this leg
    float shoulder_neutral = (float)calibrated_shoulder[leg];
    float elbow_neutral = (float)calibrated_elbow[leg];
    float wrist_neutral = (float)calibrated_wrist[leg];
    
    // Determine direction based on leg side
    // Right side (FR=0, RR=2): positive elbow, negative wrist
    // Left side (FL=1, RL=3): negative elbow, positive wrist
    bool is_right = (leg == LEG_FR || leg == LEG_RR);
    float dir = is_right ? 1.0f : -1.0f;
    float wrist_dir = is_right ? -1.0f : 1.0f;
    
    // Apply calibrated neutral + IK offsets
    servo_angles->shoulder = shoulder_neutral;  // LOCKED at calibrated value
    servo_angles->elbow = elbow_neutral + dir * t2_deg * elbow_multiplier;
    servo_angles->wrist = wrist_neutral + wrist_dir * (t2_deg * 0.5f + t3_deg * wrist_multiplier);
    
    // Clamp to valid servo range
    servo_angles->shoulder = clampf(servo_angles->shoulder, 10.0f, 170.0f);
    servo_angles->elbow = clampf(servo_angles->elbow, 15.0f, 165.0f);
    servo_angles->wrist = clampf(servo_angles->wrist, 10.0f, 170.0f);
}

// ============================================================================
// Complete Robot IK
// ============================================================================

void robot_ik(const Vec3 foot_positions[4], const BodyPose* body_pose, RobotState* state) {
    // Get body transformation matrices for each leg
    Mat4 Tlf, Trf, Tlb, Trb;
    body_ik(body_pose, &Tlf, &Trf, &Tlb, &Trb);
    
    // Store body pose
    state->body_pose = *body_pose;
    
    // Map from our leg order (FR, FL, RR, RL) to the IK matrices
    Mat4* transforms[4] = {&Trf, &Tlf, &Trb, &Tlb};  // FR, FL, RR, RL
    
    for (int leg = 0; leg < 4; leg++) {
        state->foot_positions[leg] = foot_positions[leg];
        
        // Get inverse of leg transformation
        Mat4 T_inv;
        mat4_inverse(transforms[leg], &T_inv);
        
        // Transform foot position to leg-local space
        Vec3 local_foot;
        mat4_transform_point(&T_inv, &foot_positions[leg], &local_foot);
        
        // Right side legs: FR and RR
        // Left side legs: FL and RL
        bool is_right = (leg == LEG_FR || leg == LEG_RR);
        
        // Calculate joint angles using leg IK with correct side
        leg_ik(local_foot.x, local_foot.y, local_foot.z, &state->joint_angles[leg], is_right);
        
        // Convert to servo angles
        joint_to_servo((LegIndex)leg, &state->joint_angles[leg], &state->servo_angles[leg]);
    }
}

// ============================================================================
// Gait Functions
// ============================================================================

void gait_init_walk(GaitParams* params) {
    params->step_length = 40.0f;     // 40mm forward per step
    params->step_height = 30.0f;     // 30mm foot lift
    params->body_height = 100.0f;    // 100mm standing height
    params->cycle_time_ms = 2000;    // 2 second cycle
    params->duty_factor = 0.75f;     // 75% of time on ground
    
    // Walk gait: each leg has different phase (FR, FL, RR, RL order)
    params->phase_offset[LEG_FR] = 0;
    params->phase_offset[LEG_FL] = 50;
    params->phase_offset[LEG_RR] = 75;
    params->phase_offset[LEG_RL] = 25;
}

void gait_init_trot(GaitParams* params) {
    params->step_length = 50.0f;
    params->step_height = 35.0f;
    params->body_height = 100.0f;
    params->cycle_time_ms = 800;     // Faster for trot
    params->duty_factor = 0.5f;      // 50% duty for trot
    
    // Trot gait: diagonal legs move together (FR, FL, RR, RL order)
    params->phase_offset[LEG_FR] = 0;
    params->phase_offset[LEG_FL] = 50;
    params->phase_offset[LEG_RR] = 50;
    params->phase_offset[LEG_RL] = 0;
}

void gait_update(RobotState* state, const GaitParams* params, float global_phase, bool forward) {
    float L = SPOT_BODY_LENGTH;
    float W = SPOT_BODY_WIDTH;
    float swing_duty = 1.0f - params->duty_factor;
    
    for (int leg = 0; leg < 4; leg++) {
        // Calculate this leg's phase
        float leg_phase = global_phase + (float)params->phase_offset[leg] / 100.0f;
        while (leg_phase >= 1.0f) leg_phase -= 1.0f;
        while (leg_phase < 0.0f) leg_phase += 1.0f;
        
        // Determine if swing or stance
        bool is_swing;
        float t;
        
        if (leg_phase < swing_duty) {
            is_swing = true;
            t = leg_phase / swing_duty;
        } else {
            is_swing = false;
            t = (leg_phase - swing_duty) / params->duty_factor;
        }
        
        // Calculate foot position
        Vec3 foot;
        
        // Base position (leg's default standing position)
        // Using FR, FL, RR, RL order (indices 0, 1, 2, 3)
        bool is_left = (leg == LEG_FL || leg == LEG_RL);
        bool is_front = (leg == LEG_FR || leg == LEG_FL);
        
        foot.x = is_front ? (L / 2.0f) : (-L / 2.0f);
        foot.y = is_left ? (W / 2.0f + 30.0f) : (-(W / 2.0f + 30.0f));
        foot.z = -params->body_height;
        
        // Add gait motion
        float step_dir = forward ? 1.0f : -1.0f;
        
        if (is_swing) {
            // Swing phase: foot in air, moving forward
            float swing_t = smoothstep(t);
            foot.x += step_dir * lerp(-params->step_length / 2.0f, 
                                       params->step_length / 2.0f, swing_t);
            // Parabolic height trajectory
            float height_t = 4.0f * t * (1.0f - t);
            foot.z += params->step_height * height_t;
        } else {
            // Stance phase: foot on ground, pushing back
            float stance_t = smoothstep(t);
            foot.x += step_dir * lerp(params->step_length / 2.0f, 
                                      -params->step_length / 2.0f, stance_t);
        }
        
        state->foot_positions[leg] = foot;
    }
    
    // Calculate IK for all legs
    robot_ik(state->foot_positions, &state->body_pose, state);
}

// ============================================================================
// State Management
// ============================================================================

void robot_state_init(RobotState* state, float body_height) {
    memset(state, 0, sizeof(RobotState));
    
    state->body_pose.z = body_height;
    
    // Set default standing foot positions
    robot_stand(state, body_height);
}

void robot_stand(RobotState* state, float body_height) {
    float L = SPOT_BODY_LENGTH;
    float W = SPOT_BODY_WIDTH;
    
    state->body_pose.omega = 0;
    state->body_pose.phi = 0;
    state->body_pose.psi = 0;
    state->body_pose.x = 0;
    state->body_pose.y = 0;
    state->body_pose.z = body_height;
    
    // Set foot positions relative to body center
    // Foot Y offset = half body width + leg reach
    float foot_y_offset = W / 2.0f + SPOT_L1;  // Use actual leg offset
    
    // Front Right (FR = index 0)
    state->foot_positions[LEG_FR].x = L / 2.0f;
    state->foot_positions[LEG_FR].y = -foot_y_offset;
    state->foot_positions[LEG_FR].z = -body_height;
    
    // Front Left (FL = index 1)
    state->foot_positions[LEG_FL].x = L / 2.0f;
    state->foot_positions[LEG_FL].y = foot_y_offset;
    state->foot_positions[LEG_FL].z = -body_height;
    
    // Rear Right (RR = index 2)
    state->foot_positions[LEG_RR].x = -L / 2.0f;
    state->foot_positions[LEG_RR].y = -foot_y_offset;
    state->foot_positions[LEG_RR].z = -body_height;
    
    // Rear Left (RL = index 3)
    state->foot_positions[LEG_RL].x = -L / 2.0f;
    state->foot_positions[LEG_RL].y = foot_y_offset;
    state->foot_positions[LEG_RL].z = -body_height;
    
    // For standing, set servos directly to neutral without IK
    // This ensures the robot stands at the calibrated neutral position
    state->servo_angles[LEG_FR].shoulder = NEUTRAL_SHOULDER_FR;
    state->servo_angles[LEG_FR].elbow = NEUTRAL_ELBOW_FR;
    state->servo_angles[LEG_FR].wrist = NEUTRAL_WRIST_FR;
    
    state->servo_angles[LEG_FL].shoulder = NEUTRAL_SHOULDER_FL;
    state->servo_angles[LEG_FL].elbow = NEUTRAL_ELBOW_FL;
    state->servo_angles[LEG_FL].wrist = NEUTRAL_WRIST_FL;
    
    state->servo_angles[LEG_RR].shoulder = NEUTRAL_SHOULDER_RR;
    state->servo_angles[LEG_RR].elbow = NEUTRAL_ELBOW_RR;
    state->servo_angles[LEG_RR].wrist = NEUTRAL_WRIST_RR;
    
    state->servo_angles[LEG_RL].shoulder = NEUTRAL_SHOULDER_RL;
    state->servo_angles[LEG_RL].elbow = NEUTRAL_ELBOW_RL;
    state->servo_angles[LEG_RL].wrist = NEUTRAL_WRIST_RL;
}

// ============================================================================
// ============================================================================
// Turn Function - Yaw-Based Rotation (Python SpotModel Method)
// ============================================================================
/**
 * @brief Rotate robot in place using yaw angle
 * 
 * METHOD: Similar to Python SpotModel.IK() with yaw rotation
 * 1. Set body yaw angle (psi) to rotate the body frame
 * 2. For each leg, calculate foot position in body frame
 * 3. During swing phase: rotate foot around body center (Z-axis)
 * 4. Create arc motion by rotating stance->swing transition
 * 
 * KEY CONCEPT: Each foot traces an arc around the robot's center.
 * The yaw angle rotates the body frame, and foot positions are
 * calculated relative to this rotated frame.
 * 
 * DIAGONAL GAIT: FR+RL move together, FL+RR move together
 * This creates stable tripod-like support during turning.
 */
void gait_turn(RobotState* state, float angle_deg, const GaitParams* params, float phase) {
    float L = SPOT_BODY_LENGTH;
    float W = SPOT_BODY_WIDTH;
    float swing_duty = 1.0f - params->duty_factor;
    
    // Calculate yaw rotation angle per step (scaled by step length)
    // Larger angles = tighter turn, smaller angles = wider arc
    float yaw_per_step = deg2rad(angle_deg * 0.15f);  // 15% of desired angle per cycle
    
    // Set incremental body yaw for smooth turning
    state->body_pose.psi = yaw_per_step * phase;
    
    for (int leg = 0; leg < 4; leg++) {
        // Calculate leg phase with offset
        float leg_phase = phase + (float)params->phase_offset[leg] / 100.0f;
        while (leg_phase >= 1.0f) leg_phase -= 1.0f;
        while (leg_phase < 0.0f) leg_phase += 1.0f;
        
        // Determine swing vs stance
        bool is_swing;
        float t;
        if (leg_phase < swing_duty) {
            is_swing = true;
            t = leg_phase / swing_duty;
        } else {
            is_swing = false;
            t = (leg_phase - swing_duty) / params->duty_factor;
        }
        
        // Get neutral foot position in body frame
        bool is_left = (leg == LEG_FL || leg == LEG_RL);
        bool is_front = (leg == LEG_FR || leg == LEG_FL);
        float foot_y_offset = W / 2.0f + SPOT_L1;
        
        Vec3 foot_neutral;
        foot_neutral.x = is_front ? (L / 2.0f) : (-L / 2.0f);
        foot_neutral.y = is_left ? foot_y_offset : -foot_y_offset;
        foot_neutral.z = -params->body_height;
        
        // ====================================================================
        // YAW ROTATION METHOD (from Python SpotModel)
        // ====================================================================
        // Calculate rotation angle for this leg at this phase
        // Positive angle_deg = turn left (CCW), negative = turn right (CW)
        float rotation_radius = sqrtf(foot_neutral.x * foot_neutral.x + 
                                      foot_neutral.y * foot_neutral.y);
        float base_angle = atan2f(foot_neutral.y, foot_neutral.x);
        
        Vec3 foot;
        
        if (is_swing) {
            // SWING PHASE: Rotate foot from back position to front position
            float swing_t = smoothstep(t);
            
            // Swing arc: rotate from -yaw to +yaw
            float swing_yaw = lerp(-yaw_per_step, yaw_per_step, swing_t);
            float rotated_angle = base_angle + swing_yaw;
            
            // Apply rotation: [x', y'] = [r*cos(θ), r*sin(θ)]
            foot.x = rotation_radius * cosf(rotated_angle);
            foot.y = rotation_radius * sinf(rotated_angle);
            
            // Lift foot during swing
            float height_t = 4.0f * t * (1.0f - t);  // Parabolic arc
            foot.z = foot_neutral.z + params->step_height * height_t;
            
        } else {
            // STANCE PHASE: Rotate foot from front to back (pushes robot)
            float stance_t = smoothstep(t);
            
            // Stance arc: rotate from +yaw back to -yaw
            float stance_yaw = lerp(yaw_per_step, -yaw_per_step, stance_t);
            float rotated_angle = base_angle + stance_yaw;
            
            // Apply rotation
            foot.x = rotation_radius * cosf(rotated_angle);
            foot.y = rotation_radius * sinf(rotated_angle);
            foot.z = foot_neutral.z;  // Stay on ground
        }
        
        state->foot_positions[leg] = foot;
    }
    
    // Calculate IK with rotated foot positions
    robot_ik(state->foot_positions, &state->body_pose, state);
}

// ============================================================================
// Interpolation Functions
// ============================================================================

void servo_angles_lerp(const ServoAngles* from, const ServoAngles* to, float t, ServoAngles* result) {
    float st = smoothstep(t);
    result->shoulder = lerp(from->shoulder, to->shoulder, st);
    result->elbow = lerp(from->elbow, to->elbow, st);
    result->wrist = lerp(from->wrist, to->wrist, st);
}

void robot_state_lerp(const RobotState* from, const RobotState* to, float t, RobotState* result) {
    float st = smoothstep(t);
    
    // Interpolate body pose
    result->body_pose.omega = lerp(from->body_pose.omega, to->body_pose.omega, st);
    result->body_pose.phi = lerp(from->body_pose.phi, to->body_pose.phi, st);
    result->body_pose.psi = lerp(from->body_pose.psi, to->body_pose.psi, st);
    result->body_pose.x = lerp(from->body_pose.x, to->body_pose.x, st);
    result->body_pose.y = lerp(from->body_pose.y, to->body_pose.y, st);
    result->body_pose.z = lerp(from->body_pose.z, to->body_pose.z, st);
    
    // Interpolate each leg
    for (int leg = 0; leg < 4; leg++) {
        vec3_lerp(&from->foot_positions[leg], &to->foot_positions[leg], t, &result->foot_positions[leg]);
        servo_angles_lerp(&from->servo_angles[leg], &to->servo_angles[leg], t, &result->servo_angles[leg]);
        
        result->joint_angles[leg].theta1 = lerp(from->joint_angles[leg].theta1, to->joint_angles[leg].theta1, st);
        result->joint_angles[leg].theta2 = lerp(from->joint_angles[leg].theta2, to->joint_angles[leg].theta2, st);
        result->joint_angles[leg].theta3 = lerp(from->joint_angles[leg].theta3, to->joint_angles[leg].theta3, st);
    }
}
