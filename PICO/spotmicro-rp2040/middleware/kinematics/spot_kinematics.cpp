/**
 * @file spot_kinematics.cpp
 * @brief Inverse Kinematics Implementation for SpotMicro
 */

#include "spot_kinematics.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ============================================================================
// Utility Functions
// ============================================================================

float spot_deg_to_rad(float degrees)
{
    return degrees * M_PI / 180.0f;
}

float spot_rad_to_deg(float radians)
{
    return radians * 180.0f / M_PI;
}

float spot_clamp(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

float spot_lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}

float spot_smoothstep(float t)
{
    // Hermite interpolation for smooth acceleration/deceleration
    t = spot_clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

void spot_interpolate_position(const point3d_t *from, const point3d_t *to,
                                float t, point3d_t *result)
{
    float smooth_t = spot_smoothstep(t);
    result->x = spot_lerp(from->x, to->x, smooth_t);
    result->y = spot_lerp(from->y, to->y, smooth_t);
    result->z = spot_lerp(from->z, to->z, smooth_t);
}

void spot_interpolate_angles(const leg_angles_t *from, const leg_angles_t *to,
                              float t, leg_angles_t *result)
{
    float smooth_t = spot_smoothstep(t);
    result->shoulder = spot_lerp(from->shoulder, to->shoulder, smooth_t);
    result->elbow = spot_lerp(from->elbow, to->elbow, smooth_t);
    result->wrist = spot_lerp(from->wrist, to->wrist, smooth_t);
}

// ============================================================================
// Inverse Kinematics
// ============================================================================

bool spot_ik_calculate(const point3d_t *foot_pos, leg_angles_t *angles, leg_index_t leg_index)
{
    float x = foot_pos->x;  // Forward/backward
    float y = foot_pos->y;  // Lateral
    float z = foot_pos->z;  // Vertical (negative is down)
    
    // Mirror Y for left side legs
    bool is_left = (leg_index == LEG_FL || leg_index == LEG_RL);
    if (is_left) {
        y = -y;
    }
    
    // Step 1: Calculate shoulder angle (lateral/abduction)
    // Shoulder rotates the leg sideways
    float y_offset = y - SPOT_SHOULDER_OFFSET;
    float d_yz = sqrtf(y_offset * y_offset + z * z);
    
    if (d_yz < 0.001f) {
        // Foot directly below hip, shoulder at 0
        angles->shoulder = 0.0f;
    } else {
        angles->shoulder = spot_rad_to_deg(atan2f(y_offset, -z));
    }
    
    // Step 2: Calculate 2-link IK for elbow and wrist
    // Project onto the leg plane (ignoring shoulder rotation)
    float d_xz = sqrtf(x * x + d_yz * d_yz);
    
    // Check if position is reachable
    float max_reach = SPOT_L1_LENGTH + SPOT_L2_LENGTH;
    float min_reach = fabsf(SPOT_L1_LENGTH - SPOT_L2_LENGTH);
    
    if (d_xz > max_reach * 0.98f) {
        // Target too far, clamp to maximum reach
        d_xz = max_reach * 0.98f;
    }
    if (d_xz < min_reach * 1.02f) {
        // Target too close
        d_xz = min_reach * 1.02f;
    }
    
    // Law of cosines to find elbow angle
    float cos_elbow = (SPOT_L1_LENGTH * SPOT_L1_LENGTH + 
                       SPOT_L2_LENGTH * SPOT_L2_LENGTH - 
                       d_xz * d_xz) / 
                      (2.0f * SPOT_L1_LENGTH * SPOT_L2_LENGTH);
    
    cos_elbow = spot_clamp(cos_elbow, -1.0f, 1.0f);
    float elbow_angle = acosf(cos_elbow);  // Interior angle at elbow
    
    // Convert to servo angle (180 - interior angle for typical servo mounting)
    angles->elbow = 180.0f - spot_rad_to_deg(elbow_angle);
    
    // Step 3: Calculate upper leg (wrist) angle
    // Angle from vertical to upper leg
    float alpha = atan2f(x, d_yz);  // Angle due to forward/back position
    
    // Angle contribution from leg geometry
    float cos_beta = (SPOT_L1_LENGTH * SPOT_L1_LENGTH + 
                      d_xz * d_xz - 
                      SPOT_L2_LENGTH * SPOT_L2_LENGTH) / 
                     (2.0f * SPOT_L1_LENGTH * d_xz);
    
    cos_beta = spot_clamp(cos_beta, -1.0f, 1.0f);
    float beta = acosf(cos_beta);
    
    angles->wrist = spot_rad_to_deg(alpha + beta);
    
    // Mirror angles for left side
    if (is_left) {
        angles->shoulder = -angles->shoulder;
    }
    
    // Clamp angles to safe servo range
    angles->shoulder = spot_clamp(angles->shoulder, -45.0f, 45.0f);
    angles->elbow = spot_clamp(angles->elbow, 0.0f, 180.0f);
    angles->wrist = spot_clamp(angles->wrist, 0.0f, 180.0f);
    
    return true;
}

void spot_fk_calculate(const leg_angles_t *angles, point3d_t *foot_pos, leg_index_t leg_index)
{
    bool is_left = (leg_index == LEG_FL || leg_index == LEG_RL);
    
    float shoulder_rad = spot_deg_to_rad(angles->shoulder);
    float elbow_rad = spot_deg_to_rad(180.0f - angles->elbow);  // Convert back to interior angle
    float wrist_rad = spot_deg_to_rad(angles->wrist);
    
    // Calculate leg length projections
    float l1_proj = SPOT_L1_LENGTH * cosf(wrist_rad - elbow_rad / 2.0f);
    float l2_proj = SPOT_L2_LENGTH * cosf(wrist_rad + elbow_rad / 2.0f);
    
    float leg_length = l1_proj + l2_proj;
    
    // Forward position
    foot_pos->x = leg_length * sinf(wrist_rad);
    
    // Vertical position
    float z_base = -leg_length * cosf(wrist_rad);
    foot_pos->z = z_base * cosf(shoulder_rad);
    
    // Lateral position
    float y_offset = -z_base * sinf(shoulder_rad) + SPOT_SHOULDER_OFFSET;
    foot_pos->y = is_left ? -y_offset : y_offset;
}

// ============================================================================
// Gait Generation
// ============================================================================

void spot_gait_init_default(gait_params_t *params)
{
    params->step_length = 30.0f;      // 30mm forward step (conservative start)
    params->step_height = 25.0f;      // 25mm foot lift
    params->stance_width = 50.0f;     // 50mm from center laterally
    params->body_height = 130.0f;     // 130mm standing height
    params->cycle_time_ms = 1500;     // 1.5 seconds per gait cycle (slower for stability)
    
    // Trot gait: diagonal legs move together
    // FR and RL at 0%, FL and RR at 50%
    params->phase_offset[LEG_FR] = 0;
    params->phase_offset[LEG_FL] = 50;
    params->phase_offset[LEG_RR] = 50;
    params->phase_offset[LEG_RL] = 0;
}

void spot_gait_walk_trajectory(float phase, const gait_params_t *params,
                                point3d_t *foot_pos, bool is_swing)
{
    // Default stance position
    foot_pos->y = params->stance_width;
    foot_pos->z = -params->body_height;
    
    if (is_swing) {
        // Swing phase (0.0 to 1.0): Foot in air, moving forward
        // Use parabolic trajectory for height
        float swing_t = phase;
        
        // X position: rear to front
        foot_pos->x = spot_lerp(-params->step_length / 2.0f, 
                                 params->step_length / 2.0f, 
                                 swing_t);
        
        // Z position: parabolic arc
        float height_factor = 4.0f * swing_t * (1.0f - swing_t);  // Peaks at 0.5
        foot_pos->z = -params->body_height + params->step_height * height_factor;
    } else {
        // Stance phase (0.0 to 1.0): Foot on ground, pushing back
        float stance_t = phase;
        
        // X position: front to rear (opposite of swing)
        foot_pos->x = spot_lerp(params->step_length / 2.0f, 
                                -params->step_length / 2.0f, 
                                stance_t);
        
        // Z stays at ground level
        foot_pos->z = -params->body_height;
    }
}

void spot_gait_update_walk(spot_state_t *state, const gait_params_t *params, float global_phase)
{
    // Swing duty cycle (percentage of time foot is in air)
    const float swing_duty = 0.25f;  // 25% swing, 75% stance for walk
    
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        // Calculate this leg's phase with offset
        float leg_phase = global_phase + (float)params->phase_offset[leg] / 100.0f;
        
        // Wrap to 0-1
        while (leg_phase >= 1.0f) leg_phase -= 1.0f;
        while (leg_phase < 0.0f) leg_phase += 1.0f;
        
        bool is_swing;
        float phase_in_period;
        
        if (leg_phase < swing_duty) {
            // In swing phase
            is_swing = true;
            phase_in_period = leg_phase / swing_duty;
        } else {
            // In stance phase
            is_swing = false;
            phase_in_period = (leg_phase - swing_duty) / (1.0f - swing_duty);
        }
        
        // Get foot trajectory
        point3d_t foot_pos;
        spot_gait_walk_trajectory(phase_in_period, params, &foot_pos, is_swing);
        
        // Adjust lateral position based on leg
        bool is_left = (leg == LEG_FL || leg == LEG_RL);
        if (is_left) {
            foot_pos.y = -params->stance_width;
        } else {
            foot_pos.y = params->stance_width;
        }
        
        // Store foot position
        state->foot_positions[leg] = foot_pos;
        
        // Calculate joint angles using IK
        spot_ik_calculate(&foot_pos, &state->joint_angles[leg], (leg_index_t)leg);
    }
}

// ============================================================================
// State Management
// ============================================================================

void spot_state_init(spot_state_t *state)
{
    memset(state, 0, sizeof(spot_state_t));
    
    // Default standing position
    state->body_height = 120.0f;
    
    // Initialize foot positions for standing
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        bool is_left = (leg == LEG_FL || leg == LEG_RL);
        
        state->foot_positions[leg].x = 0.0f;
        state->foot_positions[leg].y = is_left ? -35.0f : 35.0f;
        state->foot_positions[leg].z = -state->body_height;
        
        // Calculate initial joint angles
        spot_ik_calculate(&state->foot_positions[leg], 
                         &state->joint_angles[leg], 
                         (leg_index_t)leg);
    }
    
    // Set default servo mapping
    spot_set_default_servo_mapping(state);
}

void spot_set_default_servo_mapping(spot_state_t *state)
{
    // YOUR ACTUAL SERVO MAPPING (calibrated):
    // Channel  0: FL Elbow
    // Channel  1: FR Elbow
    // Channel  2: FL Shoulder
    // Channel  3: FR Shoulder
    // Channel  4: FR Wrist
    // Channel  5: FL Wrist
    // Channel  6: RL Wrist
    // Channel  7: RR Wrist
    // Channel  8: RR Shoulder
    // Channel  9: RL Shoulder
    // Channel 10: RR Elbow
    // Channel 11: RL Elbow
    
    // Front Right (FR)
    state->servo_config[LEG_FR].shoulder_channel = 3;
    state->servo_config[LEG_FR].elbow_channel = 1;
    state->servo_config[LEG_FR].wrist_channel = 4;
    state->servo_config[LEG_FR].shoulder_direction = 1;
    state->servo_config[LEG_FR].elbow_direction = 1;
    state->servo_config[LEG_FR].wrist_direction = 1;
    state->servo_config[LEG_FR].shoulder_offset = 90.0f;
    state->servo_config[LEG_FR].elbow_offset = 90.0f;
    state->servo_config[LEG_FR].wrist_offset = 90.0f;
    
    // Front Left (FL)
    state->servo_config[LEG_FL].shoulder_channel = 2;
    state->servo_config[LEG_FL].elbow_channel = 0;
    state->servo_config[LEG_FL].wrist_channel = 5;
    state->servo_config[LEG_FL].shoulder_direction = -1;  // Mirrored
    state->servo_config[LEG_FL].elbow_direction = -1;
    state->servo_config[LEG_FL].wrist_direction = -1;
    state->servo_config[LEG_FL].shoulder_offset = 90.0f;
    state->servo_config[LEG_FL].elbow_offset = 90.0f;
    state->servo_config[LEG_FL].wrist_offset = 90.0f;
    
    // Rear Right (RR)
    state->servo_config[LEG_RR].shoulder_channel = 8;
    state->servo_config[LEG_RR].elbow_channel = 10;
    state->servo_config[LEG_RR].wrist_channel = 7;
    state->servo_config[LEG_RR].shoulder_direction = 1;
    state->servo_config[LEG_RR].elbow_direction = 1;
    state->servo_config[LEG_RR].wrist_direction = 1;
    state->servo_config[LEG_RR].shoulder_offset = 90.0f;
    state->servo_config[LEG_RR].elbow_offset = 90.0f;
    state->servo_config[LEG_RR].wrist_offset = 90.0f;
    
    // Rear Left (RL)
    state->servo_config[LEG_RL].shoulder_channel = 9;
    state->servo_config[LEG_RL].elbow_channel = 11;
    state->servo_config[LEG_RL].wrist_channel = 6;
    state->servo_config[LEG_RL].shoulder_direction = -1;  // Mirrored
    state->servo_config[LEG_RL].elbow_direction = -1;
    state->servo_config[LEG_RL].wrist_direction = -1;
    state->servo_config[LEG_RL].shoulder_offset = 90.0f;
    state->servo_config[LEG_RL].elbow_offset = 90.0f;
    state->servo_config[LEG_RL].wrist_offset = 90.0f;
}

void spot_joint_to_servo_angle(const spot_state_t *state, leg_index_t leg,
                                const leg_angles_t *joint_angles,
                                float *servo_angles)
{
    const leg_servo_config_t *config = &state->servo_config[leg];
    
    // Apply direction and offset to convert joint angles to servo angles
    servo_angles[0] = config->shoulder_offset + 
                      joint_angles->shoulder * config->shoulder_direction;
    servo_angles[1] = config->elbow_offset + 
                      joint_angles->elbow * config->elbow_direction;
    servo_angles[2] = config->wrist_offset + 
                      joint_angles->wrist * config->wrist_direction;
    
    // Clamp to valid servo range
    for (int i = 0; i < 3; i++) {
        servo_angles[i] = spot_clamp(servo_angles[i], 0.0f, 180.0f);
    }
}
