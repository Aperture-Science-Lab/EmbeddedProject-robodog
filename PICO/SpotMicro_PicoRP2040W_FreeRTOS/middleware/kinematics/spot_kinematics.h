/**
 * @file spot_kinematics.h
 * @brief Inverse Kinematics for SpotMicro Quadruped Robot
 * 
 * SpotMicro leg configuration:
 *   - 3 DOF per leg (shoulder, elbow, wrist)
 *   - 4 legs (Front Right, Front Left, Rear Right, Rear Left)
 *   - Total 12 servos
 * 
 * Coordinate System (per leg):
 *   - X: Forward/Backward
 *   - Y: Left/Right (lateral)
 *   - Z: Up/Down (vertical, negative is down)
 * 
 * Leg Dimensions (adjust to your robot):
 *   - L1: Shoulder to elbow length
 *   - L2: Elbow to wrist (foot) length
 */

#ifndef SPOT_KINEMATICS_H
#define SPOT_KINEMATICS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// SpotMicro Physical Dimensions (in mm) - ADJUST THESE TO YOUR ROBOT!
// ============================================================================
// SpotMicro V4 typical dimensions - MEASURE YOUR ACTUAL ROBOT!
#define SPOT_L1_LENGTH      55.0f   // Upper leg (shoulder to elbow) in mm
#define SPOT_L2_LENGTH      107.0f  // Lower leg (elbow to foot) in mm
#define SPOT_SHOULDER_OFFSET 52.0f  // Shoulder lateral offset in mm

// Body dimensions
#define SPOT_BODY_LENGTH    207.0f  // Front to rear hip distance
#define SPOT_BODY_WIDTH     78.0f   // Left to right hip distance

// ============================================================================
// Servo Configuration
// ============================================================================
#define NUM_LEGS            4
#define SERVOS_PER_LEG      3
#define TOTAL_SERVOS        12

// Joint indices within a leg
typedef enum {
    JOINT_SHOULDER = 0,  // Lateral rotation (abduction/adduction)
    JOINT_ELBOW = 1,     // Upper leg rotation
    JOINT_WRIST = 2      // Lower leg rotation
} joint_index_t;

// Leg indices
typedef enum {
    LEG_FR = 0,  // Front Right
    LEG_FL = 1,  // Front Left
    LEG_RR = 2,  // Rear Right
    LEG_RL = 3   // Rear Left
} leg_index_t;

// ============================================================================
// Data Structures
// ============================================================================

// 3D point/vector
typedef struct {
    float x;
    float y;
    float z;
} point3d_t;

// Joint angles for one leg (in degrees)
typedef struct {
    float shoulder;  // Lateral angle (abduction/adduction)
    float elbow;     // Upper leg angle
    float wrist;     // Lower leg angle
} leg_angles_t;

// Servo channel mapping for a leg
typedef struct {
    uint8_t shoulder_channel;
    uint8_t elbow_channel;
    uint8_t wrist_channel;
    int8_t shoulder_direction;  // 1 or -1 for servo direction
    int8_t elbow_direction;
    int8_t wrist_direction;
    float shoulder_offset;      // Calibration offset in degrees
    float elbow_offset;
    float wrist_offset;
} leg_servo_config_t;

// Complete robot state
typedef struct {
    point3d_t foot_positions[NUM_LEGS];   // Foot positions relative to hip
    leg_angles_t joint_angles[NUM_LEGS];  // Calculated joint angles
    leg_servo_config_t servo_config[NUM_LEGS];  // Servo mappings
    float body_height;  // Current body height
} spot_state_t;

// Gait parameters
typedef struct {
    float step_length;      // Forward step length in mm
    float step_height;      // Foot lift height in mm
    float stance_width;     // Lateral stance width in mm
    float body_height;      // Standing height in mm
    uint32_t cycle_time_ms; // Complete gait cycle time
    uint8_t phase_offset[NUM_LEGS];  // Phase offset 0-100%
} gait_params_t;

// ============================================================================
// Inverse Kinematics Functions
// ============================================================================

/**
 * @brief Calculate joint angles from foot position using inverse kinematics
 * @param foot_pos Target foot position relative to hip (mm)
 * @param angles Output joint angles (degrees)
 * @param leg_index Which leg (for mirroring calculations)
 * @return true if solution found, false if unreachable
 */
bool spot_ik_calculate(const point3d_t *foot_pos, leg_angles_t *angles, leg_index_t leg_index);

/**
 * @brief Calculate foot position from joint angles (forward kinematics)
 * @param angles Joint angles (degrees)
 * @param foot_pos Output foot position (mm)
 * @param leg_index Which leg
 */
void spot_fk_calculate(const leg_angles_t *angles, point3d_t *foot_pos, leg_index_t leg_index);

// ============================================================================
// Gait Generation Functions
// ============================================================================

/**
 * @brief Initialize default gait parameters
 * @param params Output gait parameters
 */
void spot_gait_init_default(gait_params_t *params);

/**
 * @brief Calculate foot trajectory for walk gait
 * @param phase Current phase in gait cycle (0.0 to 1.0)
 * @param params Gait parameters
 * @param foot_pos Output foot position
 * @param is_swing true if leg is in swing phase
 */
void spot_gait_walk_trajectory(float phase, const gait_params_t *params, 
                                point3d_t *foot_pos, bool is_swing);

/**
 * @brief Update all leg positions for forward walk
 * @param state Robot state to update
 * @param params Gait parameters
 * @param global_phase Global gait phase (0.0 to 1.0)
 */
void spot_gait_update_walk(spot_state_t *state, const gait_params_t *params, float global_phase);

// ============================================================================
// Interpolation Functions
// ============================================================================

/**
 * @brief Linear interpolation between two values
 */
float spot_lerp(float a, float b, float t);

/**
 * @brief Smooth step interpolation (ease in/out)
 */
float spot_smoothstep(float t);

/**
 * @brief Interpolate between two positions
 */
void spot_interpolate_position(const point3d_t *from, const point3d_t *to, 
                                float t, point3d_t *result);

/**
 * @brief Interpolate between two angle sets
 */
void spot_interpolate_angles(const leg_angles_t *from, const leg_angles_t *to,
                              float t, leg_angles_t *result);

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Convert degrees to radians
 */
float spot_deg_to_rad(float degrees);

/**
 * @brief Convert radians to degrees
 */
float spot_rad_to_deg(float radians);

/**
 * @brief Clamp value to range
 */
float spot_clamp(float value, float min_val, float max_val);

/**
 * @brief Initialize robot state with default standing position
 */
void spot_state_init(spot_state_t *state);

/**
 * @brief Set default servo channel mapping
 */
void spot_set_default_servo_mapping(spot_state_t *state);

/**
 * @brief Convert joint angles to servo angles (applying offsets and directions)
 */
void spot_joint_to_servo_angle(const spot_state_t *state, leg_index_t leg, 
                                const leg_angles_t *joint_angles, 
                                float *servo_angles);

#ifdef __cplusplus
}
#endif

#endif // SPOT_KINEMATICS_H
