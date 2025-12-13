/**
 * @file spot_kinematics_v2.h
 * @brief Improved Inverse Kinematics for SpotMicro Quadruped Robot
 * 
 * Based on: "Implementing Kinematics of a four-legged Robot"
 * 
 * Leg Configuration (4 segments per leg):
 *   l1 = Coxa length (shoulder lateral offset)
 *   l2 = Shoulder joint offset (small vertical offset)
 *   l3 = Upper leg (femur - shoulder to knee)
 *   l4 = Lower leg (tibia - knee to foot)
 * 
 * Coordinate System:
 *   X: Forward (positive) / Backward (negative)
 *   Y: Right (positive) / Left (negative) 
 *   Z: Up (positive) / Down (negative)
 * 
 * Body pose defined by:
 *   omega = Roll (rotation around X axis)
 *   phi   = Pitch (rotation around Y axis)
 *   psi   = Yaw (rotation around Z axis)
 *   xm, ym, zm = Body center translation
 */

#ifndef SPOT_KINEMATICS_V2_H
#define SPOT_KINEMATICS_V2_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Physical Dimensions (mm) - MEASURE YOUR ROBOT AND ADJUST!
// ============================================================================

// Leg segment lengths (based on SpotMicro design)
#define SPOT_L1  29.0f    // Coxa: Shoulder lateral offset (side extension)
#define SPOT_L2  10.0f    // Shoulder: Small offset at shoulder joint
#define SPOT_L3  55.0f    // Femur: Upper leg length (shoulder to knee)
#define SPOT_L4  58.0f   // Tibia: Lower leg length (knee to foot)

// Body dimensions (center to hip joint)
#define SPOT_BODY_LENGTH  140.0f   // Front-to-back hip distance (half = 70mm each side)
#define SPOT_BODY_WIDTH   110.0f    // Left-to-right hip distance (half = 55mm each side)

// ============================================================================
// Servo Neutral Angles - DEFAULT VALUES (can be overridden by calibration)
// ============================================================================
// When servos are at these angles, legs point straight down

// Shoulder neutral angles (horizontal shoulder = vertical leg)
#define NEUTRAL_SHOULDER_FR  60.0f
#define NEUTRAL_SHOULDER_FL  120.0f
#define NEUTRAL_SHOULDER_RR  120.0f
#define NEUTRAL_SHOULDER_RL  60.0f

// Elbow neutral angles
#define NEUTRAL_ELBOW_FR     90.0f
#define NEUTRAL_ELBOW_FL     90.0f
#define NEUTRAL_ELBOW_RR     90.0f
#define NEUTRAL_ELBOW_RL     90.0f

// Wrist neutral angles - extended for paw contact (not wrist)
#define NEUTRAL_WRIST_FR     50.0f
#define NEUTRAL_WRIST_FL     130.0f
#define NEUTRAL_WRIST_RR     50.0f
#define NEUTRAL_WRIST_RL     130.0f

// External calibrated values (defined in main_controller.cpp)
// These override the defaults when set
extern int calibrated_shoulder[4];
extern int calibrated_elbow[4];
extern int calibrated_wrist[4];

// ============================================================================
// Data Types
// ============================================================================

// Leg indices - matching main_controller convention
typedef enum {
    LEG_FR = 0,  // Front Right (index 0 in arrays)
    LEG_FL = 1,  // Front Left
    LEG_RR = 2,  // Rear Right
    LEG_RL = 3   // Rear Left
} LegIndex;

// 3D Vector
typedef struct {
    float x, y, z;
} Vec3;

// 4x4 Transformation Matrix (row-major)
typedef struct {
    float m[4][4];
} Mat4;

// Joint angles for one leg (in radians internally, degrees for servos)
typedef struct {
    float theta1;  // Shoulder/Coxa angle
    float theta2;  // Elbow/Femur angle  
    float theta3;  // Wrist/Tibia angle
} LegAngles;

// Servo angles for one leg (in degrees, ready for servo)
typedef struct {
    float shoulder;
    float elbow;
    float wrist;
} ServoAngles;

// Body pose
typedef struct {
    float omega;  // Roll (X rotation) in radians
    float phi;    // Pitch (Y rotation) in radians
    float psi;    // Yaw (Z rotation) in radians
    float x;      // X translation
    float y;      // Y translation
    float z;      // Z translation (body height)
} BodyPose;

// Complete robot state
typedef struct {
    Vec3 foot_positions[4];      // Foot positions in world space
    LegAngles joint_angles[4];   // Joint angles (radians)
    ServoAngles servo_angles[4]; // Servo angles (degrees)
    BodyPose body_pose;
} RobotState;

// Gait parameters
typedef struct {
    float step_length;       // Forward step distance (mm)
    float step_height;       // Foot lift height (mm)
    float body_height;       // Standing height (mm)
    uint32_t cycle_time_ms;  // Time for one complete gait cycle
    float duty_factor;       // Fraction of cycle foot is on ground (0.5-0.8)
    uint8_t phase_offset[4]; // Phase offset for each leg (0-100%)
} GaitParams;

// ============================================================================
// Function Prototypes
// ============================================================================

// --- Utility Functions ---
float deg2rad(float deg);
float rad2deg(float rad);
float clampf(float val, float min, float max);

// --- Matrix Operations ---
void mat4_identity(Mat4* m);
void mat4_multiply(const Mat4* a, const Mat4* b, Mat4* result);
void mat4_transform_point(const Mat4* m, const Vec3* p, Vec3* result);
void mat4_inverse(const Mat4* m, Mat4* result);

// --- Leg Inverse Kinematics ---
/**
 * @brief Calculate leg joint angles from foot position in leg space
 * @param x Forward position (mm)
 * @param y Lateral position (mm) - positive = outward from body
 * @param z Vertical position (mm) - negative = down
 * @param angles Output joint angles (radians)
 * @param is_right_side true for FR/RR legs, false for FL/RL legs
 * @return true if position is reachable
 */
bool leg_ik(float x, float y, float z, LegAngles* angles, bool is_right_side);
bool leg_ik_simple(float x, float y, float z, LegAngles* angles);  // Backward compat

// --- Body Inverse Kinematics ---
/**
 * @brief Calculate transformation matrices for all 4 legs based on body pose
 * @param pose Body pose (rotation + translation)
 * @param Tlf Output: Left Front leg transformation
 * @param Trf Output: Right Front leg transformation  
 * @param Tlb Output: Left Back leg transformation
 * @param Trb Output: Right Back leg transformation
 */
void body_ik(const BodyPose* pose, Mat4* Tlf, Mat4* Trf, Mat4* Tlb, Mat4* Trb);

// --- Complete Robot IK ---
/**
 * @brief Calculate all servo angles for given foot positions and body pose
 * @param foot_positions Array of 4 foot positions in world space
 * @param body_pose Body pose
 * @param state Output robot state with calculated angles
 */
void robot_ik(const Vec3 foot_positions[4], const BodyPose* body_pose, RobotState* state);

// --- Convert to Servo Angles ---
/**
 * @brief Convert joint angles (radians) to servo angles (degrees)
 * @param leg Which leg
 * @param joint_angles Joint angles in radians
 * @param servo_angles Output servo angles in degrees
 */
void joint_to_servo(LegIndex leg, const LegAngles* joint_angles, ServoAngles* servo_angles);

// --- Gait Functions ---
void gait_init_walk(GaitParams* params);
void gait_init_trot(GaitParams* params);
void gait_update(RobotState* state, const GaitParams* params, float phase, bool forward);

// --- Turn Functions ---
/**
 * @brief Turn the robot in place by rotating body yaw
 * @param state Robot state to update
 * @param angle_deg Turn angle in degrees (positive = left, negative = right)
 * @param params Gait parameters
 * @param phase Current gait phase (0-1)
 */
void gait_turn(RobotState* state, float angle_deg, const GaitParams* params, float phase);

// --- Interpolation ---
float lerp(float a, float b, float t);
float smoothstep(float t);
void vec3_lerp(const Vec3* a, const Vec3* b, float t, Vec3* result);

/**
 * @brief Smoothly interpolate between two servo angle sets
 * @param from Starting servo angles
 * @param to Target servo angles
 * @param t Interpolation factor (0-1)
 * @param result Output interpolated angles
 */
void servo_angles_lerp(const ServoAngles* from, const ServoAngles* to, float t, ServoAngles* result);

/**
 * @brief Interpolate entire robot state
 * @param from Starting state
 * @param to Target state
 * @param t Interpolation factor (0-1)
 * @param result Output interpolated state
 */
void robot_state_lerp(const RobotState* from, const RobotState* to, float t, RobotState* result);

// --- State Management ---
void robot_state_init(RobotState* state, float body_height);
void robot_stand(RobotState* state, float body_height);

#ifdef __cplusplus
}
#endif

#endif // SPOT_KINEMATICS_V2_H
