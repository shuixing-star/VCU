#ifndef VCU_TYPES_H
#define VCU_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#define VCU_PER_MILLE_MAX 1000

typedef enum {
    VCU_CONTROL_SOURCE_NONE = 0,
    VCU_CONTROL_SOURCE_LOCAL,
    VCU_CONTROL_SOURCE_REMOTE
} vcu_control_source_t;

typedef enum {
    VCU_STATE_INIT = 0,
    VCU_STATE_DISARMED,
    VCU_STATE_READY,
    VCU_STATE_DRIVE,
    VCU_STATE_FAULT
} vcu_state_t;

typedef uint32_t vcu_fault_flags_t;

#define VCU_FAULT_NONE                ((vcu_fault_flags_t)0U)
#define VCU_FAULT_INPUT_INVALID       ((vcu_fault_flags_t)(1UL << 0))
#define VCU_FAULT_REMOTE_TIMEOUT      ((vcu_fault_flags_t)(1UL << 1))
#define VCU_FAULT_MOTOR_LEFT_TIMEOUT  ((vcu_fault_flags_t)(1UL << 2))
#define VCU_FAULT_MOTOR_RIGHT_TIMEOUT ((vcu_fault_flags_t)(1UL << 3))
#define VCU_FAULT_MOTOR_LEFT          ((vcu_fault_flags_t)(1UL << 4))
#define VCU_FAULT_MOTOR_RIGHT         ((vcu_fault_flags_t)(1UL << 5))
#define VCU_FAULT_ACTUATOR            ((vcu_fault_flags_t)(1UL << 6))
#define VCU_FAULT_INTERNAL            ((vcu_fault_flags_t)(1UL << 31))

typedef struct {
    bool link_valid;
    bool enable_requested;
    bool reset_requested;
    bool reverse_requested;
    bool parking_requested;
    int16_t throttle_per_mille;
    int16_t steering_per_mille;
    uint16_t brake_per_mille;
    uint32_t timestamp_ms;
} vcu_remote_input_t;

typedef struct {
    uint16_t throttle_adc;
    uint16_t brake_adc;
    bool reverse_left;
    bool reverse_right;
    bool parking;
    bool remote_selected;
    bool local_inputs_valid;
    vcu_remote_input_t remote;
    uint32_t timestamp_ms;
} vcu_input_snapshot_t;

typedef struct {
    vcu_control_source_t source;
    bool valid;
    bool enable_requested;
    bool reset_requested;
    bool reverse_left;
    bool reverse_right;
    bool parking_requested;
    int16_t throttle_per_mille;
    int16_t steering_per_mille;
    uint16_t brake_per_mille;
    uint32_t timestamp_ms;
} vcu_driver_intent_t;

typedef struct {
    bool valid;
    bool faulted;
    int32_t speed_rpm;
    uint32_t timestamp_ms;
} vcu_motor_feedback_t;

typedef struct {
    vcu_motor_feedback_t left_motor;
    vcu_motor_feedback_t right_motor;
} vcu_vehicle_feedback_t;

typedef struct {
    vcu_state_t state;
    uint32_t fault_flags;
    bool actuation_allowed;
} vcu_safety_output_t;

typedef struct {
    uint16_t left_throttle_per_mille;
    uint16_t right_throttle_per_mille;
    bool left_brake;
    bool right_brake;
    bool left_reverse;
    bool right_reverse;
    bool left_hold;
    bool right_hold;
    bool left_parking;
    bool right_parking;
    bool output_enabled;
    uint32_t timestamp_ms;
} vcu_actuator_command_t;

#endif
