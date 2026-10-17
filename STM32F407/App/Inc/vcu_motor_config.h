#ifndef VCU_MOTOR_CONFIG_H
#define VCU_MOTOR_CONFIG_H

/*
 * Motor CAN configuration for MX_CAN-SV3.03 protocol sections 2.2/2.3.
 * The current VCU/ESP32 bus uses 500 kbit/s. Set 250000UL when all nodes
 * are changed to the protocol document's default rate.
 */
#ifndef VCU_MOTOR_CAN_BITRATE
#define VCU_MOTOR_CAN_BITRATE 500000UL
#endif

#ifndef VCU_MOTOR_LEFT_DRIVER_RECEIVE_ID
#define VCU_MOTOR_LEFT_DRIVER_RECEIVE_ID 0x0C01EFD0UL
#endif
#ifndef VCU_MOTOR_LEFT_DRIVER_SEND_ID
#define VCU_MOTOR_LEFT_DRIVER_SEND_ID 0x1801D0EFUL
#endif

/* Field-confirmed addresses: VCU D0; right F0 (240), left EF (239). */
#ifndef VCU_MOTOR_RIGHT_DRIVER_RECEIVE_ID
#define VCU_MOTOR_RIGHT_DRIVER_RECEIVE_ID 0x0C01F0D0UL
#endif
#ifndef VCU_MOTOR_RIGHT_DRIVER_SEND_ID
#define VCU_MOTOR_RIGHT_DRIVER_SEND_ID 0x1801D0F0UL
#endif

/* Request the adjustable K2 brake ceiling on CH6 high. Each K2 limits this to its
 * user-configured electronic_brake_current_a and existing regen protections.
 * This is a request, not a 1000 A hardware/current-loop setting. */
#define VCU_MOTOR_BRAKE_REQUEST_A 1000U

/* The controller emits its second status frame at Send ID + 0x00010000. */
#define VCU_MOTOR_SECOND_STATUS_ID_OFFSET 0x00010000UL

#endif
