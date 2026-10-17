#ifndef VCU_BRAKE_DIAG_H
#define VCU_BRAKE_DIAG_H
#include <stdbool.h>
#include <stdint.h>
/* Additive standard CAN frame. Version 1 is marked by flags bit 7.
 * This is a sampled STM32 observation, not a controller acknowledgement.
 * All fields come from one telemetry slot; tx_seq is the low 8 bits of the
 * most recent control-pair attempt, not a loss counter or source timestamp. */
#define VCU_BRAKE_DIAG_ID 0x689U
#define VCU_BRAKE_DIAG_VERSION 0x80U
#define VCU_BRAKE_DIAG_FRESH_MS 150U
#define VCU_BD_LINK 1U
#define VCU_BD_ARM 2U
#define VCU_BD_RESET 4U
#define VCU_BD_RUN 8U
#define VCU_BD_BRAKE 16U
#define VCU_BD_INPUT_VALID 32U
#define VCU_BD_CANBOX 64U
#define VCU_BD_TX_SUCCESS 16U
#define VCU_BD_TX_VALID 32U
typedef struct {
    uint16_t ch6, ch8;
    uint8_t flags, reason, tx, tx_seq;
} vcu_brake_diag_t;
static inline void vcu_brake_diag_pack(const vcu_brake_diag_t *m, uint8_t d[8]) {
    d[0]=(uint8_t)m->ch6;d[1]=(uint8_t)(m->ch6>>8);
    d[2]=(uint8_t)m->ch8;d[3]=(uint8_t)(m->ch8>>8);
    d[4]=m->flags|VCU_BRAKE_DIAG_VERSION;d[5]=m->reason;
    d[6]=m->tx;d[7]=m->tx_seq;
}
static inline bool vcu_brake_diag_unpack(const uint8_t d[8], vcu_brake_diag_t *m) {
    if (!(d[4]&VCU_BRAKE_DIAG_VERSION) || (d[6]&0xC0U)) return false;
    m->ch6=(uint16_t)d[0]|((uint16_t)d[1]<<8);
    m->ch8=(uint16_t)d[2]|((uint16_t)d[3]<<8);
    m->flags=d[4];m->reason=d[5];m->tx=d[6];m->tx_seq=d[7];
    return true;
}
#endif
