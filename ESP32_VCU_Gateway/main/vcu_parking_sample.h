#ifndef VCU_PARKING_SAMPLE_H
#define VCU_PARKING_SAMPLE_H
#include "vcu_log_sample.h"
#include "vcu_parking_observer.h"

/* SD-only fields. Keep the original 94-field Lisp interface unchanged. */
#define VCU_PARKING_FIELD_COUNT 210U
#define VCU_PARKING_CSV_FIELD_COUNT 178U
extern const unsigned int vcu_parking_csv_fields[VCU_PARKING_CSV_FIELD_COUNT];
const vcu_log_field_t *vcu_parking_field(unsigned int index);
void vcu_parking_sample_from_state(const vcu_stm32_gateway_state_t *state,
        const vcu_parking_observer_t observers[2], uint32_t now_ms,
        double values[VCU_PARKING_FIELD_COUNT]);
/* Copies the original state and SD-only observers under the same CAN lock. */
bool vcu_stm32_gateway_get_parking_snapshot(vcu_stm32_gateway_state_t *state,
        vcu_parking_observer_t observers[2]);
#endif
