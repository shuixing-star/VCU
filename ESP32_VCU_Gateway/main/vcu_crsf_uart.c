#include "vcu_crsf_uart.h"

#include <string.h>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define VCU_CRSF_RX_BUFFER_SIZE 512
#define VCU_CRSF_TASK_STACK_SIZE 3072
#define VCU_CRSF_TASK_PRIORITY 8
#define VCU_CRSF_LINK_TIMEOUT_MS 150U

typedef struct {
    vcu_crsf_t parser;
    int uart_num;
    bool initialized;
    bool task_running;
    uint32_t uart_error_count;
    portMUX_TYPE lock;
} crsf_uart_context_t;

static crsf_uart_context_t m_ctx = {
    .uart_num = -1,
    .lock = portMUX_INITIALIZER_UNLOCKED,
};

static uint32_t now_ms(void) {
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void crsf_uart_task(void *arg) {
    (void)arg;
    uint8_t data[64];

    portENTER_CRITICAL(&m_ctx.lock);
    m_ctx.task_running = true;
    portEXIT_CRITICAL(&m_ctx.lock);

    while (m_ctx.initialized) {
        const int count = uart_read_bytes(m_ctx.uart_num, data, sizeof(data), pdMS_TO_TICKS(20));
        if (count > 0) {
            portENTER_CRITICAL(&m_ctx.lock);
            vcu_crsf_feed(&m_ctx.parser, data, (size_t)count, now_ms());
            portEXIT_CRITICAL(&m_ctx.lock);
        } else if (count < 0) {
            portENTER_CRITICAL(&m_ctx.lock);
            m_ctx.uart_error_count++;
            portEXIT_CRITICAL(&m_ctx.lock);
        }
    }

    portENTER_CRITICAL(&m_ctx.lock);
    m_ctx.task_running = false;
    portEXIT_CRITICAL(&m_ctx.lock);
    vTaskDelete(NULL);
}

bool vcu_crsf_uart_get_state(vcu_crsf_uart_state_t *state) {
    if (state == NULL) return false;

    portENTER_CRITICAL(&m_ctx.lock);
    memset(state, 0, sizeof(*state));
    state->initialized = m_ctx.initialized;
    state->task_running = m_ctx.task_running;
    state->uart_error_count = m_ctx.uart_error_count;
    state->valid_frame_count = m_ctx.parser.valid_frame_count;
    state->crc_error_count = m_ctx.parser.crc_error_count;
    state->last_valid_frame_ms = m_ctx.parser.last_valid_frame_ms;
    memcpy(state->channels, m_ctx.parser.channels, sizeof(state->channels));
    state->link_ok = vcu_crsf_link_ok(&m_ctx.parser, now_ms(), VCU_CRSF_LINK_TIMEOUT_MS);
    portEXIT_CRITICAL(&m_ctx.lock);
    return true;
}

bool vcu_crsf_uart_init(int uart_num, int pin_tx, int pin_rx, int baudrate) {
    if (m_ctx.initialized || uart_num < 0 || uart_num >= UART_NUM_MAX) return false;


    const uart_config_t config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    vcu_crsf_init(&m_ctx.parser, VCU_CRSF_CHANNEL_MID);
    m_ctx.uart_num = uart_num;
    m_ctx.uart_error_count = 0U;

    if (uart_driver_install(uart_num, VCU_CRSF_RX_BUFFER_SIZE, 0, 0, NULL, 0) != ESP_OK ||
        uart_param_config(uart_num, &config) != ESP_OK ||
        uart_set_pin(uart_num, pin_tx, pin_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        if (uart_is_driver_installed(uart_num)) uart_driver_delete(uart_num);
        m_ctx.uart_num = -1;
        return false;
    }

    m_ctx.initialized = true;
    if (xTaskCreatePinnedToCore(crsf_uart_task, "crsf_rx", VCU_CRSF_TASK_STACK_SIZE,
            NULL, VCU_CRSF_TASK_PRIORITY, NULL, tskNO_AFFINITY) != pdPASS) {
        m_ctx.initialized = false;
        uart_driver_delete(uart_num);
        m_ctx.uart_num = -1;
        return false;
    }

    return true;
}

