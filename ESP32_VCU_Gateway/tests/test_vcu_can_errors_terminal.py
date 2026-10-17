"""Exercise the real terminal dispatcher and CAN error callback on the host."""
import argparse
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def run(cc):
    gateway = (ROOT / 'main/vcu_stm32_gateway.c').read_text(encoding='utf-8')
    terminal = (ROOT / 'main/terminal.c').read_text(encoding='utf-8')
    # Compile production callback and dispatch prefix; replace only RTOS/printing.
    context = gateway[gateway.index('#define VCU_MOTOR_RAW_FRAME_COUNT'):gateway.index('static uint8_t s_command_sequence;')]
    callback = gateway[gateway.index('static const char *can_error_type_name(uint8_t type)\n{'):gateway.index('static void terminal_request_reset(int argc')]
    dispatcher = terminal[terminal.index('void terminal_process_string(char *str) {'):terminal.index('\tif (strcmp(argv[0], "threads") == 0) {')] + '}\n'
    prelude = r'''
#include <assert.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "vcu_stm32_gateway.h"
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
static char output[32768];
static uint32_t now_ms(void) { return 10000U; }
static void commands_printf(const char *format, ...) {
    va_list args;
    size_t used = strlen(output);
    va_start(args, format);
    vsnprintf(output + used, sizeof(output) - used, format, args);
    va_end(args);
    strcat(output, "\n");
}
'''
    registration = r'''
static struct { const char *command; void (*cbf)(int, const char **); }
callbacks[] = {{"vcu_can_errors", terminal_can_errors}};
static int callback_write = 1;
'''
    tests = r'''
static void command(const char *text) {
    char buffer[256];
    strcpy(buffer, text);
    output[0] = '\0';
    terminal_process_string(buffer);
}
int main(void) {
    (void)s_motor_raw_ids;
    (void)s_motor_raw_names;
    command("vcu_can_errors");
    assert(strstr(output, "total=0 stored=0 sequence_gaps=0"));
    assert(!strstr(output, "Usage:"));
    m_ctx.can_error_log[0].event.sequence = 191;
    m_ctx.can_error_log[0].event.type = VCU_GATEWAY_CAN_ERROR_HAL_IRQ;
    m_ctx.can_error_log[0].event.flags = 4;
    m_ctx.can_error_log[0].event.tx_error_count = 8;
    m_ctx.can_error_next = 1;
    m_ctx.has_can_error_sequence = true;
    m_ctx.public_state.has_can_error = true;
    m_ctx.public_state.can_error_event_count = 10;
    m_ctx.public_state.can_error_stored_count = 1;
    m_ctx.public_state.can_error_sequence_gap_count = 2;
    command("VCU_CAN_ERRORS  ");
    assert(strstr(output, "total=10 stored=1 sequence_gaps=2"));
    assert(strstr(output, "#191 type=HAL_IRQ(1) lec=BIT_RECESSIVE(4)"));
    assert(m_ctx.public_state.can_error_stored_count == 1);
    m_ctx.can_error_log[0].event.type = VCU_GATEWAY_CAN_ERROR_TX_TIMEOUT;
    command("vcu_can_errors");
    assert(strstr(output, "tx_probe=MISSING"));
    m_ctx.can_error_log[0].probe_mask=1;
    command("vcu_can_errors");
    assert(strstr(output, "tx_probe=PARTIAL"));
    m_ctx.can_error_log[0].probe_mask=VCU_GATEWAY_TX_PROBE_COMPLETE;
    m_ctx.can_error_log[0].probe[0]=0x1C000303;
    m_ctx.can_error_log[0].probe[2]=0x0C01EFD0;
    m_ctx.can_error_log[0].probe[3]=0x0C01EFD1;
    m_ctx.can_error_log[0].probe[4]=2510;
    m_ctx.can_error_log[0].probe[5]=2500;
    m_ctx.can_error_log[0].probe[6]=0x0102010D;
    command("vcu_can_errors");
    assert(strstr(output, "source=CONTROL_PAIR trigger=DEADLINE"));
    assert(strstr(output, "COMPLETE_AT_SNAPSHOT"));
    assert(strstr(output, "elapsed_us=2510"));
    assert(strstr(output, "task_switches=3"));
    const char *invalid[] = {"vcu_can_errors wrong", "vcu_can_errors clear extra"};
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        command(invalid[i]);
        assert(strstr(output, "Usage: vcu_can_errors [clear]"));
        assert(m_ctx.public_state.can_error_stored_count == 1);
    }
    command("vcu_can_errors clear");
    assert(strstr(output, "error log cleared"));
    assert(!m_ctx.public_state.has_can_error);
    assert(!m_ctx.has_can_error_sequence);
    assert(m_ctx.can_error_next == 0);
    assert(m_ctx.public_state.can_error_event_count == 0);
    assert(m_ctx.public_state.can_error_stored_count == 0);
    assert(m_ctx.public_state.can_error_sequence_gap_count == 0);
    command("vcu_can_errors");
    assert(strstr(output, "total=0 stored=0 sequence_gaps=0"));
    puts("PASS: empty/populated log, command case/spacing, invalid args, clear and re-read");
    return 0;
}
'''
    with tempfile.TemporaryDirectory(prefix='vcu-terminal-test-') as directory:
        source = Path(directory) / 'test.c'
        executable = Path(directory) / 'test.exe'
        source.write_text(prelude + context + callback + registration + dispatcher + tests, encoding='utf-8')
        subprocess.run([cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-I', str(ROOT / 'main'), str(source), '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default='gcc')
    run(parser.parse_args().cc)
