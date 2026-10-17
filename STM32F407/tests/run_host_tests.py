"""Run STM32 VCU host regressions without connecting any hardware."""
import argparse
from pathlib import Path
import subprocess
import tempfile


def run(cc, output):
    firmware = Path(__file__).resolve().parents[1]
    output.mkdir(parents=True, exist_ok=True)
    common = [cc, '-std=c11', '-Wall', '-Wextra', '-Werror']
    for path in ('App/Inc', 'Protocol/Inc', 'Bsp/CAN', 'Bsp/CRSF'):
        common += ['-I', str(firmware / path)]
    protocol = 'Protocol/Src/mx_can_protocol.c'
    motor = 'App/Src/vcu_motor_can.c'
    suites = {
        'test_vcu_crsf_port': [],
        'test_vcu_error_delivery': ['Bsp/CAN/vcu_can_port.c',
            'App/Src/vcu_control_telemetry.c', 'App/Src/vcu_esp_link.c',
            'Protocol/Src/vcu_gateway_protocol.c'],
        'test_vcu_app_init': ['App/Src/vcu_app.c'],
        'test_vcu_can_port': ['Bsp/CAN/vcu_can_port.c'],
        'test_vcu_control_telemetry': ['App/Src/vcu_control_telemetry.c',
            'Protocol/Src/vcu_gateway_protocol.c'],
        'test_mx_can_protocol': [protocol],
        'test_vcu_motor_can': [protocol, motor],
        'test_vcu_drive_runtime': [protocol, motor,
            'App/Src/vcu_drive_runtime.c', 'App/Src/vcu_rc_drive.c',
            'App/Src/vcu_speed_sync.c'],
    }
    for name, sources in suites.items():
        executable = (output / (name + '.exe')).resolve()
        inputs = [firmware / 'tests' / (name + '.c')]
        inputs += [firmware / source for source in sources]
        suite_flags = common
        if name in ('test_vcu_can_port', 'test_vcu_error_delivery'):
            suite_flags = common + ['-I', str(firmware / 'tests/can_hal_mock')]
        if name == 'test_vcu_crsf_port':
            suite_flags = common + ['-I', str(firmware / 'tests/crsf_hal_mock')]
        subprocess.run(suite_flags + [str(path) for path in inputs] +
                       ['-lm', '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
        print(name + ': PASS', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default='gcc', help='GCC-compatible host compiler')
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    if args.output_dir:
        run(args.cc, args.output_dir)
    else:
        with tempfile.TemporaryDirectory(prefix='vcu-host-tests-') as directory:
            run(args.cc, Path(directory))
