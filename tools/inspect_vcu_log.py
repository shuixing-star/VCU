"""Validate a downloaded VCU capture and create a chronological CSV. No hardware IO."""
import argparse
import csv
from pathlib import Path

STATES = ['STOPPED','DISARMED','ARMING_NEUTRAL','ARMED','FAULT']
REASONS = ['INIT','STOPPED','RESET','CRSF_LOST','MOTOR_TIMEOUT','MOTOR_FAULT','KILL','WAITING_NEUTRAL','NEUTRAL_DWELL','ARMED','RUNNING','CONTROL_FAILURE','CAN_TX_FAILURE','SCHEDULE_OVERRUN']

def rows(path):
    with path.open(encoding='utf-8-sig', newline='') as f:
        reader = csv.DictReader(f)
        result = list(reader)
        if any(None in r or any(v is None for v in r.values()) for r in result):
            raise ValueError(f'Column count mismatch: {path}')
        return result

def crc(data):
    value = 65535
    for b in data:
        value ^= b << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x1021 if value & 32768 else 0)) & 65535
    return value

def delta(a,b,bits=32):
    return ((a-b+(1<<(bits-1))) % (1<<bits)) - (1<<(bits-1))

def name(table, value):
    i = int(value)
    return table[i] if 0 <= i < len(table) else f'UNKNOWN_{i}'

def inspect(folder):
    meta = {r['key']:r['value'] for r in rows(folder/'metadata.csv')}
    snapshots = rows(folder/'snapshots.csv')
    tx = rows(folder/'tx_results.csv')
    raw = rows(folder/'raw_events.csv')
    for r in snapshots:
        data = bytes.fromhex(r['record_raw_hex'])
        if len(data)!=176 or crc(data[:-2])!=int.from_bytes(data[-2:],'little'):
            raise ValueError(f"Invalid snapshot CRC: sequence {r['snapshot_seq']}")
    trigger = int(meta['trigger_ms'])
    timeline = []
    for r in snapshots:
        timeline.append(dict(kind='snapshot',esp_rx_dequeue_ms=r['esp_rx_dequeue_ms'],
            relative_trigger_ms=delta(int(r['esp_rx_dequeue_ms']),trigger),
            stm_boot_nonce=r['stm_boot_nonce'],stm_time_ms=r['stm_capture_ms'],
            stm_time_basis='STM32_capture_clock',sequence=r['snapshot_seq'],
            details=f"{name(STATES,r['drive_state'])}/{name(REASONS,r['drive_reason'])}; throttle={r['throttle_per_mille']} per_mille steering={r['steering_per_mille']} per_mille; base={r['left_base_rpm']}/{r['right_base_rpm']} rpm target={r['left_target_rpm']}/{r['right_target_rpm']} rpm; wire={r['left_wire_target_rpm']}/{r['right_wire_target_rpm']} rpm; admitted={r['mailbox_admitted_mask']} TXOK={r['txok_mask']} pair_success={r['pair_success']}; fault={r['left_fault_code']}/{r['right_fault_code']} bits={r['left_fault_bits_hex']}/{r['right_fault_bits_hex']}; current={r['left_bus_current_dA']}/{r['right_bus_current_dA']} dA",
            raw_hex=r['record_raw_hex']))
    for r in tx:
        # Unwrap only against a nearby snapshot in an unambiguous session.
        # This does not synchronize clocks or invent a controller sample time.
        nearby = [s for s in snapshots if abs(delta(int(s['esp_rx_dequeue_ms']),int(r['esp_rx_dequeue_ms']))) < 2000]
        nonce, stamp, basis = '', '', 'STM32_low16_unresolved'
        if nearby and len({s['stm_boot_nonce'] for s in nearby})==1:
            anchor=min(nearby,key=lambda s:abs(delta(int(s['esp_rx_dequeue_ms']),int(r['esp_rx_dequeue_ms']))))
            a=int(anchor['tx_attempt_ms'] or anchor['stm_capture_ms'])
            nonce=anchor['stm_boot_nonce']
            stamp=(a+delta(int(r['stm_attempt_ms_low16']),a & 65535,16)) & 0xffffffff
            basis='STM32_low16_unwrapped_near_snapshot'
        timeline.append(dict(kind='tx_result',esp_rx_dequeue_ms=r['esp_rx_dequeue_ms'],
            relative_trigger_ms=delta(int(r['esp_rx_dequeue_ms']),trigger),stm_boot_nonce=nonce,
            stm_time_ms=stamp,stm_time_basis=basis,sequence=r['attempt_seq_low16'],
            details=f"admitted={r['mailbox_admitted_mask']} TXOK={r['txok_mask']} pair_success={r['pair_success']}; elapsed={r['elapsed_ms']} ms; command heartbeat={r['left_command_heartbeat']}/{r['right_command_heartbeat']}",raw_hex=r['raw_hex']))
    for r in raw:
        ident=int(r['can_id_hex'],16)
        side={0x1802D0EF:'left239',0x1802D0F0:'right240'}.get(ident,'gateway')
        data=bytes.fromhex(r['raw_hex'])
        detail=f"{side} CAN={r['can_id_hex']}; controller sampling time unavailable"
        if side!='gateway': detail+=f'; fault_bits={data[3:6].hex().upper()}'
        timeline.append(dict(kind='raw_event',esp_rx_dequeue_ms=r['esp_rx_dequeue_ms'],
            relative_trigger_ms=delta(int(r['esp_rx_dequeue_ms']),trigger),stm_boot_nonce='',stm_time_ms='',
            stm_time_basis='ESP32_driver_dequeue_only',sequence=r['event_seq'],details=detail,raw_hex=r['raw_hex']))
    timeline.sort(key=lambda r:int(r['relative_trigger_ms']))
    target=folder/'timeline.csv'
    with target.open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=['kind','esp_rx_dequeue_ms','relative_trigger_ms','stm_boot_nonce','stm_time_ms','stm_time_basis','sequence','details','raw_hex'])
        w.writeheader();w.writerows(timeline)
    summary=[f"Source: {meta.get('source','unknown')}",f"Build: {meta.get('log_build','unknown')}",
        f'Snapshots/events: {len(snapshots)}; TX outcomes: {len(tx)}; raw events: {len(raw)}',
        f"Sequence gaps: snapshot={meta.get('snapshot_sequence_gaps')} TX={meta.get('tx_sequence_gaps')}",
        f"Reassembly incomplete={meta.get('reassembly_incomplete')} invalid={meta.get('reassembly_invalid')}",
        f"Controller firmware: {meta.get('controller_firmware','unknown')}",
        f"Complete export marker: {(folder/'COMPLETE.txt').exists()}"]
    if snapshots:
        times=[delta(int(r['esp_rx_dequeue_ms']),trigger) for r in snapshots]
        summary.append(f'Snapshot coverage relative to trigger: {min(times)} .. {max(times)} ms (requested window is not a coverage guarantee)')
    summary += ['Controller sampling timestamps, iqref and high-frequency phase-current snapshots are not available.',
                'TXOK is CAN bus completion, not proof of controller execution.',f'Timeline: {target}']
    text='\n'.join(summary)+'\n'
    (folder/'summary.txt').write_text(text,encoding='utf-8')
    print(text)
    return timeline

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture_directory',type=Path)
    inspect(parser.parse_args().capture_directory)
