import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CMAKE = ROOT / "main/CMakeLists.txt"
STREAM = ROOT / "main/f280_fw_stream.c"
BRIDGE = ROOT / "main/f280_fw_bridge.c"
PARTITIONS = (
    ROOT / "partition_ota_4mb.csv",
    ROOT / "partition_ota_8mb.csv",
    ROOT / "partition_ota_16mb.csv",
)


class F280FirmwareStreamContractTests(unittest.TestCase):
    def test_build_contains_only_direct_stream_path(self):
        cmake = CMAKE.read_text(encoding="utf-8")
        self.assertIn('"f280_fw_stream.c"', cmake)
        self.assertIn('"f280_fw_bridge.c"', cmake)
        self.assertNotIn("f280_fw_stage", cmake)
        self.assertNotIn("f280_fw_delivery", cmake)

    def test_no_partition_is_reserved_for_f280_image(self):
        for path in PARTITIONS:
            table = path.read_text(encoding="utf-8")
            self.assertNotIn("f280_fw", table, path.name)

    def test_stream_consumes_tool_prefix_and_fills_erased_gaps(self):
        source = STREAM.read_text(encoding="utf-8")
        self.assertIn("#define STREAM_PREFIX_BYTES       6U", source)
        self.assertIn("static bool advance_erased", source)
        self.assertIn("memset(erased, 0xFF", source)
        self.assertIn("s_stream.running_crc != s_stream.expected_crc", source)

    def test_stm32_build_uses_gateway_safety_state(self):
        bridge = BRIDGE.read_text(encoding="utf-8")
        self.assertIn('#include "hwconf/hw.h"', bridge)
        self.assertIn("#ifdef HW_VCU_STM32_GATEWAY", bridge)
        self.assertIn("vcu_stm32_gateway_get_state", bridge)
        self.assertIn("FW_SAFE_FLAG_RUN_REQUESTED", bridge)
        self.assertIn("FW_SAFE_FLAG_OUTPUT_ENABLED", bridge)
        self.assertIn("FW_SAFE_RC_FLAG_ARM", bridge)
        self.assertIn("state->targets.left_target_rpm != 0", bridge)
        self.assertIn("state->targets.right_target_rpm != 0", bridge)
        self.assertIn("vcu_stm32_gateway_request_stop", bridge)
        self.assertIn("FW_SAFE_STOP_TIMEOUT_MS", bridge)
        self.assertIn("stm32_state_is_safe(&state, true)", bridge)
        self.assertIn("vcu_stm32_gateway_request_run", bridge)
        self.assertIn('"state-unavailable"', bridge)
        self.assertIn('"output-enabled"', bridge)
        self.assertIn('"run-requested"', bridge)
        self.assertIn('"rc-armed"', bridge)
        self.assertIn('"target-nonzero"', bridge)
        self.assertIn('"left-motor-not-ready"', bridge)
        self.assertIn('"right-motor-not-ready"', bridge)
        self.assertIn('"stop-command-tx"', bridge)

    def test_tool_all_can_commands_are_normalized(self):
        bridge = BRIDGE.read_text(encoding="utf-8")
        self.assertIn("static bool decode_tool_command", bridge)
        self.assertIn("COMM_ERASE_NEW_APP_ALL_CAN_HW", bridge)
        self.assertIn("COMM_WRITE_NEW_APP_DATA_ALL_CAN_HW", bridge)
        self.assertIn("COMM_JUMP_TO_BOOTLOADER_ALL_CAN_HW", bridge)
        self.assertIn("*command = COMM_ERASE_NEW_APP;", bridge)
        self.assertIn("*command = COMM_WRITE_NEW_APP_DATA;", bridge)
        self.assertIn("*command = COMM_JUMP_TO_BOOTLOADER;", bridge)
        self.assertIn("index = 2U; /* Skip command and HW_TYPE. */", bridge)
        self.assertIn("*payload_index = index + 1U;", bridge)
        self.assertIn("send_simple_ack(reply, command", bridge)

    def test_bridge_status_reports_dispatch_and_reject_reason(self):
        bridge = BRIDGE.read_text(encoding="utf-8")
        self.assertIn('"bridge diag : v5\\n"', bridge)
        self.assertIn('"dispatches  : %lu\\n"', bridge)
        self.assertIn('"firmware req: %lu\\n"', bridge)
        self.assertIn('"safety block: %s\\n"', bridge)
        self.assertIn("static bool is_firmware_command", bridge)
        self.assertIn("!is_firmware_command(packet[0])", bridge)
        self.assertIn('"last command: raw=%u normalized=%u\\n"', bridge)
        self.assertIn('"last reject : %s\\n"', bridge)
        self.assertIn("BRIDGE_REJECT_TARGET", bridge)
        self.assertIn("BRIDGE_REJECT_DECODE", bridge)
        self.assertIn("BRIDGE_REJECT_ERASE_FORMAT", bridge)
        self.assertIn("BRIDGE_REJECT_VEHICLE_UNSAFE", bridge)
        self.assertIn("BRIDGE_REJECT_F280_BEGIN", bridge)

    def test_stream_status_survives_software_restart_without_flash_writes(self):
        stream = STREAM.read_text(encoding="utf-8")
        bridge = BRIDGE.read_text(encoding="utf-8")
        self.assertIn("RTC_NOINIT_ATTR", stream)
        self.assertIn("retain_status_locked();", stream)
        self.assertIn("f280_fw_stream_get_retained_status", stream)
        self.assertNotIn("nvs_", stream)
        self.assertNotIn("esp_partition_", stream)
        self.assertIn('"bridge diag : v5\\n"', bridge)
        self.assertIn('"retained log: %s\\n"', bridge)
        self.assertIn('"ret offset  : %lu\\n"', bridge)

    def test_failure_aborts_and_requires_a_new_session(self):
        stream = STREAM.read_text(encoding="utf-8")
        bridge = BRIDGE.read_text(encoding="utf-8")
        self.assertIn("F280_UPD_PACKET_ABORT", stream)
        self.assertIn("f280_fw_stream_abort();", stream)
        self.assertIn("s_upload_target = 0xFFU", bridge)
        self.assertNotIn("f280_fw_stage", bridge)



