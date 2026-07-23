# SPDX-License-Identifier: GPL-3.0-only
import unittest

from tools.drive_esp32 import (
    disabled,
    fault_clear_confirmed,
    motion_session_healthy,
    validate_motion_result,
    validate_status,
    validate_transport_progress,
    zero_ready,
)


class ValidateMotionResultTest(unittest.TestCase):
    def test_accepts_motion_in_each_commanded_direction(self) -> None:
        validate_motion_result((250, -250), (True, True), (4, -3))

    def test_rejects_a_bridge_that_never_enabled(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "bridge never enabled"):
            validate_motion_result((250, 0), (False, False), (2, 0))

    def test_rejects_no_hall_motion(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "no Hall movement"):
            validate_motion_result((250, 0), (True, False), (0, 0))

    def test_rejects_motion_opposite_the_command(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "opposite"):
            validate_motion_result((250, -250), (True, True), (-1, 1))


class ZeroOutputGateTest(unittest.TestCase):
    def setUp(self) -> None:
        self.status = {
            "session_ready": True,
            "peer_healthy": True,
            "exact_ack": True,
            "states": [1, 1],
            "applied": [0, 0],
            "compare": [0, 0],
            "bridge": [0, 0],
        }

    def test_ready_requires_zero_compare_and_disabled_bridges(self) -> None:
        self.assertTrue(zero_ready(self.status))
        self.status["compare"] = [40, 0]
        self.status["bridge"] = [1, 0]
        self.assertFalse(zero_ready(self.status))

    def test_disabled_requires_zero_compare_and_disabled_bridges(self) -> None:
        self.status["states"] = [0, 0]
        self.assertTrue(disabled(self.status))
        self.status["compare"] = [0, 40]
        self.status["bridge"] = [0, 1]
        self.assertFalse(disabled(self.status))

    def test_motion_session_must_remain_ready_and_operational(self) -> None:
        self.status["states"] = [2, 2]
        self.assertTrue(motion_session_healthy(self.status))
        self.status["session_ready"] = False
        self.assertFalse(motion_session_healthy(self.status))
        self.status["session_ready"] = True
        self.status["states"] = [0, 2]
        self.assertFalse(motion_session_healthy(self.status))

    def test_status_rejects_bridge_applied_command_disagreement(self) -> None:
        self.status.update(
            {
                "protocol": 2,
                "faults": [0, 0],
                "esp_feedback_age_ms": 0,
                "pa4_raw_high": True,
                "pa4_bypass": True,
                "slave_pa4_raw_high": True,
                "halls": [2, 2],
            }
        )
        self.status["compare"] = [40, 0]
        self.status["bridge"] = [1, 0]
        with self.assertRaisesRegex(RuntimeError, "applied/bridge disagreement"):
            validate_status(self.status)

    def test_status_allows_sub_deadband_ramp_with_bridge_off(self) -> None:
        self.status.update(
            {
                "protocol": 2,
                "faults": [0, 0],
                "esp_feedback_age_ms": 0,
                "pa4_raw_high": True,
                "pa4_bypass": True,
                "slave_pa4_raw_high": True,
                "halls": [2, 2],
                "applied": [49, -49],
            }
        )
        validate_status(self.status)


class TransportProgressTest(unittest.TestCase):
    def setUp(self) -> None:
        self.baseline = {
            "tx": 20,
            "rx": 8,
            "crc_errors": 1,
            "ack_timeouts": 0,
        }

    def test_accepts_bidirectional_progress_without_new_errors(self) -> None:
        validate_transport_progress(
            self.baseline,
            {"tx": 40, "rx": 18, "crc_errors": 1, "ack_timeouts": 0},
        )

    def test_rejects_missing_bidirectional_progress(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "did not progress"):
            validate_transport_progress(
                self.baseline,
                {"tx": 20, "rx": 18, "crc_errors": 1, "ack_timeouts": 0},
            )

    def test_rejects_new_crc_or_ack_errors(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "CRC"):
            validate_transport_progress(
                self.baseline,
                {"tx": 40, "rx": 18, "crc_errors": 2, "ack_timeouts": 0},
            )
        with self.assertRaisesRegex(RuntimeError, "acknowledgment timeout"):
            validate_transport_progress(
                self.baseline,
                {"tx": 40, "rx": 18, "crc_errors": 1, "ack_timeouts": 1},
            )


class FaultClearGateTest(unittest.TestCase):
    def setUp(self) -> None:
        self.status = {
            "protocol": 2,
            "states": [3, 3],
            "faults": [1, 16],
            "esp_feedback_age_ms": 0,
            "pa4_raw_high": True,
            "pa4_bypass": True,
            "slave_pa4_raw_high": True,
            "halls": [2, 2],
            "compare": [0, 0],
            "bridge": [0, 0],
            "applied": [0, 0],
            "exact_ack": True,
            "clear_pending": True,
        }

    def test_clear_stage_allows_transient_fault_telemetry(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "Controller fault"):
            validate_status(self.status)
        validate_status(self.status, allow_faults=True)

    def test_clear_confirmation_requires_both_faults_and_pending_to_clear(
        self,
    ) -> None:
        self.assertFalse(fault_clear_confirmed(self.status))
        self.status["states"] = [0, 0]
        self.status["faults"] = [0, 0]
        self.assertFalse(fault_clear_confirmed(self.status))
        self.status["clear_pending"] = False
        self.assertTrue(fault_clear_confirmed(self.status))


if __name__ == "__main__":
    unittest.main()
