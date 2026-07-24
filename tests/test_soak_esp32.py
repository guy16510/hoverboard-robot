# SPDX-License-Identifier: GPL-3.0-only
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from soak_esp32 import (  # noqa: E402
    validate_age_margins,
    validate_soak_progress,
    validate_soak_snapshot,
)


class SoakGateTest(unittest.TestCase):
    def setUp(self) -> None:
        self.baseline = {
            "protocol": 2,
            "states": [1, 1],
            "faults": [0, 0],
            "session_ready": True,
            "peer_healthy": True,
            "exact_ack": True,
            "transport_overflows": [0, 0, 0, 0],
            "esp_feedback_age_ms": 20,
            "command_age_ms": 40,
            "slave_feedback_age_ms": 20,
            "slave_command_age_ms": 20,
            "pa4_raw_high": False,
            "pa4_bypass": True,
            "slave_pa4_raw_high": True,
            "halls": [2, 4],
            "applied": [0, 0],
            "compare": [0, 0],
            "bridge": [0, 0],
            "tx": 10,
            "rx": 10,
            "crc_errors": 0,
            "ack_timeouts": 0,
            "remote_parser": [110, 10, 0, 0],
        }

    def progressed(self) -> dict[str, object]:
        status = dict(self.baseline)
        status.update(
            {
                "tx": 20,
                "rx": 20,
                "remote_parser": [220, 20, 0, 0],
            }
        )
        return status

    def test_accepts_sustained_clean_progress(self) -> None:
        final = self.progressed()
        validate_soak_snapshot(self.baseline, final)
        validate_soak_progress(self.baseline, final, 20)

    def test_rejects_master_invalid_or_framing_growth(self) -> None:
        status = self.progressed()
        status["remote_parser"] = [220, 20, 1, 0]
        with self.assertRaisesRegex(RuntimeError, "invalid command"):
            validate_soak_snapshot(self.baseline, status)

        status["remote_parser"] = [220, 20, 0, 1]
        with self.assertRaisesRegex(RuntimeError, "framing errors"):
            validate_soak_snapshot(self.baseline, status)

    def test_rejects_crc_timeout_and_overflow_growth(self) -> None:
        status = self.progressed()
        status["crc_errors"] = 1
        with self.assertRaisesRegex(RuntimeError, "CRC"):
            validate_soak_snapshot(self.baseline, status)

        status = self.progressed()
        status["ack_timeouts"] = 1
        with self.assertRaisesRegex(RuntimeError, "acknowledgment"):
            validate_soak_snapshot(self.baseline, status)

        status = self.progressed()
        status["transport_overflows"] = [0, 0, 1, 0]
        with self.assertRaisesRegex(RuntimeError, "overflow"):
            validate_soak_snapshot(self.baseline, status)

    def test_rejects_thin_watchdog_age_margins(self) -> None:
        status = self.progressed()
        status["command_age_ms"] = 301
        with self.assertRaisesRegex(RuntimeError, "MASTER command age"):
            validate_age_margins(status)

        status = self.progressed()
        status["slave_feedback_age_ms"] = 81
        with self.assertRaisesRegex(RuntimeError, "SLAVE feedback age"):
            validate_age_margins(status)

        status = self.progressed()
        status["slave_command_age_ms"] = 81
        with self.assertRaisesRegex(RuntimeError, "SLAVE command age"):
            validate_age_margins(status)

    def test_requires_repeated_snapshots_and_parser_progress(self) -> None:
        final = self.progressed()
        with self.assertRaisesRegex(RuntimeError, "Too few"):
            validate_soak_progress(self.baseline, final, 4)

        final["remote_parser"] = [110, 10, 0, 0]
        with self.assertRaisesRegex(RuntimeError, "parser did not make"):
            validate_soak_progress(self.baseline, final, 20)


if __name__ == "__main__":
    unittest.main()
