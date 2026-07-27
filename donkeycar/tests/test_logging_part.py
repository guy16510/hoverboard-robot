import json
from collections.abc import Callable

from trashcan_robot.logging_part import JsonRunLogger
from trashcan_robot.state import RobotState
from trashcan_robot.transport import MockMotorTransport


def test_logger_writes_at_configured_telemetry_rate(
    tmp_path,
) -> None:
    now = [100.0]
    clock: Callable[[], float] = lambda: now[0]
    logger = JsonRunLogger(
        RobotState(),
        MockMotorTransport(),
        str(tmp_path),
        model_name=None,
        telemetry_hz=5,
        clock=clock,
    )

    logger.run()
    now[0] = 100.1
    logger.run()
    now[0] = 100.2
    logger.run()

    records = [
        json.loads(line)
        for line in logger.path.read_text(encoding="utf-8").splitlines()
    ]
    assert len(records) == 2
