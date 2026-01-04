"""
Calibration data for box-following robot.

Contains hardware-specific constants for motor speeds and distance thresholds.
"""


class CalibrationData:
    """Calibration constants for box-following behavior."""

    # Motor speeds (0-100)
    FORWARD_SPEED: int = 25
    TURN_SPEED: int = 20
    REVERSE_SPEED: int = 15

    # Distance thresholds (cm)
    BOX_DETECTION_DISTANCE: float = 30.0  # Max distance to detect box
    BOX_TOO_CLOSE_DISTANCE: float = 15.0  # Min comfortable distance
    BOX_OPTIMAL_MIN: float = 15.0  # Lower bound of optimal range
    BOX_OPTIMAL_MAX: float = 25.0  # Upper bound of optimal range
    DISTANCE_CHANGE_THRESHOLD: float = 5.0  # Movement detection threshold
    MIN_DETECTION_DISTANCE: float = 8.0  # Ignore readings below this (ground bounce)

    # Tracking beep settings
    TRACKING_BEEP_FREQ: int = 500  # Hz
    TRACKING_BEEP_DURATION: float = 0.05  # seconds
    TRACKING_BEEP_INTERVAL: int = 10  # loop iterations (10 * 100ms = 1 sec)

    # Movement increments
    FORWARD_STEP_DURATION: float = 0.17  # seconds (1/3 wheel rotation)
    SCAN_WIGGLE_ANGLE: float = 45.0  # Degrees to wiggle left/right when scanning

    # Recovery mode
    MAX_RECOVERY_ATTEMPTS: int = 5  # Lateral search attempts before giving up
