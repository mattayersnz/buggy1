"""
Calibration data for Raspberry Pi Pico buggy movement.

This module contains hardware-specific calibration values for
accurate position tracking using time-based dead reckoning.
"""


class CalibrationData:
    """Calibration constants for movement and positioning."""

    # Base movement speeds
    FORWARD_SPEED: int = 25
    TURN_SPEED: int = 20
    REVERSE_SPEED: int = 20

    # Time calibrations (from current main.py)
    # These are initial estimates - may need refinement during testing
    TIME_FOR_4_ROTATIONS: float = 2.0  # seconds
    TIME_FOR_90_DEGREE_TURN: float = 5.0  # seconds

    # Derived calibration factors
    # Distance per second at FORWARD_SPEED (25)
    # Assuming 4 wheel rotations = approximately 40cm (10cm per rotation estimate)
    DISTANCE_PER_SECOND_AT_FORWARD_SPEED: float = 20.0  # cm/s at speed 25

    # Degrees per second at TURN_SPEED (20)
    DEGREES_PER_SECOND_AT_TURN_SPEED: float = 18.0  # deg/s (90 degrees in 5 seconds)

    # Speed-to-distance mapping (cm per second at various speeds)
    # Linear interpolation used for speeds not in this table
    SPEED_TO_CM_PER_SECOND = {
        15: 12.0,   # Minimum recommended speed
        20: 16.0,
        25: 20.0,   # FORWARD_SPEED
        30: 24.0,
        35: 28.0,
        40: 32.0,
        50: 40.0,
        60: 48.0,
        75: 60.0,
        100: 80.0   # Maximum speed
    }

    # Position tracking parameters
    BOUNDARY_SIZE: float = 1.0  # meters (1m x 1m square)
    SAFETY_MARGIN: float = 0.1  # meters (10cm buffer from edges)

    # Starting position (center of square)
    START_X: float = 0.5  # meters
    START_Y: float = 0.5  # meters
    START_HEADING: float = 0.0  # degrees (0 = north/forward)

    # Drift compensation (can be adjusted based on testing)
    DISTANCE_DRIFT_FACTOR: float = 1.0  # Multiply distance by this (1.0 = no compensation)
    HEADING_DRIFT_PER_METER: float = 0.0  # degrees of drift per meter traveled

    # Sensor configuration
    OBSTACLE_DETECTION_DISTANCE: float = 25.0  # cm - trigger investigation behavior
    MIN_SAFE_REAR_DISTANCE: float = 3.0  # cm - safety buffer when reversing

    @staticmethod
    def get_cm_per_second(speed):
        """
        Get estimated distance traveled per second at given speed.

        Uses linear interpolation for speeds not in calibration table.

        Args:
            speed: Motor speed (0-100)

        Returns:
            Estimated cm per second
        """
        if speed in CalibrationData.SPEED_TO_CM_PER_SECOND:
            return CalibrationData.SPEED_TO_CM_PER_SECOND[speed]

        # Linear interpolation
        speeds = sorted(CalibrationData.SPEED_TO_CM_PER_SECOND.keys())

        # Clamp to min/max
        if speed <= speeds[0]:
            return CalibrationData.SPEED_TO_CM_PER_SECOND[speeds[0]]
        if speed >= speeds[-1]:
            return CalibrationData.SPEED_TO_CM_PER_SECOND[speeds[-1]]

        # Find bounding speeds
        for i in range(len(speeds) - 1):
            if speeds[i] <= speed <= speeds[i + 1]:
                lower_speed = speeds[i]
                upper_speed = speeds[i + 1]
                lower_cm = CalibrationData.SPEED_TO_CM_PER_SECOND[lower_speed]
                upper_cm = CalibrationData.SPEED_TO_CM_PER_SECOND[upper_speed]

                # Linear interpolation
                ratio = (speed - lower_speed) / (upper_speed - lower_speed)
                return lower_cm + ratio * (upper_cm - lower_cm)

        # Fallback (should never reach here)
        return CalibrationData.DISTANCE_PER_SECOND_AT_FORWARD_SPEED

    @staticmethod
    def calculate_movement_time(distance_cm, speed):
        """
        Calculate time needed to travel a given distance at given speed.

        Args:
            distance_cm: Distance to travel in centimeters
            speed: Motor speed (0-100)

        Returns:
            Time in seconds
        """
        cm_per_second = CalibrationData.get_cm_per_second(speed)
        if cm_per_second <= 0:
            return 0.0
        return distance_cm / cm_per_second

    @staticmethod
    def calculate_turn_time(angle_degrees):
        """
        Calculate time needed to turn a given angle.

        Args:
            angle_degrees: Angle to turn in degrees

        Returns:
            Time in seconds
        """
        if CalibrationData.DEGREES_PER_SECOND_AT_TURN_SPEED <= 0:
            return 0.0
        return abs(angle_degrees) / CalibrationData.DEGREES_PER_SECOND_AT_TURN_SPEED
