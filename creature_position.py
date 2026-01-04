"""
Position tracking and boundary detection for the buggy creature.

This module implements time-based dead reckoning to track the buggy's
position within a 1-meter square boundary.
"""

import math
from calibration_data import CalibrationData


class Position:
    """Represents a 2D position with heading."""

    def __init__(self, x, y, heading):
        """
        Initialize position.

        Args:
            x: X coordinate in meters (0.0 to 1.0)
            y: Y coordinate in meters (0.0 to 1.0)
            heading: Heading in degrees (0 = north/forward, clockwise)
        """
        self.x = x
        self.y = y
        self.heading = heading % 360.0  # Normalize to 0-360

    def __repr__(self):
        return f"Position(x={self.x:.3f}m, y={self.y:.3f}m, heading={self.heading:.1f}°)"


class PositionTracker:
    """
    Tracks buggy position using dead reckoning.

    Uses time-based movement estimation to maintain position
    within a defined boundary.
    """

    def __init__(self):
        """Initialize position tracker at center of boundary."""
        self.current_position = Position(
            CalibrationData.START_X,
            CalibrationData.START_Y,
            CalibrationData.START_HEADING
        )
        self.boundary_size = CalibrationData.BOUNDARY_SIZE
        self.safety_margin = CalibrationData.SAFETY_MARGIN

    def update_position(self, distance_cm):
        """
        Update position after linear movement.

        Args:
            distance_cm: Distance traveled in centimeters (negative for reverse)
        """
        distance_m = distance_cm / 100.0  # Convert cm to meters

        # Apply drift compensation
        distance_m *= CalibrationData.DISTANCE_DRIFT_FACTOR

        # Calculate position change based on heading
        # Heading 0° = north (positive Y), 90° = east (positive X)
        heading_rad = math.radians(self.current_position.heading)

        # Using standard mathematical convention
        dx = distance_m * math.sin(heading_rad)
        dy = distance_m * math.cos(heading_rad)

        # Update position
        self.current_position.x += dx
        self.current_position.y += dy

        # Clamp to boundary (safety fallback)
        self.current_position.x = max(0.0, min(self.boundary_size, self.current_position.x))
        self.current_position.y = max(0.0, min(self.boundary_size, self.current_position.y))

    def update_heading(self, angle_degrees):
        """
        Update heading after rotation.

        Args:
            angle_degrees: Angle change in degrees (positive = clockwise, negative = counter-clockwise)
        """
        self.current_position.heading = (self.current_position.heading + angle_degrees) % 360.0

    def is_movement_safe(self, distance_cm):
        """
        Check if a forward movement would stay within boundaries.

        Args:
            distance_cm: Intended forward movement distance in centimeters

        Returns:
            True if movement is safe, False if it would exceed boundaries
        """
        distance_m = distance_cm / 100.0

        # Calculate predicted position
        heading_rad = math.radians(self.current_position.heading)
        predicted_x = self.current_position.x + distance_m * math.sin(heading_rad)
        predicted_y = self.current_position.y + distance_m * math.cos(heading_rad)

        # Check if predicted position is within safe zone
        min_safe = self.safety_margin
        max_safe = self.boundary_size - self.safety_margin

        return (min_safe <= predicted_x <= max_safe and
                min_safe <= predicted_y <= max_safe)

    def get_distance_to_boundary(self, heading_override = None):
        """
        Calculate approximate distance to boundary in current heading direction.

        Args:
            heading_override: Optional heading to check instead of current heading

        Returns:
            Distance to boundary in centimeters
        """
        heading = heading_override if heading_override is not None else self.current_position.heading
        heading_rad = math.radians(heading)

        # Calculate unit vector in heading direction
        dx = math.sin(heading_rad)
        dy = math.cos(heading_rad)

        # Find intersection with boundary
        # Check all four boundaries and find closest
        distances = []

        # North boundary (y = boundary_size)
        if dy > 0.001:
            t = (self.boundary_size - self.current_position.y) / dy
            if t > 0:
                distances.append(t)

        # South boundary (y = 0)
        if dy < -0.001:
            t = -self.current_position.y / dy
            if t > 0:
                distances.append(t)

        # East boundary (x = boundary_size)
        if dx > 0.001:
            t = (self.boundary_size - self.current_position.x) / dx
            if t > 0:
                distances.append(t)

        # West boundary (x = 0)
        if dx < -0.001:
            t = -self.current_position.x / dx
            if t > 0:
                distances.append(t)

        if distances:
            return min(distances) * 100.0  # Convert to cm
        else:
            return 100.0  # Large safe value if no boundary found

    def get_safe_heading(self):
        """
        Calculate a safe heading that points away from boundaries.

        Returns:
            Suggested safe heading in degrees
        """
        # Calculate center of boundary
        center_x = self.boundary_size / 2.0
        center_y = self.boundary_size / 2.0

        # Vector from current position to center
        to_center_x = center_x - self.current_position.x
        to_center_y = center_y - self.current_position.y

        # Calculate heading toward center
        if abs(to_center_x) < 0.001 and abs(to_center_y) < 0.001:
            # Already at center, return current heading
            return self.current_position.heading

        # Calculate angle to center
        heading_to_center = math.degrees(math.atan2(to_center_x, to_center_y))
        return heading_to_center % 360.0

    def is_near_boundary(self, threshold_cm=15.0):
        """
        Check if position is near any boundary.

        Args:
            threshold_cm: Distance threshold in centimeters

        Returns:
            True if near boundary, False otherwise
        """
        threshold_m = threshold_cm / 100.0

        return (self.current_position.x < threshold_m or
                self.current_position.x > (self.boundary_size - threshold_m) or
                self.current_position.y < threshold_m or
                self.current_position.y > (self.boundary_size - threshold_m))

    def get_boundary_proximity(self):
        """
        Get which boundary is closest and how far away it is.

        Returns:
            Tuple of (boundary_name, distance_cm)
            boundary_name is one of: "north", "south", "east", "west"
        """
        distances = {
            "north": (self.boundary_size - self.current_position.y) * 100.0,
            "south": self.current_position.y * 100.0,
            "east": (self.boundary_size - self.current_position.x) * 100.0,
            "west": self.current_position.x * 100.0
        }

        closest_boundary = min(distances.items(), key=lambda item: item[1])
        return closest_boundary[0], closest_boundary[1]

    def reset_to_center(self):
        """Reset position to center of boundary."""
        self.current_position = Position(
            self.boundary_size / 2.0,
            self.boundary_size / 2.0,
            self.current_position.heading  # Keep current heading
        )

    def get_position_tuple(self):
        """
        Get current position as tuple.

        Returns:
            Tuple of (x_meters, y_meters, heading_degrees)
        """
        return (
            self.current_position.x,
            self.current_position.y,
            self.current_position.heading
        )
