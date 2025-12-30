"""
Behavior controller for creature-like movements.

This module implements movement primitives and complex behaviors
that make the buggy feel alive and autonomous.
"""

import time
import random
from typing import Tuple, Optional
from calibration_data import CalibrationData
from creature_position import PositionTracker
from creature_mood import MoodConfig


class BehaviorController:
    """
    Controls movement primitives and complex behaviors.

    Integrates with robot hardware, position tracker, and mood system
    to create lifelike autonomous behavior.
    """

    def __init__(self, robot, position_tracker: PositionTracker):
        """
        Initialize behavior controller.

        Args:
            robot: Instance of KitronikPicoRobotBuggy
            position_tracker: Instance of PositionTracker
        """
        self.robot = robot
        self.position_tracker = position_tracker

    def move_forward(self, distance_cm: float, speed: int) -> None:
        """
        Move forward a specified distance.

        Args:
            distance_cm: Distance to move in centimeters
            speed: Motor speed (0-100)
        """
        duration = CalibrationData.calculate_movement_time(distance_cm, speed)

        self.robot.motorOn("l", "f", speed)
        self.robot.motorOn("r", "f", speed)
        time.sleep(duration)
        self.robot.motorOff("l")
        self.robot.motorOff("r")

        # Update position
        self.position_tracker.update_position(distance_cm)

    def turn(self, angle_degrees: float, direction: str) -> None:
        """
        Turn by specified angle.

        Args:
            angle_degrees: Angle to turn in degrees
            direction: "left" or "right"
        """
        duration = CalibrationData.calculate_turn_time(angle_degrees)
        speed = CalibrationData.TURN_SPEED

        if direction == "left":
            self.robot.motorOn("l", "r", speed)
            self.robot.motorOn("r", "f", speed)
            angle_change = -angle_degrees  # Counter-clockwise
        else:
            self.robot.motorOn("l", "f", speed)
            self.robot.motorOn("r", "r", speed)
            angle_change = angle_degrees  # Clockwise

        time.sleep(duration)
        self.robot.motorOff("l")
        self.robot.motorOff("r")

        # Update heading
        self.position_tracker.update_heading(angle_change)

    def back_away(self, distance_cm: float) -> None:
        """
        Move backward a specified distance.

        Checks rear sensor for safety.

        Args:
            distance_cm: Distance to move backward in centimeters
        """
        # Check rear sensor
        rear_distance = self.robot.getDistance("r")
        if rear_distance > 0:
            safe_distance = min(distance_cm, rear_distance - CalibrationData.MIN_SAFE_REAR_DISTANCE)
        else:
            safe_distance = distance_cm

        if safe_distance <= 0:
            return  # Not safe to reverse

        speed = CalibrationData.REVERSE_SPEED
        duration = CalibrationData.calculate_movement_time(safe_distance, speed)

        self.robot.motorOn("l", "r", speed)
        self.robot.motorOn("r", "r", speed)
        time.sleep(duration)
        self.robot.motorOff("l")
        self.robot.motorOff("r")

        # Update position (negative distance for reverse)
        self.position_tracker.update_position(-safe_distance)

    def pivot_inspect(self, angle: float = 45.0) -> None:
        """
        Perform curious side-to-side inspection pivot.

        Args:
            angle: Angle to pivot each direction (degrees)
        """
        self.turn(angle, "left")
        time.sleep(0.3)
        self.turn(angle, "right")
        time.sleep(0.3)
        self.turn(angle / 2.0, "left")  # Return to approximate center

    def wander_step(self, mood_config: MoodConfig) -> None:
        """
        Take a random wander step based on mood.

        Args:
            mood_config: Current mood configuration
        """
        speed = random.randint(*mood_config.speed_range)
        distance = random.uniform(10.0, 30.0)  # 10-30cm steps

        # Check if movement is safe (boundary check)
        if not self.position_tracker.is_movement_safe(distance):
            # Movement would exceed boundary - don't move
            return

        self.move_forward(distance, speed)

    def idle_observe(self, duration: float, mood_config: MoodConfig) -> None:
        """
        Stop and observe with LED breathing effect.

        Args:
            duration: How long to observe in seconds
            mood_config: Current mood configuration
        """
        self.robot.motorOff("l")
        self.robot.motorOff("r")

        if not mood_config.led_breathing:
            # Just wait without breathing
            time.sleep(duration)
            return

        # Breathing LED effect
        start_time = time.time()
        cycle_duration = 2.0  # 2 second breathing cycle

        while time.time() - start_time < duration:
            elapsed = time.time() - start_time
            phase = (elapsed % cycle_duration) / cycle_duration

            # Sinusoidal breathing (smoother than linear)
            if phase < 0.5:
                # Fade in
                brightness = 20 + int(60 * (phase * 2))
            else:
                # Fade out
                brightness = 80 - int(60 * ((phase - 0.5) * 2))

            self.robot.setBrightness(brightness)
            self.robot.show()
            time.sleep(0.05)

        # Reset to normal brightness
        self.robot.setBrightness(20)
        self.robot.show()

    def play_curious_sound(self, mood_config: MoodConfig) -> None:
        """
        Play mood-appropriate sound sequence.

        Args:
            mood_config: Current mood configuration
        """
        pattern = mood_config.sound_pattern

        for _ in range(pattern.beep_count):
            freq = pattern.base_frequency + random.randint(
                -pattern.frequency_variation,
                pattern.frequency_variation
            )
            self.robot.soundFrequency(freq)
            time.sleep(pattern.beep_duration)
            self.robot.silence()
            time.sleep(pattern.beep_gap)

    def investigate_object(self, initial_distance: float, mood_config: MoodConfig) -> None:
        """
        Full investigation sequence: approach, observe, vocalize, retreat, navigate.

        Args:
            initial_distance: Initial distance to object in centimeters
            mood_config: Current mood configuration
        """
        # Phase 1: Cautious approach
        approach_distance = initial_distance - mood_config.investigation_distance
        if approach_distance > 2.0:  # Only approach if there's meaningful distance
            slow_speed = mood_config.speed_range[0]  # Use slow end of mood range
            self.move_forward(approach_distance, slow_speed)

        # Phase 2: Pause and observe
        observe_duration = random.uniform(1.0, 2.5)
        self.idle_observe(observe_duration, mood_config)

        # Phase 3: Inquisitive sound
        self.play_curious_sound(mood_config)

        # Phase 4: Pivot to inspect from different angle
        self.pivot_inspect(angle=30.0)

        # Phase 5: Back away
        self.back_away(15.0)

        # Phase 6: Navigate around (turn and move)
        turn_angle = random.choice([60, 90, 120])
        turn_direction = random.choice(["left", "right"])
        self.turn(turn_angle, turn_direction)

        # Try to continue wandering after investigation
        if self.position_tracker.is_movement_safe(20.0):
            self.wander_step(mood_config)

    def execute_boundary_response(self, mood_config: MoodConfig) -> None:
        """
        Respond to boundary detection with cautious behavior.

        Args:
            mood_config: Current mood configuration
        """
        # Express concern - set LEDs to purple
        self.set_all_leds((180, 0, 255))  # Purple

        # Play cautious sound
        self.robot.soundFrequency(300)
        time.sleep(0.2)
        self.robot.silence()

        # Back away slightly
        self.back_away(5.0)

        # Turn toward safe direction (away from boundary)
        safe_heading = self.position_tracker.get_safe_heading()
        current_heading = self.position_tracker.current_position.heading

        # Calculate angle difference
        angle_diff = safe_heading - current_heading

        # Normalize to -180 to 180
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360

        # Turn toward safe heading
        if abs(angle_diff) > 5:  # Only turn if meaningful difference
            direction = "left" if angle_diff < 0 else "right"
            self.turn(abs(angle_diff), direction)

        # Restore mood-based LED color
        self.set_all_leds(mood_config.led_color)

    def set_all_leds(self, color: Tuple[int, int, int]) -> None:
        """
        Set all LEDs to the same color.

        Args:
            color: RGB tuple (0-255 each)
        """
        for led in range(4):
            self.robot.setLED(led, color)
        self.robot.show()

    def startup_animation(self) -> None:
        """Play startup animation when creature 'wakes up'."""
        # Rainbow sequence
        colors = [
            (255, 0, 0),    # Red
            (255, 150, 0),  # Orange
            (255, 255, 0),  # Yellow
            (0, 255, 0),    # Green
            (0, 255, 255),  # Cyan
            (0, 0, 255),    # Blue
            (180, 0, 255)   # Purple
        ]

        for color in colors:
            self.set_all_leds(color)
            time.sleep(0.2)

        # Beep horn
        self.robot.beepHorn()

        # Set to initial mood color (will be set by mood manager)
        self.robot.clear()
        self.robot.show()

    def shutdown_sequence(self) -> None:
        """Graceful shutdown when creature 'goes to sleep'."""
        # Stop motors
        self.robot.motorOff("l")
        self.robot.motorOff("r")

        # Fade out LEDs
        for brightness in range(100, 0, -10):
            self.robot.setBrightness(brightness)
            self.robot.show()
            time.sleep(0.05)

        # Clear LEDs
        self.robot.clear()
        self.robot.show()
        self.robot.silence()

    def random_wander_decision(self, mood_config: MoodConfig) -> None:
        """
        Make random wandering decisions based on mood.

        Args:
            mood_config: Current mood configuration
        """
        # Random turn check
        if random.random() < mood_config.turn_bias:
            angle = random.choice([30, 45, 60, 90])
            direction = random.choice(["left", "right"])
            self.turn(angle, direction)

            # Occasional sound during random turn
            if random.random() < 0.3:
                self.play_curious_sound(mood_config)

    def check_and_handle_obstacle(self, mood_config: MoodConfig) -> bool:
        """
        Check for obstacles and handle if detected.

        Args:
            mood_config: Current mood configuration

        Returns:
            True if obstacle was detected and handled, False otherwise
        """
        front_distance = self.robot.getDistance("f")

        if front_distance > 0 and front_distance < CalibrationData.OBSTACLE_DETECTION_DISTANCE:
            # Object detected - investigate!
            self.investigate_object(front_distance, mood_config)
            return True

        return False

    def check_and_handle_boundary(self, mood_config: MoodConfig) -> bool:
        """
        Check for boundaries and handle if near edge.

        Args:
            mood_config: Current mood configuration

        Returns:
            True if boundary was detected and handled, False otherwise
        """
        # Check if next wander step would be safe
        if not self.position_tracker.is_movement_safe(20.0):
            self.execute_boundary_response(mood_config)
            return True

        return False
