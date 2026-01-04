"""
Box-Following Robot

A simple state machine that uses a distance sensor to find and follow a box.

States:
- SEARCHING (Red LEDs): Spins to find a box within 30cm
- FOUND (Green LEDs): Stops when box is detected, waits for movement
- TRACKING (Blue LEDs): Follows the box, maintaining 15-25cm distance
"""

import time
import sys

# Import Kitronik library
sys.path.append("Kitronik-Pico-Autonomous-Robotics-Platform-MicroPython")
import PicoAutonomousRobotics

# Import calibration constants
from calibration_data import CalibrationData


class State:
    """Robot state enumeration."""
    SEARCHING = "searching"
    FOUND = "found"
    TRACKING = "tracking"


# LED color constants
LED_RED = (255, 0, 0)      # Searching state
LED_GREEN = (0, 255, 0)    # Found state
LED_BLUE = (0, 0, 255)     # Tracking state


def set_all_leds(buggy, color):
    """Set all 4 LEDs to the specified color."""
    for i in range(4):
        buggy.setLED(i, color)
    buggy.show()


def stop_motors(buggy):
    """Stop both motors."""
    buggy.motorOff("l")
    buggy.motorOff("r")


def get_filtered_distance(buggy, samples=3):
    """
    Read distance sensor multiple times and return median value.
    This filters out spurious readings and electrical noise.

    Args:
        buggy: Robot hardware interface
        samples: Number of readings to take (default 3)

    Returns:
        Filtered distance in cm, or -1 if no valid detection
    """
    readings = []
    for _ in range(samples):
        dist = buggy.getDistance("f")
        if dist > 0:  # Only include valid readings
            readings.append(dist)
        time.sleep(0.02)  # Small delay between readings

    if not readings:
        return -1  # No valid readings

    # Return median value to filter outliers
    readings.sort()
    return readings[len(readings) // 2]


def spin_to_search(buggy):
    """Start spinning in place to search for box."""
    buggy.motorOn("l", "f", CalibrationData.TURN_SPEED)
    buggy.motorOn("r", "r", CalibrationData.TURN_SPEED)


def move_forward_one_step(buggy):
    """Move forward by 1/3 wheel turn (~4cm)."""
    buggy.motorOn("l", "f", CalibrationData.FORWARD_SPEED)
    buggy.motorOn("r", "f", CalibrationData.FORWARD_SPEED)
    time.sleep(CalibrationData.FORWARD_STEP_DURATION)
    stop_motors(buggy)
    time.sleep(0.05)  # Brief pause for stability


def turn_angle(buggy, degrees):
    """
    Turn by approximate angle.

    Args:
        buggy: Robot hardware interface
        degrees: Angle to turn (positive=right, negative=left)
    """
    # From calibration: 90° in 5 seconds at speed 20
    # So 1° = 5/90 = 0.0556 seconds
    duration = abs(degrees) * 0.0556

    if degrees > 0:  # Turn right
        buggy.motorOn("l", "f", CalibrationData.TURN_SPEED)
        buggy.motorOn("r", "r", CalibrationData.TURN_SPEED)
    else:  # Turn left
        buggy.motorOn("l", "r", CalibrationData.TURN_SPEED)
        buggy.motorOn("r", "f", CalibrationData.TURN_SPEED)

    time.sleep(duration)
    stop_motors(buggy)
    time.sleep(0.05)  # Brief pause for stability


def transition_to_state(buggy, new_state, current_state):
    """
    Handle state transition with appropriate setup.

    Args:
        buggy: Robot hardware interface
        new_state: State to transition to
        current_state: Current state (for logging)

    Returns:
        The new state
    """
    if new_state == current_state:
        return new_state

    print(f"State: {current_state} -> {new_state}")
    stop_motors(buggy)

    if new_state == State.SEARCHING:
        # Don't change LEDs or motors here - let the state logic handle it
        # based on search_mode
        pass
    elif new_state == State.FOUND:
        set_all_leds(buggy, LED_GREEN)
        buggy.beepHorn()
    elif new_state == State.TRACKING:
        set_all_leds(buggy, LED_BLUE)

    return new_state


def main():
    """Main control loop for box-following robot."""
    print("Box-Following Robot Starting...")

    # Initialize hardware
    buggy = PicoAutonomousRobotics.KitronikPicoRobotBuggy()

    # Initialize state machine
    current_state = State.SEARCHING
    search_mode = "wiggle"  # Start with soft search
    set_all_leds(buggy, LED_BLUE)  # Blue LEDs for wiggle search
    print(f"State: {current_state} (wiggle mode)")

    # State-specific variables
    last_distance = 0.0
    last_tracking_distance = 0.0  # Tracks last known good distance in TRACKING state
    beep_counter = 0
    recovery_mode = False
    recovery_attempts = 0
    search_mode = "wiggle"  # Can be "wiggle" (blue) or "spin" (red)

    try:
        while True:
            # Read distance sensor
            box_distance = buggy.getDistance("f")

            # ===== SEARCHING STATE =====
            # Two-phase search: wiggle first, then spin
            if current_state == State.SEARCHING:
                if search_mode == "wiggle":
                    # PHASE 1: Soft search - wiggle to look around (Blue LEDs)
                    set_all_leds(buggy, LED_BLUE)

                    # Check center first
                    box_distance = buggy.getDistance("f")
                    if box_distance > 0 and box_distance <= CalibrationData.BOX_DETECTION_DISTANCE:
                        print(f"Box found at center: {box_distance:.1f}cm!")
                        search_mode = "wiggle"  # Reset for next time
                        current_state = transition_to_state(buggy, State.FOUND, current_state)
                        last_distance = box_distance
                        continue

                    # Wiggle left 45° and check
                    turn_angle(buggy, -CalibrationData.SCAN_WIGGLE_ANGLE)
                    box_distance = buggy.getDistance("f")
                    if box_distance > 0 and box_distance <= CalibrationData.BOX_DETECTION_DISTANCE:
                        print(f"Box found on left: {box_distance:.1f}cm!")
                        turn_angle(buggy, CalibrationData.SCAN_WIGGLE_ANGLE)  # Return to center
                        search_mode = "wiggle"  # Reset for next time
                        current_state = transition_to_state(buggy, State.FOUND, current_state)
                        last_distance = box_distance
                        continue

                    # Wiggle right 90° and check
                    turn_angle(buggy, CalibrationData.SCAN_WIGGLE_ANGLE * 2)
                    box_distance = buggy.getDistance("f")
                    if box_distance > 0 and box_distance <= CalibrationData.BOX_DETECTION_DISTANCE:
                        print(f"Box found on right: {box_distance:.1f}cm!")
                        turn_angle(buggy, -CalibrationData.SCAN_WIGGLE_ANGLE)  # Return to center
                        search_mode = "wiggle"  # Reset for next time
                        current_state = transition_to_state(buggy, State.FOUND, current_state)
                        last_distance = box_distance
                        continue

                    # Return to center
                    turn_angle(buggy, -CalibrationData.SCAN_WIGGLE_ANGLE)

                    # Wiggle search failed - escalate to spin mode
                    print("Wiggle search failed - starting spin search")
                    search_mode = "spin"
                    set_all_leds(buggy, LED_RED)
                    spin_to_search(buggy)

                elif search_mode == "spin":
                    # PHASE 2: Hard search - full spin (Red LEDs)
                    set_all_leds(buggy, LED_RED)
                    # Already spinning from mode transition
                    if box_distance > 0 and box_distance <= CalibrationData.BOX_DETECTION_DISTANCE:
                        print(f"Box found during spin: {box_distance:.1f}cm!")
                        search_mode = "wiggle"  # Reset for next time
                        current_state = transition_to_state(buggy, State.FOUND, current_state)
                        last_distance = box_distance

            # ===== FOUND STATE =====
            # Robot has found box and waits for it to move
            elif current_state == State.FOUND:
                # Check if box is lost (out of range or no detection)
                if box_distance == -1 or box_distance > CalibrationData.BOX_DETECTION_DISTANCE:
                    print("Box lost! Searching...")
                    current_state = transition_to_state(buggy, State.SEARCHING, current_state)

                # Check if box moved significantly
                elif abs(box_distance - last_distance) >= CalibrationData.DISTANCE_CHANGE_THRESHOLD:
                    print(f"Box moved! Tracking... (was {last_distance:.1f}cm, now {box_distance:.1f}cm)")
                    current_state = transition_to_state(buggy, State.TRACKING, current_state)
                    last_tracking_distance = box_distance  # Initialize tracking distance
                    beep_counter = 0

                # Box still present and stationary - update reference distance
                else:
                    last_distance = box_distance

            # ===== TRACKING STATE =====
            # Robot follows the box using smart distance-based tracking
            elif current_state == State.TRACKING:
                # Use filtered reading for more reliable tracking
                box_distance = get_filtered_distance(buggy)

                # Check if box is in range
                if box_distance > 0 and box_distance <= CalibrationData.BOX_DETECTION_DISTANCE:
                    # Box is in range - exit recovery mode if active
                    if recovery_mode:
                        recovery_mode = False
                        recovery_attempts = 0
                        print("Box reacquired - resuming tracking")

                    # Check if distance increased significantly (box moved away)
                    if box_distance > last_tracking_distance + 3:
                        print(f"Box moved away (was {last_tracking_distance:.1f}cm, now {box_distance:.1f}cm) - moving forward")
                        move_forward_one_step(buggy)
                        # Only update tracking distance if we get a valid reading
                        new_distance = get_filtered_distance(buggy)
                        if new_distance > 0 and new_distance <= CalibrationData.BOX_DETECTION_DISTANCE:
                            last_tracking_distance = new_distance

                    # Box too close - back up slowly
                    elif box_distance < CalibrationData.BOX_TOO_CLOSE_DISTANCE:
                        buggy.motorOn("l", "r", CalibrationData.REVERSE_SPEED)
                        buggy.motorOn("r", "r", CalibrationData.REVERSE_SPEED)
                        last_tracking_distance = box_distance

                    # Box in optimal range - stop and maintain position
                    elif CalibrationData.BOX_OPTIMAL_MIN <= box_distance <= CalibrationData.BOX_OPTIMAL_MAX:
                        stop_motors(buggy)
                        last_tracking_distance = box_distance

                    # Box at edge of range (25-30cm) - move forward
                    elif box_distance > CalibrationData.BOX_OPTIMAL_MAX:
                        move_forward_one_step(buggy)
                        # Only update tracking distance if we get a valid reading
                        new_distance = get_filtered_distance(buggy)
                        if new_distance > 0 and new_distance <= CalibrationData.BOX_DETECTION_DISTANCE:
                            last_tracking_distance = new_distance

                else:
                    # Box lost - try wiggling to reacquire
                    if not recovery_mode:
                        recovery_mode = True
                        recovery_attempts = 0
                        print("Box lost - wiggling to find it")

                    if recovery_attempts < 3:  # Try 3 wiggle attempts
                        # Wiggle left
                        turn_angle(buggy, -CalibrationData.SCAN_WIGGLE_ANGLE)
                        box_distance = get_filtered_distance(buggy)
                        if box_distance > 0 and box_distance <= CalibrationData.BOX_DETECTION_DISTANCE:
                            print(f"Found via left wiggle! at {box_distance:.1f}cm")
                            turn_angle(buggy, CalibrationData.SCAN_WIGGLE_ANGLE)  # Return to center
                            recovery_mode = False
                            last_tracking_distance = box_distance
                            continue

                        # Wiggle right (90° from left position)
                        turn_angle(buggy, CalibrationData.SCAN_WIGGLE_ANGLE * 2)
                        box_distance = get_filtered_distance(buggy)
                        if box_distance > 0 and box_distance <= CalibrationData.BOX_DETECTION_DISTANCE:
                            print(f"Found via right wiggle! at {box_distance:.1f}cm")
                            turn_angle(buggy, -CalibrationData.SCAN_WIGGLE_ANGLE)  # Return to center
                            recovery_mode = False
                            last_tracking_distance = box_distance
                            continue

                        # Return to center
                        turn_angle(buggy, -CalibrationData.SCAN_WIGGLE_ANGLE)
                        recovery_attempts += 1
                        continue

                    else:
                        # Wiggling failed - return to SEARCHING (wiggle mode)
                        recovery_mode = False
                        recovery_attempts = 0
                        search_mode = "wiggle"  # Start with soft search
                        print("Wiggling failed - returning to wiggle search mode")
                        current_state = transition_to_state(buggy, State.SEARCHING, current_state)
                        continue

                # Periodic beeping to indicate tracking state
                beep_counter += 1
                if beep_counter % CalibrationData.TRACKING_BEEP_INTERVAL == 0:
                    buggy.soundFrequency(CalibrationData.TRACKING_BEEP_FREQ)
                    time.sleep(CalibrationData.TRACKING_BEEP_DURATION)
                    buggy.silence()

            # Loop delay (100ms cycle time)
            time.sleep(0.1)

    except KeyboardInterrupt:
        # Graceful shutdown on Ctrl+C
        print("\nShutting down...")
        stop_motors(buggy)
        set_all_leds(buggy, (0, 0, 0))  # Turn off all LEDs
        buggy.silence()
        print("Stopped.")


# Run the main program
if __name__ == "__main__":
    main()
