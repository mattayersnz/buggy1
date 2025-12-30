"""
Curious Creature - Autonomous Buggy Controller

Makes the Raspberry Pi Pico buggy behave like a curious, living creature
that roams within a 1-meter square area with dynamic moods and lifelike behaviors.
"""

import time
import sys

# Import Kitronik library
sys.path.append("Kitronik-Pico-Autonomous-Robotics-Platform-MicroPython")
import PicoAutonomousRobotics

# Import creature modules
from creature_position import PositionTracker
from creature_mood import MoodManager, Mood
from creature_behavior import BehaviorController


def main():
    """Main creature control loop."""
    print("Creature awakening...")

    # Initialize subsystems
    robot = PicoAutonomousRobotics.KitronikPicoRobotBuggy()
    position_tracker = PositionTracker()
    mood_manager = MoodManager(start_mood=Mood.CURIOUS)
    behavior = BehaviorController(robot, position_tracker)

    # Startup animation (creature "wakes up")
    behavior.startup_animation()
    time.sleep(1)

    print("Creature is alive!")
    print(f"Starting position: {position_tracker.current_position}")

    try:
        while True:
            # Update mood system (check for transitions)
            current_mood_config = mood_manager.update()

            # Apply mood-based LED color
            behavior.set_all_leds(current_mood_config.led_color)

            # Check for obstacles first
            if behavior.check_and_handle_obstacle(current_mood_config):
                # Object was detected and investigated
                mood_manager.log_event("object_encounter")
                continue

            # Check for boundaries
            if behavior.check_and_handle_boundary(current_mood_config):
                # Boundary was detected and handled
                mood_manager.log_event("boundary_encounter")
                continue

            # Normal wandering behavior

            # Random pause check
            if mood_manager.should_pause():
                pause_duration = mood_manager.get_pause_duration()
                behavior.idle_observe(pause_duration, current_mood_config)

            # Random turn and occasional sound
            behavior.random_wander_decision(current_mood_config)

            # Continue wandering
            behavior.wander_step(current_mood_config)

            # Small delay between decision cycles
            time.sleep(0.1)

    except KeyboardInterrupt:
        # Graceful shutdown on Ctrl+C
        print("\nCreature going to sleep...")
        behavior.shutdown_sequence()
        print("Creature is asleep.")
        print(f"Final position: {position_tracker.current_position}")


# Run the main program
if __name__ == "__main__":
    main()
