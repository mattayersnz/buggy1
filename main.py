import random
from time import sleep

import PicoAutonomousRobotics

# Initialize the robot
robot = PicoAutonomousRobotics.KitronikPicoRobotBuggy()

# Configuration constants
FORWARD_SPEED = 25  # Speed for forward movement (0-100) 25 is the lowest speed
TURN_SPEED = 20  # Speed for turning (0-100)
OBSTACLE_DISTANCE = 15  # Distance in cm to trigger obstacle avoidance

# Timing calibration (adjust these based on your robot's actual performance)
# Time to complete 4 wheel rotations at FORWARD_SPEED
TIME_FOR_4_ROTATIONS = 2.0  # seconds (you may need to adjust this)
# Time to turn 90 degrees at TURN_SPEED
TIME_FOR_90_DEGREE_TURN = 5.0  # seconds (you may need to adjust this)


def check_obstacle():
    """Check if there's an obstacle within OBSTACLE_DISTANCE cm"""
    distance = robot.getDistance("f")  # Get distance from front ultrasonic sensor
    if distance == -1:  # Sensor timeout or not fitted
        return False
    return distance < OBSTACLE_DISTANCE


def move_forward_4_rotations():
    """Move forward for 4 wheel rotations"""
    # Turn on both motors forward
    robot.motorOn("l", "f", FORWARD_SPEED)
    robot.motorOn("r", "f", FORWARD_SPEED)

    # Move for the calculated time
    sleep(TIME_FOR_4_ROTATIONS)

    # Stop motors
    robot.motorOff("l")
    robot.motorOff("r")


def turn_90_degrees():
    """Turn 90 degrees left or right randomly"""
    # Randomly choose left or right (50/50)
    turn_direction = random.choice(["left", "right"])

    if turn_direction == "left":
        # Turn left: left motor reverse, right motor forward
        robot.motorOn("l", "r", TURN_SPEED)
        robot.motorOn("r", "f", TURN_SPEED)
        # Flash left LEDs blue
        robot.setLED(0, robot.BLUE)
        robot.setLED(1, robot.BLUE)
    else:
        # Turn right: left motor forward, right motor reverse
        robot.motorOn("l", "f", TURN_SPEED)
        robot.motorOn("r", "r", TURN_SPEED)
        # Flash right LEDs blue
        robot.setLED(2, robot.BLUE)
        robot.setLED(3, robot.BLUE)

    robot.show()

    # Turn for the calculated time
    sleep(TIME_FOR_90_DEGREE_TURN)

    # Stop motors
    robot.motorOff("l")
    robot.motorOff("r")

    # Clear LEDs
    robot.clear(0)
    robot.clear(1)
    robot.clear(2)
    robot.clear(3)
    robot.show()

    # Brief pause after turn
    sleep(0.2)


def main():
    """Main robot control loop"""
    print("Robot starting...")
    robot.beepHorn()  # Signal start

    # Set LEDs to green to show robot is running
    for i in range(4):
        robot.setLED(i, robot.GREEN)
    robot.show()

    sleep(1)  # Give a second before starting

    try:
        while True:
            # Check for obstacles before moving
            if check_obstacle():
                print("Obstacle detected!")
                # Set LEDs to red
                for i in range(4):
                    robot.setLED(i, robot.RED)
                robot.show()

                # Stop and turn
                robot.motorOff("l")
                robot.motorOff("r")
                sleep(0.5)
                turn_90_degrees()

                # Set LEDs back to green
                for i in range(4):
                    robot.setLED(i, robot.GREEN)
                robot.show()

                # Continue to next iteration (will check obstacle again)
                continue

            # No obstacle - execute normal movement pattern
            print("Moving forward 4 rotations...")
            move_forward_4_rotations()

            print("Stopping and turning...")
            sleep(0.3)  # Brief pause
            turn_90_degrees()

    except KeyboardInterrupt:
        # Clean shutdown on Ctrl+C
        print("\nStopping robot...")
        robot.motorOff("l")
        robot.motorOff("r")
        for i in range(4):
            robot.clear(i)
        robot.show()
        print("Robot stopped.")


# Run the main program
if __name__ == "__main__":
    main()
