import sys
sys.path.append("Kitronik-Pico-Autonomous-Robotics-Platform-MicroPython")
import PicoAutonomousRobotics
import time

buggy = PicoAutonomousRobotics.KitronikPicoRobotBuggy()
print("Testing sensor for 5 seconds...")
for i in range(50):
    dist = buggy.getDistance("f")
    print(f"{i}: {dist:.1f} cm" if dist > 0 else f"{i}: No detection")
    time.sleep(0.1)
