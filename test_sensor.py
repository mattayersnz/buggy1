"""Quick sensor test - upload and run this to check sensor readings"""
import sys
sys.path.append("Kitronik-Pico-Autonomous-Robotics-Platform-MicroPython")
import PicoAutonomousRobotics
import time

buggy = PicoAutonomousRobotics.KitronikPicoRobotBuggy()

print("Distance Sensor Test")
print("Reading front sensor 20 times...")
print("-" * 40)

for i in range(20):
    dist = buggy.getDistance("f")
    if dist == -1:
        print(f"{i+1}: No detection (sensor returned -1)")
    else:
        print(f"{i+1}: {dist:.1f} cm")
    time.sleep(0.3)

print("-" * 40)
print("Test complete")
