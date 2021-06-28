from dynamixel_sdk import *
import numpy as np
from leg_class import LEG
from hexapod_class import HEXAPOD

# Protocol version
BAUDRATE = 57600  # Dynamixel default baudrate : 57600
DEVICENAME1 = 'COM9'  # Check which port is being used on your controller
DEVICENAME2 = 'COM10'  # Check which port is being used on your controller

portHandler1 = PortHandler(DEVICENAME1)
portHandler2 = PortHandler(DEVICENAME2)

# Open port num1
if portHandler1.openPort():
    print("Succeeded to open the first port")
else:
    print("Failed to open the first port")
    print("Press any key to terminate...")
    quit()

# Open port num2
if portHandler2.openPort():
    print("Succeeded to open the second port")
else:
    print("Failed to open the second port")
    print("Press any key to terminate...")
    quit()

# Set ports baudrate
if portHandler1.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate #1")
else:
    print("Failed to change the baudrate #1")
    print("Press any key to terminate...")
    quit()

if portHandler2.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate #2")
else:
    print("Failed to change the baudrate #2")
    print("Press any key to terminate...")
    quit()

# leg objects initializations
# NOTE: leg side should be defined as 'left' or 'right' only. position should be 'forward', 'middle' or 'back' only.

leg1 = LEG(portHandler1, [11, 12, 13], 'left', 'front')
leg2 = LEG(portHandler1, [21, 22, 23], 'left', 'middle')
leg3 = LEG(portHandler1, [31, 32, 33], 'left', 'back')
leg4 = LEG(portHandler2, [41, 42, 43], 'right', 'front')
leg5 = LEG(portHandler2, [51, 52, 53], 'right', 'middle')
leg6 = LEG(portHandler2, [61, 62, 63], 'right', 'back')

legs = [leg1, leg2, leg3, leg4, leg5, leg6]

hexapod = HEXAPOD(legs)

# walk inputs: direction, height (y), width (z), step size, resolution (1/speed), step height, step distance, number of steps
hexapod.walk('forward', 200, 250, 150, 5, 50, 10, 3)
hexapod.roll(45, 2)
hexapod.roll(-45, 2)
hexapod.pitch(40, 2)
hexapod.pitch(-40, 2)
# hexapod.yaw(30,2)
# hexapod.yaw(-30,2)
hexapod.spin()
hexapod.walk('forward', 200, 250, 150, 5, 100, 20, 3)
# hexapod.walk('backwards', 200, 250, 150, 5, 100, 20, 1)
# hexapod.spin()


# Close ports
portHandler1.closePort()
portHandler2.closePort()