import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
from math import pi, floor

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

RIGHT_M = BP.PORT_B
LEFT_M = BP.PORT_C

EFFECTIVE_RADIUS = 2.9 # In cm
RAD_TO_DEG = 360 / (2*pi)
DEG_TO_RAD = (2*pi) / 360


def move_forward(cm: float):
    """
    cm - The goal distance to move.
    """
    goal_deg = floor((cm / EFFECTIVE_RADIUS) * RAD_TO_DEG)
    start_r = BP.get_motor_encoder(RIGHT_M)
    start_l = BP.get_motor_encoder(LEFT_M)
    print(f"Motor R status {BP.get_motor_status(RIGHT_M)} start {start_r} goal {start_r + goal_deg}")
    print(f"Motor L status {BP.get_motor_status(LEFT_M)} start {start_l} goal {start_l + goal_deg}")
    moved = 0
    while moved < goal_deg:
        try:
            now_r = BP.get_motor_encoder(RIGHT_M)
            now_l = BP.get_motor_encoder(LEFT_M)
        except IOError as error:
            print(error)

        print(f"Motor R status {BP.get_motor_status(RIGHT_M)} now {now_r}")
        print(f"Motor L status {BP.get_motor_status(LEFT_M)} now {now_l}")
        BP.set_motor_position(RIGHT_M, start_r + goal_deg)
        BP.set_motor_position(LEFT_M, start_l + goal_deg)
        print(f"Motor R status {BP.get_motor_status(RIGHT_M)} goal {start_r + goal_deg}")
        print(f"Motor L status {BP.get_motor_status(LEFT_M)} goal {start_l + goal_deg}")
        moved = (now_r - start_r + now_l - start_l) / 2
        time.sleep(0.02)

def rotate(deg: float):
    ...

try:
    try:
        BP.offset_motor_encoder(RIGHT_M, BP.get_motor_encoder(RIGHT_M)) # reset right motor 
        BP.offset_motor_encoder(LEFT_M, BP.get_motor_encoder(LEFT_M)) # reset left motor
    except IOError as error:
        print(error)

    #BP.set_motor_power(LEFT_M, BP.MOTOR_FLOAT)
    #BP.set_motor_power(RIGHT_M, BP.MOTOR_FLOAT)

    move_forward(40)
    print("Done")

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
except Exception as e:
    print(e)
finally:
    BP.reset_all()
