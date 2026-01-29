import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
from math import pi, floor

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

RIGHT_M = BP.PORT_C
LEFT_M = BP.PORT_B
TOUCH_S = BP.PORT_1

EFFECTIVE_RADIUS = 2.9 # In cm
WHEEL_SEPARATION = 15.4 # In cm
RAD_TO_DEG = 360 / (2*pi)
DEG_TO_RAD = (2*pi) / 360
EPSILON = 5 #degrees
ROTATE_CONST = 1.14# 1.13
DIST_CONST = 40/(40-2.64)# 40 / (40 - 2.6)
MAX_POWER = 35
LEFT_CONST = 1.05#1.1275
RIGHT_CONST = 1#1.05

def move(cm: float):
    """
    cm - The goal distance to move.
    """
    goal_deg = floor(DIST_CONST * (cm / EFFECTIVE_RADIUS) * RAD_TO_DEG)
    start_r = BP.get_motor_encoder(RIGHT_M)
    start_l = BP.get_motor_encoder(LEFT_M)
    print(f"Motor R status {BP.get_motor_status(RIGHT_M)} start {start_r} goal {start_r + goal_deg}")
    print(f"Motor L status {BP.get_motor_status(LEFT_M)} start {start_l} goal {start_l + goal_deg}")
    moved = 0
    BP.set_motor_position(RIGHT_M, start_r + goal_deg)
    BP.set_motor_position(LEFT_M, start_l + goal_deg)
    while abs(moved - goal_deg) > EPSILON:
        try:
            now_r = BP.get_motor_encoder(RIGHT_M)
            now_l = BP.get_motor_encoder(LEFT_M)
        except IOError as error:
            print(error)

        print(f"Motor R status {BP.get_motor_status(RIGHT_M)} now {now_r}")
        print(f"Motor L status {BP.get_motor_status(LEFT_M)} now {now_l}")
        moved = (now_r - start_r + now_l - start_l) / 2
        time.sleep(0.05)


def rotate(deg: float):
    """
    deg - The goal degrees to rotate.
    """
    rad = deg * DEG_TO_RAD
    wheel_dist = rad * (WHEEL_SEPARATION / 2)
    goal_deg = -floor(ROTATE_CONST * (wheel_dist / EFFECTIVE_RADIUS) * RAD_TO_DEG)
    start_r = BP.get_motor_encoder(RIGHT_M)
    start_l = BP.get_motor_encoder(LEFT_M)
    print(f"Motor R status {BP.get_motor_status(RIGHT_M)} start {start_r} goal {start_r + goal_deg}")
    print(f"Motor L status {BP.get_motor_status(LEFT_M)} start {start_l} goal {start_l + goal_deg}")
    moved = 0
    BP.set_motor_position(RIGHT_M, start_r - goal_deg)
    BP.set_motor_position(LEFT_M, start_l + goal_deg)
    while moved > goal_deg + EPSILON:
        try:
            now_r = BP.get_motor_encoder(RIGHT_M)
            now_l = BP.get_motor_encoder(LEFT_M)
        except IOError as error:
            print(error)

        print(f"Motor R status {BP.get_motor_status(RIGHT_M)} now {now_r}")
        print(f"Motor L status {BP.get_motor_status(LEFT_M)} now {now_l}")
        moved = (-now_r + start_r - start_l + now_l) / 2
        time.sleep(0.05)
    

try:
    try:
        BP.offset_motor_encoder(RIGHT_M, BP.get_motor_encoder(RIGHT_M)) # reset right motor 
        BP.offset_motor_encoder(LEFT_M, BP.get_motor_encoder(LEFT_M)) # reset left motor
        BP.set_sensor_type(TOUCH_S, BP.SENSOR_TYPE.TOUCH)
    except IOError as error:
        print(error)

    #BP.set_motor_power(LEFT_M, BP.MOTOR_FLOAT)
    #BP.set_motor_power(RIGHT_M, BP.MOTOR_FLOAT)
    BP.set_motor_limits(LEFT_M, LEFT_CONST * MAX_POWER)
    BP.set_motor_limits(RIGHT_M, RIGHT_CONST * MAX_POWER)
    
    last_value = 0
    while True:
        try:
            value = BP.get_sensor(BP.PORT_1)
            print(value)
        except brickpi3.SensorError as error:
            print(error)
            value = 0

        if value == 1:
            move(-10)
        elif last_value == 1:
             move(10)
        time.sleep(0.02)
        last_value = value
     
    print("Done")

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
except Exception as e:
    print(e)
finally:
    BP.reset_all()
