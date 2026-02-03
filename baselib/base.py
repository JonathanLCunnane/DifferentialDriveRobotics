import time
import brickpi3
from math import pi, floor

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

RIGHT_M = BP.PORT_C
LEFT_M = BP.PORT_B

EFFECTIVE_RADIUS = 2.9 # In cm
WHEEL_SEPARATION = 15.4 # In cm
RAD_TO_DEG = 360 / (2*pi)
DEG_TO_RAD = (2*pi) / 360
EPSILON = 5 #degrees
ROTATE_CONST = 1.15# 1.13
DIST_CONST = 40/(40-2.8)# 40 / (40 - 2.6)
MAX_POWER = 35
LEFT_CONST = 1.08#1.1275
RIGHT_CONST = 1#1.05

def move_forward(
    cm: float,
    dist_const=DIST_CONST,
    effective_radius=EFFECTIVE_RADIUS,
):
    """
    cm - The goal distance to move.
    """
    goal_deg = floor(dist_const * (cm / effective_radius) * RAD_TO_DEG)
    start_r = BP.get_motor_encoder(RIGHT_M)
    start_l = BP.get_motor_encoder(LEFT_M)
    print(f"Motor R status {BP.get_motor_status(RIGHT_M)} start {start_r} goal {start_r + goal_deg}")
    print(f"Motor L status {BP.get_motor_status(LEFT_M)} start {start_l} goal {start_l + goal_deg}")
    moved = 0
    BP.set_motor_position(RIGHT_M, start_r + goal_deg)
    BP.set_motor_position(LEFT_M, start_l + goal_deg)
    while moved < goal_deg - EPSILON:
        try:
            now_r = BP.get_motor_encoder(RIGHT_M)
            now_l = BP.get_motor_encoder(LEFT_M)
        except IOError as error:
            print(error)

        print(f"Motor R status {BP.get_motor_status(RIGHT_M)} now {now_r}")
        print(f"Motor L status {BP.get_motor_status(LEFT_M)} now {now_l}")
        moved = (now_r - start_r + now_l - start_l) / 2
        time.sleep(0.05)


def rotate(
    deg: float,
    rotate_const=ROTATE_CONST,
    effective_radius=EFFECTIVE_RADIUS,
    wheel_separation=WHEEL_SEPARATION,
):
    """
    deg - The goal degrees to rotate.
    """
    rad = deg * DEG_TO_RAD
    wheel_dist = rad * (wheel_separation / 2)
    goal_deg = -floor(rotate_const * (wheel_dist / effective_radius) * RAD_TO_DEG)
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
