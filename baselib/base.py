import time
import brickpi3
from math import pi, floor, atan2

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
MAX_POWER = 25
LEFT_CONST = 1.085#1.1275
RIGHT_CONST = 1.05
def waypoint(state: tuple[float, float, float], wp: tuple[float, float], max_dist: float=None, verbose=True):
    """Where theta is in radians."""
    max_dist = float("inf") if max_dist is None else max_dist
    x, y, theta = state
    Wx, Wy = wp
    abs_alpha = atan2(Wy-y,Wx-x)
    distance = ((Wx-x)**2 + (Wy-y)**2)**0.5
    distance = min(max_dist, distance)
    if (distance == 0):
        return state
    to_turn = (abs_alpha - theta) * RAD_TO_DEG
    print(f"Waypoint rotating {to_turn} from {theta}")
    rotate(to_turn, rotate_const=(90/74), verbose=verbose)
    print(f"Waypoint moving {distance}")
    move(distance, dist_const=(40/34), verbose=verbose)
    return (Wx, Wy, abs_alpha)

def init_BP():
    """Can throw errors"""
    try:
        BP.offset_motor_encoder(RIGHT_M, BP.get_motor_encoder(RIGHT_M)) # reset right motor 
        BP.offset_motor_encoder(LEFT_M, BP.get_motor_encoder(LEFT_M)) # reset left motor
    except IOError as error:
        print(error)

    BP.set_motor_limits(LEFT_M, LEFT_CONST * MAX_POWER)
    BP.set_motor_limits(RIGHT_M, RIGHT_CONST * MAX_POWER)

def move(
    cm: float,
    dist_const=DIST_CONST,
    effective_radius=EFFECTIVE_RADIUS,
    verbose=True,
):
    """
    cm - The goal distance to move.
    """
    if (cm == 0):
        return
    goal_deg = floor(dist_const * (cm / effective_radius) * RAD_TO_DEG)
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

        if verbose:
            print(f"Motor R status {BP.get_motor_status(RIGHT_M)} now {now_r}")
            print(f"Motor L status {BP.get_motor_status(LEFT_M)} now {now_l}")
        moved = (now_r - start_r + now_l - start_l) / 2
        time.sleep(0.05)


def rotate(
    deg: float,
    rotate_const=ROTATE_CONST,
    effective_radius=EFFECTIVE_RADIUS,
    wheel_separation=WHEEL_SEPARATION,
    verbose=True,
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
    while abs(moved - goal_deg) > EPSILON:
        try:
            now_r = BP.get_motor_encoder(RIGHT_M)
            now_l = BP.get_motor_encoder(LEFT_M)
        except IOError as error:
            print(error)

        if verbose:
            print(f"Motor R status {BP.get_motor_status(RIGHT_M)} now {now_r}")
            print(f"Motor L status {BP.get_motor_status(LEFT_M)} now {now_l}")
        moved = (-now_r + start_r - start_l + now_l) / 2
        time.sleep(0.05)
