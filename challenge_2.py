from brickpi3 import SensorError
from baselib.base import BP, RIGHT_M, LEFT_M, SONAR_M, move, rotate, init_BP, DEG_TO_RAD, EPSILON, waypoint, UseMotorMaxDPS, RAD_TO_DEG, WHEEL_SEPARATION, EFFECTIVE_RADIUS
from time import sleep
from math import cos, sin, exp, atan2, asin
from random import gauss, random
import traceback

SONAR_S = BP.PORT_4
# GOAL_DEG = 60
ROTATE_SONAR_DPS = 90


SAFE_FRAC = 0.25 # 
SAFE_MARGIN = 0.2 # on either side
CORNER_MARGIN = 0.17
SAFE_DIST = 35
# FORWARD_FRAC = 0.25

ROBOT_WIDTH = WHEEL_SEPARATION + 3 # weird bumper
ROBOT_RADIUS = ROBOT_WIDTH / 2
ROBOT_LENGTH = WHEEL_SEPARATION * 0.8 # guess

FOR_FORWARD_EXCLUDE_MEASUREMENTS_ABOVE = SAFE_DIST * (1 + SAFE_MARGIN) # can remove forward frac when we move constant
FOR_FORWARD_SAFE_DIST_EITHER_SIDE = ROBOT_RADIUS * (1 + SAFE_MARGIN)
FOR_CORNER_SAFE_GAP = ROBOT_WIDTH * (1 + CORNER_MARGIN) # safe margin eiher side
FOR_CORNERS_EXCLUDE_MEASUREMENTS_ABOVE = SAFE_DIST * (1 + SAFE_FRAC)

FORWARD_HALF_DEG = atan2(FOR_FORWARD_SAFE_DIST_EITHER_SIDE, SAFE_DIST) * RAD_TO_DEG
CORNER_MEASURE_HALF_DEG = 60

CAN_EPSILON = 20


# problem: fix so that forward is moving constant until exactly within epsilon of safe dist
# so that we dont ahve to sue forward frac everywhere

SONAR_OFFSET = -2

def get_forward_depth(measurements: dict[float, int]):
    # negatvie angle in dict is left
    # exludde measurements that are above FOR_FORWARD_EXCLUDE_MEASUREMENTS_ABOVE
    # turn into a list of distnaces (value not key) and mutliply each by cos of the angle (key) tog et forward distance
    forward_distances = []
    for angle, distance in measurements.items():
        if distance is not None:
            forward_distance = distance * cos(DEG_TO_RAD * angle)
            forward_distances.append(forward_distance)
    # find the minimum of the forward distances and return it
    return min(forward_distances) if forward_distances else None

def get_best_corners_state(robot_x, robot_y, measurements: dict[float, int]):
    # for both left (negative angle) and right (pos)
    # get all angles (keys that are negative and group)
    left_measurements = [(angle, distance) for angle, distance in measurements.items() if angle < 0 and distance is not None]
    right_measurements = [(angle, distance) for angle, distance in measurements.items() if angle > 0 and distance is not None]
    # left and right measurements should be ordered by magnitude of angle
    left_measurements.sort(key=lambda x: abs(x[0]))
    right_measurements.sort(key=lambda x: abs(x[0]))
    left_translation = (0, 0)
    right_translation = (0, 0)
    left_angle = None
    right_angle = None
    print(f"Left measurements: {left_measurements}")
    print(f"Right measurements: {right_measurements}")
    for measurement in [left_measurements, right_measurements]:
        # rotate funciton (postiive angle => left but with sonar measurements positive angle => right)
        # from 0 to higher magnitude angles, check if its a corner
        # do this by checking if the depth of the next 2 measurements is greater than the depth of the last measurement  + CAN_EPSILON
        for i in range(1, len(measurement)-1):
            angle, distance = measurement[i]
            prev_angle, prev_distance = measurement[i-1]
            next_angle, next_distance = measurement[i+1]
            if distance > prev_distance + CAN_EPSILON and next_distance > prev_distance + CAN_EPSILON:
                # we have found a corner, return the translation to get to the corner
                # this is given by the distance of the last measurement * cos of its angle (to get forward translation) and sin of its angle (to get sideways translation)
                forward_translation = prev_distance * cos(DEG_TO_RAD * prev_angle)
                sideways_translation = prev_distance * sin(DEG_TO_RAD * prev_angle)
                # if positive sonar angle=> right, but positive translation angle => left, so flip the sideways translation
                if angle > 0:
                    sideways_translation = -sideways_translation
                if angle < 0:
                    left_angle = angle
                    left_translation = (forward_translation, sideways_translation)
                else:
                    right_angle = angle
                    right_translation = (forward_translation, sideways_translation)
                break

    # select the corner with the smallest absolute angle
    # get the coordinates of the corner by adding the translation to the robot coordinates
    # return (x,y,angle to rotate to face corner)
    # if positive sonar angle=> right, but positive translation angle => left, so flip the angle
    if left_angle is not None and right_angle is not None:
        if abs(left_angle) < abs(right_angle):
            return (robot_x+left_translation[0], robot_y+left_translation[1], -left_angle)
        else:
            return (robot_x+right_translation[0], robot_y+right_translation[1], -right_angle)
    elif left_angle is not None:
        return (robot_x+left_translation[0], robot_y+left_translation[1], -left_angle)
    elif right_angle is not None:
        return (robot_x+right_translation[0], robot_y+right_translation[1], -right_angle)
    else:        
        print("No corners found => probably a wall => move forward")
        return None
    

def get_waypoint_from_corner(robot_x, robot_y, corner_x, corner_y, angle_to_corner):
    # distance to corner
    distance_to_corner = ((corner_x - robot_x)**2 + (corner_y - robot_y)**2)**0.5
    # get theta to point we want to move
    # use half_safety_gap /2 for safety margin opposite angle
    # 2sine-1(safety_gap / corner_distance)
    safety_gap = FOR_CORNER_SAFE_GAP / 2
    extra_deg_rotation =2 * asin((safety_gap / 2) / distance_to_corner) * RAD_TO_DEG
    if angle_to_corner < 0:
        extra_deg_rotation = -abs(extra_deg_rotation)
    to_theta = angle_to_corner + extra_deg_rotation
    print(f"Distance to corner is {distance_to_corner}, angle to corner is {angle_to_corner}, to theta is {to_theta}")
    # make sure to_theta has the same sign as angle_to_corner
    if angle_to_corner < 0:
        to_theta = -abs(to_theta)
    else:
        to_theta = abs(to_theta)

    # get the coordinates of to theta bearing at distance to corner from robot x and robot y
    to_x = robot_x + distance_to_corner * cos(DEG_TO_RAD * to_theta)
    to_y = robot_y + distance_to_corner * sin(DEG_TO_RAD * to_theta)
    return to_x, to_y

    # get position distance to corner + safety gap in the direction of corner 


def reset_bearings(theta_bearing):
    # theta in degrees (rtheta bearing in radians not used here always conveted)
    # sets the bearings of the motors to 0 and rotation sensor bearing to 0
    # rotate theta bearing backward to return to 0 degrees
    # convert theta bearing from radians to degrees
    print(f"Resetting bearings to 0 degrees from theta bearing {theta_bearing}")
    rotate(-theta_bearing, verbose=False)
    BP.set_motor_position(SONAR_M, 0)
    return


def get_sonar_measurements(half_deg_bearing) -> dict[float, int]:
    def half(goal_deg: float, measure: bool=True):
        sonar_start = BP.get_motor_encoder(SONAR_M)
        sonar_moved = 0
        BP.set_motor_position(SONAR_M, sonar_start + goal_deg)
        res = dict() if measure else None
        while abs(goal_deg - sonar_moved) > EPSILON:
            sonar_deg = BP.get_motor_encoder(SONAR_M)
            if measure:
                sonar_values = [None for _ in range(5)]
                for i in range(len(sonar_values)):
                    try:
                        sonar_values[i] = BP.get_sensor(SONAR_S)
                    except SensorError:
                        print("Sensor could not be read")
                sonar_values = [val for val in sonar_values if val is not None]
                sonar_value = None if not sonar_values else sonar_values[len(sonar_values)//2]
                res[sonar_deg] = sonar_value 
            sleep(0.02) 
            sonar_moved = sonar_deg - sonar_start
            # print(f"Motor Sonar status {BP.get_motor_status(SONAR_M)} now {sonar_deg}")
        return res

    with UseMotorMaxDPS(ROTATE_SONAR_DPS):
        first_half = half(half_deg_bearing)
        half(-half_deg_bearing, False)
        second_half = half(-half_deg_bearing)
        half(half_deg_bearing, False)
    return {**first_half, **second_half}

try:
    init_BP()
    BP.set_sensor_type(SONAR_S, BP.SENSOR_TYPE.NXT_ULTRASONIC)
    state = (0, 0, 0)
    x, y, theta = state
    END_GOAL_CM = 320

    MOVE_DPS = 320
    STOP_DPS = 0

    BP.set_motor_dps(RIGHT_M, MOVE_DPS)
    BP.set_motor_dps(LEFT_M, MOVE_DPS)

    encoder_start_r = BP.get_motor_encoder(RIGHT_M)
    encoder_start_l = BP.get_motor_encoder(LEFT_M)
    reset_bearings(theta)
    while x < END_GOAL_CM:
        measurements = get_sonar_measurements(FORWARD_HALF_DEG)
        depth = get_forward_depth(measurements)
        print(f"Depth is {depth}")

        if depth <= SAFE_DIST:
            BP.set_motor_dps(RIGHT_M, STOP_DPS)
            BP.set_motor_dps(LEFT_M, STOP_DPS)

            encoder_end_r = BP.get_motor_encoder(RIGHT_M)
            encoder_end_l = BP.get_motor_encoder(LEFT_M)

            encoder_moved_r = encoder_end_r - encoder_start_r
            encoder_moved_l = encoder_end_l - encoder_start_l
            encoder_degrees_moved = (encoder_moved_r + encoder_moved_l) / 2
            cm_moved = encoder_degrees_moved * (EFFECTIVE_RADIUS * DEG_TO_RAD)
            x += cm_moved

            # we are too close, work out safe space either side of obstacle to move to
            sleep(0.15)

            print("Getting corner measurements")
            measurements = get_sonar_measurements(CORNER_MEASURE_HALF_DEG)
            best_corner_tuple = get_best_corners_state(x, y, measurements)

            if best_corner_tuple is None:
                # we didnt find a corner => probably a wall => move forward
                print("No corners found => probably a wall => move forward")
            else:
                corner_x, corner_y, corner_angle = best_corner_tuple
                print(f"We are at {x}, {y} with theta {theta}")
                print(f"Best corner is at {corner_x}, {corner_y} with angle {corner_angle}")
                to_x, to_y = get_waypoint_from_corner(x, y, corner_x, corner_y, corner_angle)
                print(f"Moving to {to_x}, {to_y}")
                x, y, rtheta = waypoint((x, y, theta), (to_x, to_y), verbose=False)
                theta = RAD_TO_DEG * rtheta
                reset_bearings(theta)
                theta = 0
                
            BP.set_motor_dps(RIGHT_M, MOVE_DPS)
            BP.set_motor_dps(LEFT_M, MOVE_DPS)
        else:
            print(f"Moving foward") # remove later, debug


            
            
except KeyboardInterrupt:
    BP.reset_all()
except Exception as e:
    print(traceback.format_exc())
finally:
    BP.reset_all()

