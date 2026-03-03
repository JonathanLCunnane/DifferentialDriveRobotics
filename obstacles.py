from brickpi3 import SensorError
from baselib.base import BP, RIGHT_M, LEFT_M, SONAR_M, move, rotate, init_BP, DEG_TO_RAD, EPSILON, waypoint, UseMotorMaxDPS
from time import sleep
from math import cos, sin, exp, atan2
from random import gauss, random
import traceback

LINE_OFFSET = 50
LINE_SCALE = 3
BOX_SIZE = 210
MAX_Y = BOX_SIZE * LINE_SCALE
SONAR_S = BP.PORT_4
GOAL_DEG = 60
ROTATE_SONAR_DPS = 90
VERBOSE = True

SONAR_OFFSET = -2

def get_sonar_measurements() -> dict[float, int]:
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
                print(sonar_values)
                res[sonar_deg] = sonar_value 
            sleep(0.02) 
            sonar_moved = sonar_deg - sonar_start
            print(f"Motor Sonar status {BP.get_motor_status(SONAR_M)} now {sonar_deg}")
        return res

    with UseMotorMaxDPS(ROTATE_SONAR_DPS):
        first_half = half(GOAL_DEG)
        half(-GOAL_DEG, False)
        second_half = half(-GOAL_DEG)
        half(GOAL_DEG, False)
    return {**first_half, **second_half}

try:
    init_BP()
    BP.set_sensor_type(SONAR_S, BP.SENSOR_TYPE.NXT_ULTRASONIC)
    state = (0, 0, 0)
    x, y, theta = state
    x_step = 100
    while x < 320:
        measurements = get_sonar_measurements()
        corners = find_corner_clusters(measurements)
        if corners is None:
            waypoint(x + x_step, y)

    
except KeyboardInterrupt:
    BP.reset_all()
except Exception as e:
    print(traceback.format_exc())
finally:
    BP.reset_all()
