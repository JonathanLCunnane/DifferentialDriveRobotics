from sys import exit
from baselib.base import BP, RIGHT_M, LEFT_M, rotate, init_BP, waypoint
from math import atan2

try:
    init_BP()

    state = (0, 0, 0)
    while True:
        valid = False
        while not valid:
            try:
                Wx = float(input("Enter waypoint X coordinate in cm: "))
                Wy = float(input("Enter waypoint Y coordinate in cm: "))
                valid = True
            except KeyboardInterrupt:
                exit()
            except:
                print("Ensure X and Y are valid numbers.")
        wp = (Wx, Wy)
        x, y, theta = state

        state = waypoint(state, wp)
        
    print("Done")
except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
except Exception as e:
    print(e)
finally:
    BP.reset_all()
