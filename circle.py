import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

RIGHT_M = BP.PORT_B
LEFT_M = BP.PORT_C

try:
    try:
        BP.offset_motor_encoder(RIGHT_M, BP.get_motor_encoder(RIGHT_M)) # reset encoder A
        BP.offset_motor_encoder(LEFT_M, BP.get_motor_encoder(LEFT_M)) # reset encoder D
    except IOError as error:
        print(error)

    BP.set_motor_power(LEFT_M, BP.MOTOR_FLOAT)                          # float motor D
    BP.set_motor_power(RIGHT_M, BP.MOTOR_FLOAT)
    #BP.set_motor_limits(LEFT_M, 50)                                     
    while True:
        # The following BP.get_motor_encoder function returns the encoder value
        try:
            target = 200    # read motor D's position
        except IOError as error:
            print(error)

        BP.set_motor_dps(LEFT_M, -target)             # set the target speed for motor A in Degrees Per Second
        BP.set_motor_dps(RIGHT_M, target)

        print(f"Left motor Target Degrees Per Second: {target}", "  Left motor Status: ", BP.get_motor_status(LEFT_M))
        print(f"Right motor Target Degrees Per Second: {target}", "  Right motor Status: ", BP.get_motor_status(RIGHT_M))

        time.sleep(0.02)

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
