import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

RIGHT_M = BP.PORT_B
LEFT_M = BP.PORT_C

try:
    try:
        BP.offset_motor_encoder(RIGHT_M, BP.get_motor_encoder(RIGHT_M)) # reset encoder B
    except IOError as error:
        print(error)
    
    while True:
        # The following BP.get_motor_encoder function returns the encoder value (what we want to use to control motor C's power).
        try:
            power = BP.get_motor_encoder(RIGHT_M) / 10
            if power > 100:
                power = 100
            elif power < -100:
                power = -100
        except IOError as error:
            print(error)
            power = 0
        BP.set_motor_power(LEFT_M, power)
        
        time.sleep(0.02)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
