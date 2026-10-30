#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick                            # import EV3 brick functions
from pybricks.ev3devices import Motor                         # import motor control functions
from pybricks.parameters import Port                          # import motor port names

# Initialize EV3
ev3 = EV3Brick()                                              # create EV3 brick object

# Initialize motors
left_motor = Motor(Port.B)                                    # connect left motor to port B
right_motor = Motor(Port.C)                                   # connect right motor to port C

# Robot measurements
WHEEL_RADIUS = 2.74                                           # store wheel radius in cm
WHEEL_DISTANCE = 12.0                                         # store distance between wheels in cm


def drive_straight(distance):                                 # function for driving given distance in cm
    """
    Drive forward by the given distance in centimetres.
    """

    motor_degrees = (180 * distance) / (3.14159 * WHEEL_RADIUS)  # convert distance in cm into motor rotation in degrees

    left_motor.run_angle(300, motor_degrees, wait=False)      # start left motor at 300 deg/s without waiting
    right_motor.run_angle(300, motor_degrees)                 # start right motor at 300 deg/s and wait until it finishes


def turn(angle):                                              # function for turning robot by given angle in degrees
    """
    Turn the robot in place by the given angle in degrees.
    """

    motor_degrees = (WHEEL_DISTANCE * angle) / (2 * WHEEL_RADIUS)  # calculate wheel rotation needed for an in-place turn

    left_motor.run_angle(300, motor_degrees, wait=False)      # start left motor turn at 300 deg/s without waiting
    right_motor.run_angle(300, -motor_degrees)                # rotate right motor in opposite direction and wait until it finishes


# Drive around a 50 cm square

drive_straight(50)                                            # drive first side of square, 50 cm
turn(-90)                                                     # turn robot by -90 degrees

drive_straight(50)                                            # drive second side of square, 50 cm
turn(-90)                                                     # turn robot by -90 degrees

drive_straight(50)                                            # drive third side of square, 50 cm
turn(-90)                                                     # turn robot by -90 degrees

drive_straight(50)                                            # drive fourth side of square, 50 cm
turn(-90)                                                     # make final -90 degree turn
