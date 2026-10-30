#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait
from math import pi, sin, cos


 
# INITIALIZATION

ev3 = EV3Brick()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)


 
# CALIBRATED ROBOT PARAMETERS
 

WHEEL_RADIUS = 2.74       # cm
WHEEL_DISTANCE = 12.0     # cm

BASE_SPEED = 300

# Proportional heading correction gain
KP = 4.0

# Time to stop at every corner for measurement
MEASURE_WAIT = 10000      # milliseconds = 10 seconds


 
# ROBOT ESTIMATED POSE
 

x = 0.0
y = 0.0
heading = 0.0             # radians


 
# ODOMETRY
 

def update_odometry(delta_left_deg, delta_right_deg):

    global x, y, heading

    # Convert encoder rotation into distance travelled
    # by each wheel
    d_left = (
        pi * WHEEL_RADIUS * delta_left_deg
    ) / 180.0

    d_right = (
        pi * WHEEL_RADIUS * delta_right_deg
    ) / 180.0

    # Average forward distance
    d = (d_left + d_right) / 2.0

    # Estimated change in robot heading
    d_heading = (
        d_left - d_right
    ) / WHEEL_DISTANCE

    # Update estimated position
    x += d * cos(
        heading + d_heading / 2.0
    )

    y += d * sin(
        heading + d_heading / 2.0
    )

    heading += d_heading


 
# DRIVE STRAIGHT WITH CORRECTION
 

def drive_straight(distance):

    global heading

    # Convert desired distance into motor rotation
    target_degrees = (
        180.0 * distance
    ) / (pi * WHEEL_RADIUS)

    # Start encoders from zero
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    previous_left = 0
    previous_right = 0

    # Heading that we want to maintain
    desired_heading = heading

    while True:

        left_angle = left_motor.angle()
        right_angle = right_motor.angle()

        # Encoder movement since previous loop
        delta_left = (
            left_angle - previous_left
        )

        delta_right = (
            right_angle - previous_right
        )

        # Update robot's estimated pose
        update_odometry(
            delta_left,
            delta_right
        )

        previous_left = left_angle
        previous_right = right_angle

        # Average wheel rotation
        average_angle = (
            left_angle + right_angle
        ) / 2.0

        # Stop when requested distance reached
        if average_angle >= target_degrees:
            break

          
        # HEADING CORRECTION
          

        heading_error = desired_heading - heading
        heading_error_deg = heading_error * 180.0 / pi

        correction = KP * heading_error_deg

        # limit correction
        if correction > 80:
            correction = 80

        elif correction < -80:
            correction = -80

        # apply correction to motor speeds
        left_speed = BASE_SPEED + correction
        right_speed = BASE_SPEED - correction

        left_motor.run(left_speed)
        right_motor.run(right_speed)

        wait(10)

    left_motor.hold()
    right_motor.hold()

    wait(200)


 
# TURN FUNCTION
 

def turn(angle):

    motor_degrees = (
        WHEEL_DISTANCE * angle
    ) / (2.0 * WHEEL_RADIUS)

    # Encoder positions before turn
    left_before = left_motor.angle()
    right_before = right_motor.angle()

    left_motor.run_angle(
        200,
        motor_degrees,
        wait=False
    )

    right_motor.run_angle(
        200,
        -motor_degrees
    )

    # Actual encoder changes
    delta_left = (
        left_motor.angle()
        - left_before
    )

    delta_right = (
        right_motor.angle()
        - right_before
    )

    # Update odometry after turn
    update_odometry(
        delta_left,
        delta_right
    )

    wait(200)


 
# PRINT ESTIMATED POSITION
 

def print_position(corner):

    print(
        "Corner",
        corner,
        ": x =",
        round(x, 1),
        "cm, y =",
        round(y, 1),
        "cm, heading =",
        round(
            heading * 180.0 / pi,
            1
        ),
        "deg"
    )


   
# +90 DEGREE SQUARE TEST
   

# SIDE 1
drive_straight(50)

print_position(1)

ev3.speaker.beep()

# 10 seconds to put sticker / measure
wait(MEASURE_WAIT)

turn(-90)


# SIDE 2
drive_straight(50)

print_position(2)

ev3.speaker.beep()

wait(MEASURE_WAIT)

turn(-90)


# SIDE 3
drive_straight(50)

print_position(3)

ev3.speaker.beep()

wait(MEASURE_WAIT)

turn(-90)


# SIDE 4
drive_straight(50)

print_position(4)

ev3.speaker.beep()

wait(MEASURE_WAIT)

turn(-90)


 
# FINAL ESTIMATED POSITION
 

print_position("FINAL")

ev3.speaker.beep()
