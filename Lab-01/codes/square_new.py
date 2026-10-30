#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick                            # import EV3 brick functions
from pybricks.ev3devices import Motor                         # import motor control functions
from pybricks.parameters import Port                          # import motor port names
from pybricks.tools import wait                               # import wait function, time is given in milliseconds
from math import pi, sin, cos                                 # import pi and trigonometric functions for odometry



# INITIALIZATION
ev3 = EV3Brick()                                              # create EV3 brick object

left_motor = Motor(Port.B)                                    # connect left motor to port B
right_motor = Motor(Port.C)                                   # connect right motor to port C



# CALIBRATED ROBOT PARAMETERS
WHEEL_RADIUS = 2.74                                           # store wheel radius in cm
WHEEL_DISTANCE = 12.0                                         # store distance between wheels in cm

BASE_SPEED = 300                                              # store base motor speed, 300 deg/s

# Proportional heading correction gain
KP = 4.0                                                      # set correction gain, 4 deg/s for each degree of heading error

# Time to stop at every corner for measurement
MEASURE_WAIT = 10000                                          # store corner pause, 10000 milliseconds = 10 seconds



# ROBOT ESTIMATED POSE
x = 0.0                                                       # set initial x position to 0 cm
y = 0.0                                                       # set initial y position to 0 cm
heading = 0.0                                                 # set initial heading to 0 radians

# ODOMETRY

def update_odometry(delta_left_deg, delta_right_deg):         # function for updating estimated position from wheel movement

    global x, y, heading                                      # use global position and heading variables

    # Convert encoder rotation into distance travelled
    # by each wheel
    d_left = (                                                # calculate distance travelled by left wheel
        pi * WHEEL_RADIUS * delta_left_deg                    # multiply pi, wheel radius and left encoder change
    ) / 180.0                                                 # divide by 180 to get left wheel distance in cm

    d_right = (                                               # calculate distance travelled by right wheel
        pi * WHEEL_RADIUS * delta_right_deg                   # multiply pi, wheel radius and right encoder change
    ) / 180.0                                                 # divide by 180 to get right wheel distance in cm

    # Average forward distance
    d = (d_left + d_right) / 2.0                              # get average distance travelled by both wheels in cm

    # Estimated change in robot heading
    d_heading = (                                             # calculate change in robot heading
        d_left - d_right                                      # subtract right wheel distance from left wheel distance
    ) / WHEEL_DISTANCE                                        # divide by wheel spacing to get heading change in radians

    # Update estimated position
    x += d * cos(                                             # add movement along x axis using cosine
        heading + d_heading / 2.0                             # use heading at the middle of this movement
    )                                                         # finish updating x position in cm

    y += d * sin(                                             # add movement along y axis using sine
        heading + d_heading / 2.0                             # use heading at the middle of this movement
    )                                                         # finish updating y position in cm

    heading += d_heading                                      # add heading change to current heading in radians



# DRIVE STRAIGHT WITH CORRECTION
def drive_straight(distance):                                 # function for driving given distance with heading correction

    global heading                                            # use global estimated heading

    # Convert desired distance into motor rotation
    target_degrees = (                                        # calculate motor rotation needed for requested distance
        180.0 * distance                                      # multiply requested distance in cm by 180
    ) / (pi * WHEEL_RADIUS)                                   # divide by pi and wheel radius to get motor degrees

    # Start encoders from zero
    left_motor.reset_angle(0)                                 # reset left motor encoder angle to 0 degrees
    right_motor.reset_angle(0)                                # reset right motor encoder angle to 0 degrees

    previous_left = 0                                         # store previous left encoder reading, initially 0
    previous_right = 0                                        # store previous right encoder reading, initially 0

    # Heading that we want to maintain
    desired_heading = heading                                 # keep estimated heading at start of this straight segment

    while True:                                               # repeat until requested distance is reached

        left_angle = left_motor.angle()                       # read current left motor angle in degrees
        right_angle = right_motor.angle()                     # read current right motor angle in degrees

        # Encoder movement since previous loop
        delta_left = (                                        # calculate left wheel rotation since previous loop
            left_angle - previous_left                        # subtract previous left angle from current angle
        )                                                     # store left encoder change in degrees

        delta_right = (                                       # calculate right wheel rotation since previous loop
            right_angle - previous_right                      # subtract previous right angle from current angle
        )                                                     # store right encoder change in degrees

        # Update robot's estimated pose
        update_odometry(                                      # update estimated position using both encoder changes
            delta_left,                                       # pass left wheel rotation in degrees
            delta_right                                       # pass right wheel rotation in degrees
        )                                                     # finish position update call

        previous_left = left_angle                            # save left angle for next loop
        previous_right = right_angle                          # save right angle for next loop

        # Average wheel rotation
        average_angle = (                                     # calculate average motor rotation
            left_angle + right_angle                          # add current left and right encoder angles
        ) / 2.0                                               # divide by 2 to get average rotation in degrees

        # Stop when requested distance reached
        if average_angle >= target_degrees:                   # check if average rotation reached target
            break                                             # leave loop when requested distance is reached


        # HEADING CORRECTION
        heading_error = desired_heading - heading             # subtract current heading from desired heading
        heading_error_deg = heading_error * 180.0 / pi        # convert heading error from radians into degrees

        correction = KP * heading_error_deg                   # multiply heading error by gain to get speed correction

        # limit correction
        if correction > 80:                                   # check if correction is above 80 deg/s
            correction = 80                                   # limit positive correction to 80 deg/s

        elif correction < -80:                                # check if correction is below -80 deg/s
            correction = -80                                  # limit negative correction to -80 deg/s

        # apply correction to motor speeds
        left_speed = BASE_SPEED + correction                  # add correction to left motor speed
        right_speed = BASE_SPEED - correction                 # subtract correction from right motor speed

        left_motor.run(left_speed)                            # run left motor at calculated speed in deg/s
        right_motor.run(right_speed)                          # run right motor at calculated speed in deg/s

        wait(10)                                              # wait 10 milliseconds before next loop

    left_motor.hold()                                         # stop left motor and hold its position
    right_motor.hold()                                        # stop right motor and hold its position

    wait(200)                                                 # wait 200 milliseconds after straight movement



# TURN FUNCTION
def turn(angle):                                              # function for turning robot by given angle in degrees

    motor_degrees = (                                         # calculate wheel rotation needed for an in-place turn
        WHEEL_DISTANCE * angle                                # multiply wheel spacing by requested turn angle
    ) / (2.0 * WHEEL_RADIUS)                                  # divide by twice wheel radius to get motor degrees

    # Encoder positions before turn
    left_before = left_motor.angle()                          # read left encoder angle before turning
    right_before = right_motor.angle()                        # read right encoder angle before turning

    left_motor.run_angle(                                     # start rotating left motor for the turn
        200,                                                  # set left motor speed to 200 deg/s
        motor_degrees,                                        # set requested left motor rotation in degrees
        wait=False                                            # continue without waiting for left motor to finish
    )                                                         # finish left motor command

    right_motor.run_angle(                                    # start rotating right motor for the turn
        200,                                                  # set right motor speed to 200 deg/s
        -motor_degrees                                        # rotate right wheel in opposite direction
    )                                                         # wait until right motor finishes, default wait is True

    # Actual encoder changes
    delta_left = (                                            # calculate actual left wheel rotation during turn
        left_motor.angle()                                    # read left encoder angle after turning
        - left_before                                         # subtract left angle recorded before turn
    )                                                         # store left encoder change in degrees

    delta_right = (                                           # calculate actual right wheel rotation during turn
        right_motor.angle()                                   # read right encoder angle after turning
        - right_before                                        # subtract right angle recorded before turn
    )                                                         # store right encoder change in degrees

    # Update odometry after turn
    update_odometry(                                          # update estimated position after the turn
        delta_left,                                           # pass actual left wheel rotation in degrees
        delta_right                                           # pass actual right wheel rotation in degrees
    )                                                         # finish position update call

    wait(200)                                                 # wait 200 milliseconds after turning



# PRINT ESTIMATED POSITION
def print_position(corner):                                   # function for printing estimated position at a corner

    print(                                                    # print corner label, position and heading to terminal
        "Corner",                                             # print Corner label
        corner,                                               # print corner number or FINAL
        ": x =",                                              # print label for x position
        round(x, 1),                                          # round x position to 1 decimal place
        "cm, y =",                                            # print cm unit and label for y position
        round(y, 1),                                          # round y position to 1 decimal place
        "cm, heading =",                                      # print cm unit and label for heading
        round(                                                # round heading before printing
            heading * 180.0 / pi,                             # convert heading from radians into degrees
            1                                                 # keep 1 decimal place
        ),                                                    # finish rounding heading
        "deg"                                                 # print heading unit, degrees
    )                                                         # finish terminal output line



# -90 DEGREE SQUARE TEST
# SIDE 1
drive_straight(50)                                            # drive first side of square, 50 cm

print_position(1)                                             # print estimated position before first turn

ev3.speaker.beep()                                            # make beep to indicate measurement pause

# 10 seconds to put sticker / measure
wait(MEASURE_WAIT)                                            # wait 10 seconds for marking or measuring position

turn(-90)                                                     # turn robot by -90 degrees


# SIDE 2
drive_straight(50)                                            # drive second side of square, 50 cm

print_position(2)                                             # print estimated position before second turn

ev3.speaker.beep()                                            # make beep to indicate measurement pause

wait(MEASURE_WAIT)                                            # wait 10 seconds for marking or measuring position

turn(-90)                                                     # turn robot by -90 degrees


# SIDE 3
drive_straight(50)                                            # drive third side of square, 50 cm

print_position(3)                                             # print estimated position before third turn

ev3.speaker.beep()                                            # make beep to indicate measurement pause

wait(MEASURE_WAIT)                                            # wait 10 seconds for marking or measuring position

turn(-90)                                                     # turn robot by -90 degrees


# SIDE 4
drive_straight(50)                                            # drive fourth side of square, 50 cm

print_position(4)                                             # print estimated position before fourth turn

ev3.speaker.beep()                                            # make beep to indicate measurement pause

wait(MEASURE_WAIT)                                            # wait 10 seconds for marking or measuring position

turn(-90)                                                     # make final -90 degree turn


# FINAL ESTIMATED POSITION
print_position("FINAL")                                       # print final estimated position and heading

ev3.speaker.beep()                                            # make beep to indicate end of experiment
