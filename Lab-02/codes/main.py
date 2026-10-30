#!/usr/bin/env pybricks-micropython - 50.1

from pybricks.hubs import EV3Brick                            # import EV3 brick functions
from pybricks.ev3devices import Motor, ColorSensor            # import motor and color sensor functions
from pybricks.parameters import Port, Button                  # import port names and EV3 button names
from pybricks.tools import wait, StopWatch                    # import delay function and timer

# hardware
ev3 = EV3Brick()                                              # create EV3 brick object

left_motor = Motor(Port.B)                                    # connect left motor to port B
right_motor = Motor(Port.C)                                   # connect right motor to port C

sensor = ColorSensor(Port.S3)                                 # connect color sensor to port S3

# sensor values
BLACK = 10                                                    # store reference reflection value for black tape
FLOOR = 55                                                    # store reference reflection value for floor

TARGET = 26                                                   # set tuned target to 26, not the black-floor midpoint of 32.5


# control bands
EDGE_BAND = 4                                                 # set reflection band to 4 units on each side of target

EDGE_LOW = TARGET - EDGE_BAND                                 # calculate lower reflection limit, 26 - 4 = 22
EDGE_HIGH = TARGET + EDGE_BAND                                # calculate upper reflection limit, 26 + 4 = 30


# speed - test
STRAIGHT_SPEED = 300                                          # set base speed for normal following, 300 deg/s

HARD_OUTER = 248                                              # set outer wheel speed during hard turns, 248 deg/s
HARD_INNER = 49                                               # set inner wheel speed during hard turns, 49 deg/s

MAX_SPEED = 320                                               # set maximum allowed motor speed, 320 deg/s

# false = no continuous terminal printing = faster loop
DEBUG = False                                                 # disable periodic debug output in control loop


# turn states
NORMAL = 0                                                    # store number for normal following state
TURN_LEFT = 1                                                 # store number for locked left turn state
TURN_RIGHT = 2                                                # store number for locked right turn state

state = NORMAL                                                # start controller in normal following state

# helpers

def stop_robot():                                             # function for stopping both motors
    left_motor.brake()                                        # brake left motor
    right_motor.brake()                                       # brake right motor


def clamp(value, low, high):                                  # function for keeping a value between two limits

    if value < low:                                           # check if value is below lower limit
        return low                                            # return lower limit instead

    if value > high:                                          # check if value is above upper limit
        return high                                           # return upper limit instead

    return value                                              # return original value if it is inside limits


# start
ev3.screen.clear()                                            # clear EV3 screen before showing instructions
ev3.screen.print("Put sensor")                                # print first line of sensor placement instruction
ev3.screen.print("on line edge")                              # ask user to place sensor on tape edge
ev3.screen.print("CENTER = start")                            # show button used to start experiment

print("")                                                     # print empty line in terminal
print("==============================")                       # print separator before program information
print("RIGHT-SIDE SENSOR EDGE FOLLOWER")                      # print controller name in terminal
print("==============================")                       # print separator after controller name
print("BLACK =", BLACK)                                       # print black tape reference value
print("FLOOR =", FLOOR)                                       # print floor reference value
print("TARGET =", TARGET)                                     # print selected target reflection
print("STRAIGHT =", STRAIGHT_SPEED)                           # print normal following base speed
print("HARD OUTER =", HARD_OUTER)                             # print outer wheel speed for hard turns
print("HARD INNER =", HARD_INNER)                             # print inner wheel speed for hard turns
print("MAX =", MAX_SPEED)                                     # print maximum motor speed
print("")                                                     # print empty line after settings

# wait for CENTER
while Button.CENTER not in ev3.buttons.pressed():             # wait until CENTER button is pressed
    wait(20)                                                  # wait 20 milliseconds before checking again

# wait until button is released
while ev3.buttons.pressed():                                  # wait while any EV3 button is still pressed
    wait(20)                                                  # wait 20 milliseconds before checking release


ev3.speaker.beep()                                            # make beep to indicate start

timer = StopWatch()                                           # start timer for elapsed experiment time

loops = 0                                                     # set control loop counter to 0

# main loop
while True:                                                   # repeat controller loop until manually stopped

    # manual stop
    if Button.DOWN in ev3.buttons.pressed():                  # check if DOWN button is pressed
        break                                                 # leave control loop to stop robot


    # sensor
    reflection = sensor.reflection()                          # read reflected light value from color sensor

    # state: normal
    if state == NORMAL:                                       # check if controller is in normal following state
        # sensor too far onto floor
        if reflection > EDGE_HIGH:                            # check if reflection is above upper limit of 30
            state = TURN_LEFT                                 # switch to locked left turn state
            left_speed = HARD_INNER                           # set left wheel to slower inner speed
            right_speed = HARD_OUTER                          # set right wheel to faster outer speed


        # sensor too far onto black
        elif reflection < EDGE_LOW:                           # check if reflection is below lower limit of 22
            state = TURN_RIGHT                                # switch to locked right turn state
            left_speed = HARD_OUTER                           # set left wheel to faster outer speed
            right_speed = HARD_INNER                          # set right wheel to slower inner speed


        # sensor is close to the edge
        else:                                                 # use proportional steering when reading is inside band
            error = reflection - TARGET                       # subtract target from current reflection

            # stronger correction for higher speed
            correction = error * 7                            # multiply reflection error by steering gain of 7

            left_speed = (                                    # calculate left motor speed for normal following
                STRAIGHT_SPEED                                # start with base speed of 300 deg/s
                - correction                                  # subtract steering correction from left speed
            )                                                 # store calculated left motor speed

            right_speed = (                                   # calculate right motor speed for normal following
                STRAIGHT_SPEED                                # start with base speed of 300 deg/s
                + correction                                  # add steering correction to right speed
            )                                                 # store calculated right motor speed


            left_speed = clamp(                               # keep normal left speed inside allowed range
                left_speed,                                   # pass calculated left speed
                80,                                           # set minimum normal speed to 80 deg/s
                MAX_SPEED                                     # set maximum speed to MAX_SPEED
            )                                                 # store limited left speed

            right_speed = clamp(                              # keep normal right speed inside allowed range
                right_speed,                                  # pass calculated right speed
                80,                                           # set minimum normal speed to 80 deg/s
                MAX_SPEED                                     # set maximum speed to MAX_SPEED
            )                                                 # store limited right speed


    # state: turn left
    elif state == TURN_LEFT:                                  # check if controller is already in locked left turn

        left_speed = HARD_INNER                               # keep left wheel at slower inner speed
        right_speed = HARD_OUTER                              # keep right wheel at faster outer speed

        # keep turn locked until edge is found again
        if reflection <= TARGET:                              # check if reflection reached target or went below it
            state = NORMAL                                    # return to normal following on next loop


    # state: turn right
    elif state == TURN_RIGHT:                                 # check if controller is already in locked right turn

        left_speed = HARD_OUTER                               # keep left wheel at faster outer speed
        right_speed = HARD_INNER                              # keep right wheel at slower inner speed

        # keep turn locked until edge is found again
        if reflection >= TARGET:                              # check if reflection reached target or went above it
            state = NORMAL                                    # return to normal following on next loop


    # absolutely no reverse
    left_speed = clamp(                                       # apply final limits to left motor speed
        left_speed,                                           # pass selected left speed
        0,                                                    # set minimum to 0 so reverse is not commanded
        MAX_SPEED                                             # set upper speed limit to MAX_SPEED
    )                                                         # store final left motor speed

    right_speed = clamp(                                      # apply final limits to right motor speed
        right_speed,                                          # pass selected right speed
        0,                                                    # set minimum to 0 so reverse is not commanded
        MAX_SPEED                                             # set upper speed limit to MAX_SPEED
    )                                                         # store final right motor speed


    # drive
    left_motor.run(left_speed)                                # run left motor at selected speed in deg/s
    right_motor.run(right_speed)                              # run right motor at selected speed in deg/s


    # debug
    loops += 1                                                # increase loop counter by 1

    if DEBUG and loops % 50 == 0:                             # print every 50 loops only when DEBUG is enabled

        if state == NORMAL:                                   # check if current state is normal following
            state_name = "NORMAL"                             # store NORMAL label for terminal output

        elif state == TURN_LEFT:                              # check if current state is left turn
            state_name = "LEFT"                               # store LEFT label for terminal output

        else:                                                 # use remaining right turn state
            state_name = "RIGHT"                              # store RIGHT label for terminal output


        print(                                                # print current time, reflection, state and motor speeds
            "T:",                                             # print time label
            round(timer.time() / 1000, 1),                    # convert elapsed milliseconds to seconds, round to 1 decimal

            "| Ref:",                                         # print reflection label
            reflection,                                       # print current reflection reading

            "| State:",                                       # print controller state label
            state_name,                                       # print selected state name

            "| L:",                                           # print left motor label
            int(left_speed),                                  # print left speed as integer

            "| R:",                                           # print right motor label
            int(right_speed)                                  # print right speed as integer
        )                                                     # finish debug output line


    # faster controller loop
    wait(5)                                                   # wait 5 milliseconds before next control loop

# end
stop_robot()                                                  # brake both motors after leaving control loop

ev3.speaker.beep()                                            # make beep to indicate stop

print("")                                                     # print empty line before final time
print(                                                        # print elapsed experiment time
    "TIME:",                                                  # print TIME label
    round(timer.time() / 1000, 2)                             # convert elapsed milliseconds to seconds, round to 2 decimals
)                                                             # finish elapsed time output
