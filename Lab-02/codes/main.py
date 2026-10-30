#!/usr/bin/env pybricks-micropython - 50.1

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch

# hardware
ev3 = EV3Brick()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

sensor = ColorSensor(Port.S3)

# sensor values
BLACK = 10
FLOOR = 55

TARGET = 26 # (BLACK + FLOOR) / 2     # 32.5


# control bands
EDGE_BAND = 4

EDGE_LOW = TARGET - EDGE_BAND    # 28.5
EDGE_HIGH = TARGET + EDGE_BAND   # 36.5


# speed - test
STRAIGHT_SPEED = 300

HARD_OUTER = 248
HARD_INNER = 49

MAX_SPEED = 320

# false = no continuous terminal printing = faster loop
DEBUG = False


# turn states
NORMAL = 0
TURN_LEFT = 1
TURN_RIGHT = 2

state = NORMAL

# helpers

def stop_robot():
    left_motor.brake()
    right_motor.brake()


def clamp(value, low, high):

    if value < low:
        return low

    if value > high:
        return high

    return value


# start
ev3.screen.clear()
ev3.screen.print("Put sensor")
ev3.screen.print("on line edge")
ev3.screen.print("CENTER = start")

print("")
print("==============================")
print("RIGHT-SIDE SENSOR EDGE FOLLOWER")
print("==============================")
print("BLACK =", BLACK)
print("FLOOR =", FLOOR)
print("TARGET =", TARGET)
print("STRAIGHT =", STRAIGHT_SPEED)
print("HARD OUTER =", HARD_OUTER)
print("HARD INNER =", HARD_INNER)
print("MAX =", MAX_SPEED)
print("")

# wait for CENTER
while Button.CENTER not in ev3.buttons.pressed():
    wait(20)

# wait until button is released
while ev3.buttons.pressed():
    wait(20)


ev3.speaker.beep()

timer = StopWatch()

loops = 0

# main loop
while True:

    # manual stop
    if Button.DOWN in ev3.buttons.pressed():
        break


    # sensor
    reflection = sensor.reflection()

    # state: normal
    if state == NORMAL:
        # sensor too far onto floor
        if reflection > EDGE_HIGH:
            state = TURN_LEFT
            left_speed = HARD_INNER
            right_speed = HARD_OUTER


        # sensor too far onto black
        elif reflection < EDGE_LOW:
            state = TURN_RIGHT
            left_speed = HARD_OUTER
            right_speed = HARD_INNER


        # sensor is close to the edge
        else:
            error = reflection - TARGET

            # stronger correction for higher speed
            correction = error * 7

            left_speed = (
                STRAIGHT_SPEED
                - correction
            )

            right_speed = (
                STRAIGHT_SPEED
                + correction
            )


            left_speed = clamp(
                left_speed,
                80,
                MAX_SPEED
            )

            right_speed = clamp(
                right_speed,
                80,
                MAX_SPEED
            )


    # state: turn left
    elif state == TURN_LEFT:

        left_speed = HARD_INNER
        right_speed = HARD_OUTER

        # keep turn locked until edge is found again
        if reflection <= TARGET:
            state = NORMAL


    # state: turn right
    elif state == TURN_RIGHT:

        left_speed = HARD_OUTER
        right_speed = HARD_INNER

        # keep turn locked until edge is found again
        if reflection >= TARGET:
            state = NORMAL


    # absolutely no reverse
    left_speed = clamp(
        left_speed,
        0,
        MAX_SPEED
    )

    right_speed = clamp(
        right_speed,
        0,
        MAX_SPEED
    )


    # drive
    left_motor.run(left_speed)
    right_motor.run(right_speed)


    # debug
    loops += 1

    if DEBUG and loops % 50 == 0:

        if state == NORMAL:
            state_name = "NORMAL"

        elif state == TURN_LEFT:
            state_name = "LEFT"

        else:
            state_name = "RIGHT"


        print(
            "T:",
            round(timer.time() / 1000, 1),

            "| Ref:",
            reflection,

            "| State:",
            state_name,

            "| L:",
            int(left_speed),

            "| R:",
            int(right_speed)
        )


    # faster controller loop
    wait(5)

# end
stop_robot()

ev3.speaker.beep()

print("")
print(
    "TIME:",
    round(timer.time() / 1000, 2)
)
