#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick                            # import EV3 brick functions
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor  # import motor, color sensor and ultrasonic sensor functions
from pybricks.parameters import Port                          # import motor and sensor port names
from pybricks.robotics import DriveBase                       # import control of both drive motors as one robot
from pybricks.tools import wait, StopWatch                    # import delay function and timer
from random import choice, randint                            # import random selection and random whole-number functions

ev3 = EV3Brick()                                              # create EV3 brick object
left_motor = Motor(Port.B)                                    # connect left motor to port B
right_motor = Motor(Port.C)                                   # connect right motor to port C
color = ColorSensor(Port.S4)                                  # connect color sensor to port S4 in upgraded setup
ultrasonic = UltrasonicSensor(Port.S3)                        # connect ultrasonic sensor to port S3 in upgraded setup
robot = DriveBase(left_motor, right_motor, 54.8, 120)         # create drive base, wheel diameter 54.8 mm and wheel spacing 120 mm
timer = StopWatch()                                           # create timer for run duration and behavior deadlines

TAPE = 25                                                     # set black tape detection threshold to 25% reflection
SPEED = 110                                                   # set forward and reverse speed magnitude to 110 mm/s
REVERSE_MM = 150                                              # set backing distance after tape detection to 150 mm
TURN_RATE = 120                                               # set turning rate magnitude to 120 degrees/s
DETECT_MM = 400                                               # set early obstacle detection distance to 400 mm
CRITICAL_MM = 160                                             # set critical obstacle detection distance to 160 mm
CONFIRM = 2                                                   # require two confirming readings in avoidance evaluations
WANDER_MIN_MS = 3000                                          # set minimum random drive interval to 3 seconds
WANDER_MAX_MS = 6000                                          # set maximum random drive interval to 6 seconds
LOOP_MS = 50                                                  # set wait after each controller loop to 50 milliseconds
RUN_TIME_MS = 180000                                          # set upgraded run duration to 180 seconds

memory = {                                                    # create shared memory for behavior states and measurements
    "escape": "WANDER",                                       # start boundary escape behavior in inactive WANDER state
    "near_n": 0,                                              # set count of near obstacle readings to 0
    "close_n": 0,                                             # set count of critical obstacle readings to 0
    "avoid_state": "IDLE",                                    # start upgraded avoidance behavior in IDLE state
    "avoid_side": 1,                                          # set initial avoidance direction sign to positive
    "avoid_mark": 0,                                          # set initial angle marker for avoidance turns to 0 degrees
    "avoid_target": 0,                                        # set initial avoidance turn target to 0 degrees
    "early_events": 0,                                        # set early avoidance activation count to 0
    "critical_events": 0,                                     # set critical avoidance activation or escalation count to 0
    "edge_events": 0,                                         # set tape-triggered escape count to 0
    "wander_state": "DRIVE",                                  # start wandering behavior in forward driving state
    "wander_until": 0,                                        # leave first wandering deadline unset
    "wander_mark": 0,                                         # set initial angle marker for wandering turns to 0 degrees
    "wander_angle": 0                                         # set initial wandering turn target to 0 degrees
}                                                             # finish shared behavior memory setup


def read_sensors():                                           # function for collecting sensor readings and robot estimates
    return {                                                  # build snapshot of current readings for behavior functions
        "reflection": color.reflection(),                     # read reflected light intensity in percent
        "distance": ultrasonic.distance(),                    # read forward obstacle distance in mm
        "odo_mm": robot.distance(),                           # read estimated signed drive distance in mm
        "odo_deg": robot.angle(),                             # read estimated drive base angle in degrees
        "t": timer.time()                                     # read elapsed time in milliseconds
    }                                                         # finish sensor snapshot returned to controller


def escape(s, st):                                            # function for highest-priority boundary escape behavior
    state = st["escape"]                                      # read current boundary escape state

    if state == "WANDER":                                     # check for tape when an escape is not already running
        if s["reflection"] < TAPE:                            # check if reflection is below black tape threshold
            st["escape"] = "REVERSE"                          # switch escape behavior to backing away
            st["escape_mark_mm"] = s["odo_mm"]                # save estimated distance at start of reverse movement
            st["edge_events"] += 1                            # increase count of tape-triggered escape events
            print(s["t"] // 1000, "EDGE")                     # print elapsed whole seconds and tape detection event
            return (-SPEED, 0)                                # request reverse motion at 110 mm/s with no turning
        return None                                           # release control to the next lower-priority behavior

    if state == "REVERSE":                                    # check if robot is backing away from boundary
        if st["escape_mark_mm"] - s["odo_mm"] >= REVERSE_MM:  # check if estimated reverse distance reached 150 mm
            st["escape"] = "TURN"                             # switch escape behavior from reverse movement to turning
            st["escape_mark_deg"] = s["odo_deg"]              # save angle at start of escape turn
            side = choice((-1, 1))                            # randomly choose negative or positive turning direction
            st["escape_angle"] = side * randint(90, 180)      # choose signed escape angle between 90 and 180 degrees
            return (0, side * TURN_RATE)                      # request in-place turn at selected signed turning rate
        return (-SPEED, 0)                                    # request reverse motion at 110 mm/s with no turning

    if state == "TURN":                                       # check if boundary escape turn is active
        if abs(s["odo_deg"] - st["escape_mark_deg"]) >= abs(st["escape_angle"]):  # check if estimated escape rotation reached target angle
            st["escape"] = "WANDER"                           # finish escape and return its state to inactive WANDER
            return None                                       # release control to the next lower-priority behavior
        return (0, TURN_RATE if st["escape_angle"] > 0 else -TURN_RATE)  # continue escape rotation using stored turn direction

    return None                                               # release control to the next lower-priority behavior


def avoid(s, st):                                             # function for obstacle avoidance, below escape in priority
    dist = s["distance"]                                      # copy current ultrasonic distance in mm
    st["near_n"] = st["near_n"] + 1 if dist < DETECT_MM else 0  # count readings below 400 mm, otherwise reset near count
    st["close_n"] = st["close_n"] + 1 if dist < CRITICAL_MM else 0  # count readings below 160 mm, otherwise reset critical count

    if st["avoid_state"] == "TURN":                           # check if upgraded avoidance turn is already active
        turned = abs(s["odo_deg"] - st["avoid_mark"])         # calculate absolute rotation since start of avoidance turn

        if st["close_n"] >= CONFIRM and st["avoid_target"] < 160:  # check if confirmed critical distance requires a larger turn target
            st["avoid_target"] = randint(160, 180)            # set total avoidance target to random angle from 160 to 180 degrees
            st["critical_events"] += 1                        # increase count of critical avoidance activations or escalations
            print(s["t"] // 1000, "CRITICAL", dist)           # print elapsed seconds, critical event and distance in mm

        if turned >= st["avoid_target"]:                      # check if estimated turn has reached avoidance target
            if dist < DETECT_MM:                              # check if obstacle is still closer than 400 mm
                st["avoid_target"] = turned + randint(60, 90)  # extend turn target by another 60 to 90 degrees from current angle
                print(s["t"] // 1000, "SCAN", dist)           # print elapsed seconds, extra scanning turn and distance in mm
            else:                                             # otherwise obstacle is outside early detection distance
                st["avoid_state"] = "IDLE"                    # finish upgraded avoidance turn
                st["near_n"] = 0                              # reset count of near obstacle readings
                st["close_n"] = 0                             # reset count of critical obstacle readings
                return None                                   # release control to the next lower-priority behavior

        return (0, st["avoid_side"] * TURN_RATE)              # request in-place avoidance turn in stored direction

    if st["near_n"] >= CONFIRM:                               # start avoidance when near obstacle readings are confirmed
        st["avoid_state"] = "TURN"                            # activate upgraded avoidance turn
        st["avoid_mark"] = s["odo_deg"]                       # save estimated angle at start of avoidance turn
        st["avoid_side"] = choice((-1, 1))                    # randomly choose avoidance turn direction

        if st["close_n"] >= CONFIRM:                          # check if obstacle is confirmed inside critical range
            st["avoid_target"] = randint(160, 180)            # set total avoidance target to random angle from 160 to 180 degrees
            st["critical_events"] += 1                        # increase count of critical avoidance activations or escalations
            print(s["t"] // 1000, "CRITICAL", dist)           # print elapsed seconds, critical event and distance in mm
        else:                                                 # otherwise use early avoidance turn instead of critical turn
            st["avoid_target"] = randint(80, 110)             # set early avoidance turn target between 80 and 110 degrees
            st["early_events"] += 1                           # increase count of early avoidance activations
            print(s["t"] // 1000, "EARLY", dist)              # print elapsed seconds, early avoidance event and distance in mm

        return (0, st["avoid_side"] * TURN_RATE)              # request in-place avoidance turn in stored direction

    return None                                               # release control to the next lower-priority behavior


def wander(s, st):                                            # function for lowest-priority random wandering behavior
    state = st["wander_state"]                                # read current wandering state

    if state == "DRIVE":                                      # check if wandering behavior is driving forward
        if st["wander_until"] == 0:                           # check if first wandering drive deadline is unset
            st["wander_until"] = s["t"] + randint(WANDER_MIN_MS, WANDER_MAX_MS)  # set next wandering turn deadline 3 to 6 seconds from now

        if s["t"] >= st["wander_until"]:                      # check if wandering drive deadline has elapsed
            side = choice((-1, 1))                            # randomly choose negative or positive turning direction
            st["wander_angle"] = side * randint(30, 90)       # choose signed wandering angle between 30 and 90 degrees
            st["wander_mark"] = s["odo_deg"]                  # save angle at start of wandering turn
            st["wander_state"] = "TURN"                       # switch wandering behavior from driving to turning
            return (0, side * TURN_RATE)                      # request in-place turn at selected signed turning rate

        return (SPEED, 0)                                     # request forward motion at 110 mm/s with no turning

    if abs(s["odo_deg"] - st["wander_mark"]) >= abs(st["wander_angle"]):  # check if estimated wandering rotation reached target angle
        st["wander_state"] = "DRIVE"                          # switch wandering behavior back to forward driving
        st["wander_until"] = s["t"] + randint(WANDER_MIN_MS, WANDER_MAX_MS)  # set next wandering turn deadline 3 to 6 seconds from now
        return (SPEED, 0)                                     # request forward motion at 110 mm/s with no turning

    return (0, TURN_RATE if st["wander_angle"] > 0 else -TURN_RATE)  # continue wandering rotation using stored turn direction


def referee(s, st):                                           # function for selecting highest-priority behavior that requests control
    for name, behaviour in [                                  # check behaviors in priority order
        ("escape", escape),                                   # check boundary escape first, highest priority
        ("avoid", avoid),                                     # check obstacle avoidance second
        ("wander", wander)                                    # check wandering last, lowest priority
    ]:                                                        # start evaluating the listed behaviors
        cmd = behaviour(s, st)                                # call behavior with current sensor snapshot and shared memory
        if cmd is not None:                                   # check if behavior returned a motion command
            return name, cmd                                  # select this behavior and skip remaining lower-priority behaviors

    return "none", (0, 0)                                     # request zero speed and turning rate if no behavior supplies a command


ev3.speaker.beep()                                            # make beep through EV3 speaker
robot.reset()                                                 # reset estimated drive distance and angle to 0
timer.reset()                                                 # reset elapsed timer to 0 before starting run
last_print = -5000                                            # allow first status line to print immediately

try:                                                          # run controller loop with cleanup handled in finally block
    while timer.time() < RUN_TIME_MS:                         # repeat controller loop while run duration has not elapsed
        s = read_sensors()                                    # collect current sensor readings and robot estimates
        winner, cmd = referee(s, memory)                      # select winning behavior and its speed/turn command
        robot.drive(cmd[0], cmd[1])                           # apply selected speed in mm/s and turning rate in degrees/s

        if s["t"] - last_print >= 5000:                       # print status when at least 5 seconds passed since last print
            print("TIME", s["t"] // 1000,                     # print elapsed time in whole seconds
                  "MODE", winner,                             # print selected behavior name
                  "DIST", s["distance"])                      # print ultrasonic distance in mm and finish status line
            last_print = s["t"]                               # save time of this status output

        wait(LOOP_MS)                                         # wait 50 milliseconds before next controller loop

finally:                                                      # run cleanup when timed loop ends or an exception interrupts it
    robot.stop()                                              # stop drive base motion
    print("Elapsed:", timer.time() / 1000)                    # print elapsed run time in seconds
    print("Edges:", memory["edge_events"])                    # print tape-triggered escape count, not measured boundary exits
    print("Early avoidance:", memory["early_events"])         # print count of early avoidance activations
    print("Critical avoidance:", memory["critical_events"])   # print count of critical avoidance activations or escalations
    ev3.speaker.beep()                                        # make beep through EV3 speaker
