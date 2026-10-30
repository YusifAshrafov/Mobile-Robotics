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
color = ColorSensor(Port.S3)                                  # connect color sensor to port S3 in original setup
ultrasonic = UltrasonicSensor(Port.S4)                        # connect ultrasonic sensor to port S4 in original setup

robot = DriveBase(                                            # create drive base using both motors and robot dimensions
    left_motor,                                               # pass left motor to drive base
    right_motor,                                              # pass right motor to drive base
    wheel_diameter=54.8,                                      # set wheel diameter to 54.8 mm
    axle_track=120                                            # set distance between wheels to 120 mm
)                                                             # finish drive base configuration

TAPE = 25                                                     # set black tape detection threshold to 25% reflection
SPEED = 110                                                   # set forward and reverse speed magnitude to 110 mm/s
REVERSE_MM = 150                                              # set backing distance after tape detection to 150 mm
TURN_RATE = 120                                               # set turning rate magnitude to 120 degrees/s
ROBOT_MM = 250                                                # set original obstacle detection distance to 250 mm
CONFIRM = 2                                                   # require two confirming readings in avoidance evaluations
AVOID_MIN_MS = 600                                            # set original avoidance turn time window to 600 milliseconds
WANDER_MIN_MS = 3000                                          # set minimum random drive interval to 3 seconds
WANDER_MAX_MS = 6000                                          # set maximum random drive interval to 6 seconds
WANDER_TURN_MIN = 30                                          # set minimum wandering turn angle to 30 degrees
WANDER_TURN_MAX = 90                                          # set maximum wandering turn angle to 90 degrees
LOOP_MS = 50                                                  # set wait after each controller loop to 50 milliseconds
RUN_TIME_MS = 120000                                          # set original run duration to 120 seconds

memory = {                                                    # create shared memory for behavior states and measurements
    "escape": "WANDER",                                       # start boundary escape behavior in inactive WANDER state
    "near_n": 0,                                              # set count of near obstacle readings to 0
    "avoid_until": -1,                                        # set original avoidance deadline in the past
    "avoid_side": 1,                                          # set initial avoidance direction sign to positive
    "wander_state": "DRIVE",                                  # start wandering behavior in forward driving state
    "wander_until": 0,                                        # leave first wandering deadline unset
    "wander_mark": 0,                                         # set initial angle marker for wandering turns to 0 degrees
    "wander_angle": 0                                         # set initial wandering turn target to 0 degrees
}                                                             # finish shared behavior memory setup

timer = StopWatch()                                           # create timer for run duration and behavior deadlines

def read_sensors():                                           # function for collecting sensor readings and robot estimates
    return {                                                  # build snapshot of current readings for behavior functions
        "reflection": color.reflection(),                     # read reflected light intensity in percent
        "distance": ultrasonic.distance(),                    # read forward obstacle distance in mm
        "odo_mm": robot.distance(),                           # read estimated signed drive distance in mm
        "odo_deg": robot.angle(),                             # read estimated drive base angle in degrees
        "t": timer.time()                                     # read elapsed time in milliseconds
    }                                                         # finish sensor snapshot returned to controller

def escape(s, st):                                            # function for highest-priority boundary escape behavior
    state = st.get("escape", "WANDER")                        # read escape state, use WANDER if key is missing

    if state == "WANDER":                                     # check for tape when an escape is not already running
        if s["reflection"] < TAPE:                            # check if reflection is below black tape threshold
            st["escape"] = "REVERSE"                          # switch escape behavior to backing away
            st["escape_mark_mm"] = s["odo_mm"]                # save estimated distance at start of reverse movement
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
            st["last_turn"] = 1 if st["escape_angle"] > 0 else -1  # remember completed escape turn direction for original avoidance
            st["escape"] = "WANDER"                           # finish escape and return its state to inactive WANDER
            return None                                       # release control to the next lower-priority behavior
        return (0, TURN_RATE if st["escape_angle"] > 0 else -TURN_RATE)  # continue escape rotation using stored turn direction

def avoid(s, st):                                             # function for obstacle avoidance, below escape in priority
    near = s["distance"] < ROBOT_MM                           # check if obstacle is closer than original 250 mm threshold
    st["near_n"] = st.get("near_n", 0) + 1 if near else 0     # count near readings, reset count when obstacle is outside threshold
    active = st.get("avoid_until", -1) > s["t"]               # check if original avoidance deadline has not elapsed

    if st["near_n"] >= CONFIRM and not active:                # start avoidance after confirmation if no timed turn is active
        st["avoid_side"] = st.get("last_turn", choice((-1, 1)))  # reuse last escape direction, or choose a random fallback direction
        st["avoid_until"] = s["t"] + AVOID_MIN_MS             # set timed avoidance deadline 600 milliseconds from now
        active = True                                         # mark timed avoidance turn as active

    if active:                                                # check if timed avoidance turn should continue
        return (0, st["avoid_side"] * TURN_RATE)              # request in-place avoidance turn in stored direction

    return None                                               # release control to the next lower-priority behavior

def wander(s, st):                                            # function for lowest-priority random wandering behavior
    state = st.get("wander_state", "DRIVE")                   # read wandering state, use DRIVE if key is missing

    if state == "DRIVE":                                      # check if wandering behavior is driving forward
        if st.get("wander_until", 0) == 0:                    # check if first wandering drive deadline is unset
            st["wander_until"] = s["t"] + randint(WANDER_MIN_MS, WANDER_MAX_MS)  # set next wandering turn deadline 3 to 6 seconds from now

        if s["t"] >= st["wander_until"]:                      # check if wandering drive deadline has elapsed
            side = choice((-1, 1))                            # randomly choose negative or positive turning direction
            st["wander_angle"] = side * randint(WANDER_TURN_MIN, WANDER_TURN_MAX)  # choose signed wandering angle between 30 and 90 degrees
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

last_print = -1000                                            # allow first status line to print immediately

while timer.time() < RUN_TIME_MS:                             # repeat controller loop while run duration has not elapsed
    s = read_sensors()                                        # collect current sensor readings and robot estimates
    winner, cmd = referee(s, memory)                          # select winning behavior and its speed/turn command
    robot.drive(cmd[0], cmd[1])                               # apply selected speed in mm/s and turning rate in degrees/s

    if s["t"] - last_print >= 1000:                           # print status when at least 1 second passed since last print
        print(                                                # print current arena status in terminal
            "time:", s["t"] // 1000,                          # print elapsed time in whole seconds
            "winner:", winner,                                # print name of selected behavior
            "ref:", s["reflection"],                          # print current reflected light reading
            "dist:", s["distance"]                            # print ultrasonic distance in mm
        )                                                     # finish terminal status output
        last_print = s["t"]                                   # save time of this status output

    wait(LOOP_MS)                                             # wait 50 milliseconds before next controller loop

robot.stop()                                                  # stop drive base motion
ev3.speaker.beep()                                            # make beep through EV3 speaker
print("Finished: 120 seconds")                                # print completion message for original 120-second run
