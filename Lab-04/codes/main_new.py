#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from random import choice, randint

ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
color = ColorSensor(Port.S4)
ultrasonic = UltrasonicSensor(Port.S3)
robot = DriveBase(left_motor, right_motor, 54.8, 120)
timer = StopWatch()

TAPE = 25
SPEED = 110
REVERSE_MM = 150
TURN_RATE = 120
DETECT_MM = 400
CRITICAL_MM = 160
CONFIRM = 2
WANDER_MIN_MS = 3000
WANDER_MAX_MS = 6000
LOOP_MS = 50
RUN_TIME_MS = 180000

memory = {
    "escape": "WANDER",
    "near_n": 0,
    "close_n": 0,
    "avoid_state": "IDLE",
    "avoid_side": 1,
    "avoid_mark": 0,
    "avoid_target": 0,
    "early_events": 0,
    "critical_events": 0,
    "edge_events": 0,
    "wander_state": "DRIVE",
    "wander_until": 0,
    "wander_mark": 0,
    "wander_angle": 0
}


def read_sensors():
    return {
        "reflection": color.reflection(),
        "distance": ultrasonic.distance(),
        "odo_mm": robot.distance(),
        "odo_deg": robot.angle(),
        "t": timer.time()
    }


def escape(s, st):
    state = st["escape"]

    if state == "WANDER":
        if s["reflection"] < TAPE:
            st["escape"] = "REVERSE"
            st["escape_mark_mm"] = s["odo_mm"]
            st["edge_events"] += 1
            print(s["t"] // 1000, "EDGE")
            return (-SPEED, 0)
        return None

    if state == "REVERSE":
        if st["escape_mark_mm"] - s["odo_mm"] >= REVERSE_MM:
            st["escape"] = "TURN"
            st["escape_mark_deg"] = s["odo_deg"]
            side = choice((-1, 1))
            st["escape_angle"] = side * randint(90, 180)
            return (0, side * TURN_RATE)
        return (-SPEED, 0)

    if state == "TURN":
        if abs(s["odo_deg"] - st["escape_mark_deg"]) >= abs(st["escape_angle"]):
            st["escape"] = "WANDER"
            return None
        return (0, TURN_RATE if st["escape_angle"] > 0 else -TURN_RATE)

    return None


def avoid(s, st):
    dist = s["distance"]
    st["near_n"] = st["near_n"] + 1 if dist < DETECT_MM else 0
    st["close_n"] = st["close_n"] + 1 if dist < CRITICAL_MM else 0

    if st["avoid_state"] == "TURN":
        turned = abs(s["odo_deg"] - st["avoid_mark"])

        if st["close_n"] >= CONFIRM and st["avoid_target"] < 160:
            st["avoid_target"] = randint(160, 180)
            st["critical_events"] += 1
            print(s["t"] // 1000, "CRITICAL", dist)

        if turned >= st["avoid_target"]:
            if dist < DETECT_MM:
                st["avoid_target"] = turned + randint(60, 90)
                print(s["t"] // 1000, "SCAN", dist)
            else:
                st["avoid_state"] = "IDLE"
                st["near_n"] = 0
                st["close_n"] = 0
                return None

        return (0, st["avoid_side"] * TURN_RATE)

    if st["near_n"] >= CONFIRM:
        st["avoid_state"] = "TURN"
        st["avoid_mark"] = s["odo_deg"]
        st["avoid_side"] = choice((-1, 1))

        if st["close_n"] >= CONFIRM:
            st["avoid_target"] = randint(160, 180)
            st["critical_events"] += 1
            print(s["t"] // 1000, "CRITICAL", dist)
        else:
            st["avoid_target"] = randint(80, 110)
            st["early_events"] += 1
            print(s["t"] // 1000, "EARLY", dist)

        return (0, st["avoid_side"] * TURN_RATE)

    return None


def wander(s, st):
    state = st["wander_state"]

    if state == "DRIVE":
        if st["wander_until"] == 0:
            st["wander_until"] = s["t"] + randint(WANDER_MIN_MS, WANDER_MAX_MS)

        if s["t"] >= st["wander_until"]:
            side = choice((-1, 1))
            st["wander_angle"] = side * randint(30, 90)
            st["wander_mark"] = s["odo_deg"]
            st["wander_state"] = "TURN"
            return (0, side * TURN_RATE)

        return (SPEED, 0)

    if abs(s["odo_deg"] - st["wander_mark"]) >= abs(st["wander_angle"]):
        st["wander_state"] = "DRIVE"
        st["wander_until"] = s["t"] + randint(WANDER_MIN_MS, WANDER_MAX_MS)
        return (SPEED, 0)

    return (0, TURN_RATE if st["wander_angle"] > 0 else -TURN_RATE)


def referee(s, st):
    for name, behaviour in [
        ("escape", escape),
        ("avoid", avoid),
        ("wander", wander)
    ]:
        cmd = behaviour(s, st)
        if cmd is not None:
            return name, cmd

    return "none", (0, 0)


ev3.speaker.beep()
robot.reset()
timer.reset()
last_print = -5000

try:
    while timer.time() < RUN_TIME_MS:
        s = read_sensors()
        winner, cmd = referee(s, memory)
        robot.drive(cmd[0], cmd[1])

        if s["t"] - last_print >= 5000:
            print("TIME", s["t"] // 1000,
                  "MODE", winner,
                  "DIST", s["distance"])
            last_print = s["t"]

        wait(LOOP_MS)

finally:
    robot.stop()
    print("Elapsed:", timer.time() / 1000)
    print("Edges:", memory["edge_events"])
    print("Early avoidance:", memory["early_events"])
    print("Critical avoidance:", memory["critical_events"])
    ev3.speaker.beep()
