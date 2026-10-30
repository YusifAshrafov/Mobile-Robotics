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
color = ColorSensor(Port.S3)
ultrasonic = UltrasonicSensor(Port.S4)

robot = DriveBase(
    left_motor,
    right_motor,
    wheel_diameter=54.8,
    axle_track=120
)

TAPE = 25
SPEED = 110
REVERSE_MM = 150
TURN_RATE = 120
ROBOT_MM = 250
CONFIRM = 2
AVOID_MIN_MS = 600
WANDER_MIN_MS = 3000
WANDER_MAX_MS = 6000
WANDER_TURN_MIN = 30
WANDER_TURN_MAX = 90
LOOP_MS = 50
RUN_TIME_MS = 120000

memory = {
    "escape": "WANDER",
    "near_n": 0,
    "avoid_until": -1,
    "avoid_side": 1,
    "wander_state": "DRIVE",
    "wander_until": 0,
    "wander_mark": 0,
    "wander_angle": 0
}

timer = StopWatch()

def read_sensors():
    return {
        "reflection": color.reflection(),
        "distance": ultrasonic.distance(),
        "odo_mm": robot.distance(),
        "odo_deg": robot.angle(),
        "t": timer.time()
    }

def escape(s, st):
    state = st.get("escape", "WANDER")

    if state == "WANDER":
        if s["reflection"] < TAPE:
            st["escape"] = "REVERSE"
            st["escape_mark_mm"] = s["odo_mm"]
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
            st["last_turn"] = 1 if st["escape_angle"] > 0 else -1
            st["escape"] = "WANDER"
            return None
        return (0, TURN_RATE if st["escape_angle"] > 0 else -TURN_RATE)

def avoid(s, st):
    near = s["distance"] < ROBOT_MM
    st["near_n"] = st.get("near_n", 0) + 1 if near else 0
    active = st.get("avoid_until", -1) > s["t"]

    if st["near_n"] >= CONFIRM and not active:
        st["avoid_side"] = st.get("last_turn", choice((-1, 1)))
        st["avoid_until"] = s["t"] + AVOID_MIN_MS
        active = True

    if active:
        return (0, st["avoid_side"] * TURN_RATE)

    return None

def wander(s, st):
    state = st.get("wander_state", "DRIVE")

    if state == "DRIVE":
        if st.get("wander_until", 0) == 0:
            st["wander_until"] = s["t"] + randint(WANDER_MIN_MS, WANDER_MAX_MS)

        if s["t"] >= st["wander_until"]:
            side = choice((-1, 1))
            st["wander_angle"] = side * randint(WANDER_TURN_MIN, WANDER_TURN_MAX)
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

last_print = -1000

while timer.time() < RUN_TIME_MS:
    s = read_sensors()
    winner, cmd = referee(s, memory)
    robot.drive(cmd[0], cmd[1])

    if s["t"] - last_print >= 1000:
        print(
            "time:", s["t"] // 1000,
            "winner:", winner,
            "ref:", s["reflection"],
            "dist:", s["distance"]
        )
        last_print = s["t"]

    wait(LOOP_MS)

robot.stop()
ev3.speaker.beep()
print("Finished: 120 seconds")
