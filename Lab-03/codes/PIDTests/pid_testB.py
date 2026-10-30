#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, DataLog


# hardware
ev3 = EV3Brick()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

eyes = UltrasonicSensor(Port.S4)

robot = DriveBase(
    left_motor,
    right_motor,
    wheel_diameter=55.5,
    axle_track=104
)


# controller settings
SETPOINT = 200

KP = 6.0
KI = 3.0
KD = 0.5

UMAX = 300

LOOP_TIME = 50
H = 0.05

ALPHA = 0.3

RUN_TIME = 10000

e = d - SETPOINT
raw = (e - e_prev) / H
deriv = ALPHA * raw + (1 - ALPHA) * deriv
u = KP * e + KI * integral + KD * deriv
if abs(u) < UMAX:
    integral += e * H
u = max(-UMAX, min(UMAX, u))
robot.drive(u, 0)
log.log(clock.time(), d, e, u)
e_prev = e


# csv log
log = DataLog(
    't_ms',
    'd_mm',
    'e_mm',
    'u_mms',
    name='PID_TestB'
)

clock = StopWatch()


# controller variables
integral = 0.0
e_prev = 0.0
deriv = 0.0


def clamp(value, minimum, maximum):

    if value > maximum:
        return maximum

    if value < minimum:
        return minimum

    return value


print("PID CONTROLLER - TEST B")
print("KP =", KP)
print("KI =", KI)
print("KD =", KD)
print("Setpoint =", SETPOINT, "mm")


while clock.time() < RUN_TIME:

    # 1. read sensor
    d = eyes.distance()

    if d > 2500:

        robot.stop()

        print("Invalid sensor reading:", d)

        wait(LOOP_TIME)

        continue


    # 2. error
    e = d - SETPOINT


    # 3. derivative
    raw_deriv = (e - e_prev) / H

    deriv = (
        ALPHA * raw_deriv
        + (1 - ALPHA) * deriv
    )


    # 4. calculate command
    u = (
        KP * e
        + KI * integral
        + KD * deriv
    )


    # 5. anti-windup
    # only integrate when motor command is not saturated
    if abs(u) < UMAX:
        integral += e * H


    # 6. limit command
    u = clamp(u, -UMAX, UMAX)


    # 7. drive
    robot.drive(u, 0)


    # 8. log
    log.log(
        clock.time(),
        d,
        e,
        u
    )


    print(
        "t:", clock.time(),
        "d:", d,
        "e:", e,
        "u:", u
    )


    e_prev = e

    wait(LOOP_TIME)


robot.stop()

print("Test finished.")
print("CSV saved.")
