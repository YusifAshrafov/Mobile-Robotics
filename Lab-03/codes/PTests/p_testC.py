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
SETPOINT = 200       # desired distance, mm

KP = 6.0             # P controller gain

UMAX = 300           # maximum speed, mm/s

LOOP_TIME = 50       # control loop period, ms

RUN_TIME = 10000     # 10 seconds


# csv log
log = DataLog(
    't_ms',
    'd_mm',
    'e_mm',
    'u_mms',
    name='P_TestC_KP6'
)

clock = StopWatch()


# helper function
def clamp(value, minimum, maximum):

    if value > maximum:
        return maximum

    if value < minimum:
        return minimum

    return value

# start
print("P CONTROLLER - TEST C")
print("KP =", KP)
print("Setpoint =", SETPOINT, "mm")
print("Starting...")


while clock.time() < RUN_TIME:

    # 1. read distance
    d = eyes.distance()


    # Invalid ultrasonic reading
    if d > 2500:

        robot.stop()

        print("Invalid sensor reading:", d)

        wait(LOOP_TIME)

        continue


    # 2. calculate error
    e = d - SETPOINT


    # 3. p controller
    u = KP * e


    # Limit speed
    u = clamp(u, -UMAX, UMAX)


    # 4. drive
    robot.drive(u, 0)


    # 5. save to csv
    log.log(
        clock.time(),
        d,
        e,
        u
    )


    # Terminal output
    print(
        "t:", clock.time(),
        "d:", d,
        "e:", e,
        "u:", u
    )


    # 6. wait
    wait(LOOP_TIME)

# stop
robot.stop()

print("Test finished.")
print("CSV saved.")
