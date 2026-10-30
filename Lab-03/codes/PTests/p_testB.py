#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, DataLog


     
# HARDWARE
     

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


     
# CONTROLLER SETTINGS
     

SETPOINT = 200       # mm

KP = 6.0

UMAX = 300           # maximum speed, mm/s

LOOP_TIME = 50       # milliseconds

RUN_TIME = 10000      # run test for 10 seconds


     
# CSV LOG
     

log = DataLog(
    't_ms',
    'd_mm',
    'e_mm',
    'u_mms',
    name='P_TestB_KP6'
)

clock = StopWatch()


     
# HELPER FUNCTION
     

def clamp(value, minimum, maximum):

    if value > maximum:
        return maximum

    if value < minimum:
        return minimum

    return value


     
# START
     

print("P CONTROLLER - TEST B")
print("KP =", KP)
print("Setpoint =", SETPOINT, "mm")
print("Starting...")


while clock.time() < RUN_TIME:

    # 1. READ SENSOR
    d = eyes.distance()


    # Ignore invalid ultrasonic reading
    if d > 2500:

        robot.stop()

        print("Invalid sensor reading:", d)

        wait(LOOP_TIME)

        continue


    # 2. CALCULATE ERROR
    e = d - SETPOINT


    # 3. P CONTROLLER
    u = KP * e


    # Limit motor speed
    u = clamp(u, -UMAX, UMAX)


    # 4. DRIVE
    robot.drive(u, 0)


    # 5. SAVE DATA
    log.log(
        clock.time(),
        d,
        e,
        u
    )


    # Diagnostic output
    print(
        "t:", clock.time(),
        "d:", d,
        "e:", e,
        "u:", u
    )


    # 6. WAIT
    wait(LOOP_TIME)


     
# STOP
     

robot.stop()

print("Test finished.")
print("CSV saved.")
