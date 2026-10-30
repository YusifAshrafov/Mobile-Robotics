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
        

SETPOINT = 200

KP = 6.0
KD = 0.5
KI = 0.0

UMAX = 300

LOOP_TIME = 50       # ms
H = 0.05             # 50 ms = 0.05 s

ALPHA = 0.3          # derivative filter

RUN_TIME = 10000      # Test B = 10 seconds


        
# CSV LOG
        

log = DataLog(
    't_ms',
    'd_mm',
    'e_mm',
    'u_mms',
    name='PD_TestB'
)

clock = StopWatch()


        
# VARIABLES
        

e_prev = 0.0
deriv = 0.0


        
# CLAMP
        

def clamp(value, minimum, maximum):

    if value > maximum:
        return maximum

    if value < minimum:
        return minimum

    return value


        
# START
        

print("PD CONTROLLER - TEST B")
print("KP =", KP)
print("KD =", KD)
print("Setpoint =", SETPOINT, "mm")


while clock.time() < RUN_TIME:

    # 1. READ SENSOR
    d = eyes.distance()

    if d > 2500:

        robot.stop()
        print("Invalid sensor reading:", d)

        wait(LOOP_TIME)
        continue


    # 2. ERROR
    e = d - SETPOINT


    # 3. DERIVATIVE
    raw_deriv = (e - e_prev) / H

    deriv = (
        ALPHA * raw_deriv
        + (1 - ALPHA) * deriv
    )


    # 4. PD CONTROLLER
    u = KP * e + KD * deriv


    # 5. LIMIT SPEED
    u = clamp(u, -UMAX, UMAX)


    # 6. DRIVE
    robot.drive(u, 0)


    # 7. LOG
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


    # Save current error for next loop
    e_prev = e

    wait(LOOP_TIME)


robot.stop()

print("Test finished.")
print("CSV saved.")
