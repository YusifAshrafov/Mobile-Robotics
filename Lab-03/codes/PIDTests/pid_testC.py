#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick                            # import EV3 brick functions
from pybricks.ev3devices import Motor, UltrasonicSensor       # import motor and ultrasonic sensor functions
from pybricks.parameters import Port                          # import motor and sensor port names
from pybricks.robotics import DriveBase                       # import control of both drive motors as one robot
from pybricks.tools import wait, StopWatch, DataLog           # import delay function, timer and CSV data logger


# hardware
ev3 = EV3Brick()                                              # create EV3 brick object

left_motor = Motor(Port.B)                                    # connect left motor to port B
right_motor = Motor(Port.C)                                   # connect right motor to port C

eyes = UltrasonicSensor(Port.S4)                              # connect ultrasonic sensor to port S4

robot = DriveBase(                                            # create drive base using both motors and robot dimensions
    left_motor,                                               # pass left motor to drive base
    right_motor,                                              # pass right motor to drive base
    wheel_diameter=55.5,                                      # set wheel diameter to 55.5 mm
    axle_track=104                                            # set distance between wheels to 104 mm
)                                                             # finish drive base configuration


# controller settings
SETPOINT = 200                                                # set desired distance from target to 200 mm

KP = 6.0                                                      # set proportional gain to 6.0
KI = 3.0                                                      # set integral gain to 3.0
KD = 0.5                                                      # set derivative gain to 0.5

UMAX = 300                                                    # set maximum forward or reverse speed magnitude to 300 mm/s

LOOP_TIME = 50                                                # set wait after each loop to 50 milliseconds
H = 0.05                                                      # use nominal 0.05 s interval in derivative and integral calculations

ALPHA = 0.3                                                   # give new derivative 30% weight and previous filtered value 70%

RUN_TIME = 10000                                              # set test duration to 10000 milliseconds = 10 seconds


# csv log
log = DataLog(                                                # create CSV logger with the following column names
    't_ms',                                                   # name column for elapsed time in milliseconds
    'd_mm',                                                   # name column for measured distance in mm
    'e_mm',                                                   # name column for distance error in mm
    'u_mms',                                                  # name column for commanded robot speed in mm/s
    name='PID_TestC'                                          # set CSV log base name to PID_TestC
)                                                             # finish CSV logger configuration

clock = StopWatch()                                           # start timer for test duration and logged timestamps


# controller variables
integral = 0.0                                                # set accumulated distance error to 0
e_prev = 0.0                                                  # set previous distance error to 0 mm
deriv = 0.0                                                   # set initial filtered error rate to 0 mm/s


def clamp(value, minimum, maximum):                           # function for keeping a value between two limits

    if value > maximum:                                       # check if value is above upper limit
        return maximum                                        # return upper limit instead

    if value < minimum:                                       # check if value is below lower limit
        return minimum                                        # return lower limit instead

    return value                                              # return original value if it is inside limits


print("PID CONTROLLER - TEST C")                              # print controller type and test label in terminal
print("KP =", KP)                                             # print proportional gain in terminal
print("KI =", KI)                                             # print integral gain in terminal
print("KD =", KD)                                             # print derivative gain in terminal
print("Setpoint =", SETPOINT, "mm")                           # print desired distance and its unit


while clock.time() < RUN_TIME:                                # repeat control loop while test duration has not elapsed

    # 1. read sensor
    d = eyes.distance()                                       # read distance to target in mm

    if d > 2500:                                              # treat distance above 2500 mm as an invalid reading

        robot.stop()                                          # stop robot before retrying sensor reading

        print("Invalid sensor reading:", d)                   # print rejected distance reading in terminal

        wait(LOOP_TIME)                                       # wait 50 milliseconds before next iteration

        continue                                              # skip remaining calculations and try next loop


    # 2. error
    e = d - SETPOINT                                          # subtract desired distance from measured distance in mm


    # 3. derivative
    raw_deriv = (e - e_prev) / H                              # divide error change by nominal interval to get raw error rate

    deriv = (                                                 # calculate filtered rate of change of distance error
        ALPHA * raw_deriv                                     # take 30% of new raw error rate
        + (1 - ALPHA) * deriv                                 # add 70% of previous filtered error rate
    )                                                         # store filtered error rate for controller


    # 4. calculate command
    u = (                                                     # calculate PID speed command using current controller values
        KP * e                                                # calculate proportional contribution from current distance error
        + KI * integral                                       # add contribution from accumulated error
        + KD * deriv                                          # add contribution from filtered error rate
    )                                                         # store PID linear speed command in mm/s


    # 5. anti-windup
    # only integrate when motor command is not saturated
    if abs(u) < UMAX:                                         # allow integration only when command magnitude is below speed limit
        integral += e * H                                     # accumulate error using nominal interval for next speed calculation


    # 6. limit command
    u = clamp(u, -UMAX, UMAX)                                 # limit robot speed command to range -300 to +300 mm/s


    # 7. drive
    robot.drive(u, 0)                                         # command linear speed u in mm/s with zero turning rate


    # 8. log
    log.log(                                                  # save one row of measurements and speed command to CSV
        clock.time(),                                         # log elapsed time in milliseconds
        d,                                                    # log measured distance in mm
        e,                                                    # log distance error in mm
        u                                                     # log limited robot speed command in mm/s
    )                                                         # finish writing this data row


    print(                                                    # print current control values in terminal
        "t:", clock.time(),                                   # print elapsed time in milliseconds
        "d:", d,                                              # print measured distance in mm
        "e:", e,                                              # print distance error in mm
        "u:", u                                               # print commanded robot speed in mm/s
    )                                                         # finish terminal output line


    e_prev = e                                                # save current error for derivative calculation in next loop

    wait(LOOP_TIME)                                           # wait 50 milliseconds before next iteration


robot.stop()                                                  # stop robot after timed test

print("Test finished.")                                       # print message that timed test has finished
print("CSV saved.")                                           # print message that CSV measurements were recorded
