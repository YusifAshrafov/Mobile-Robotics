#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick                            # import EV3 brick functions
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor  # import motor, color sensor and gyro functions
from pybricks.parameters import Port, Button, Direction       # import port names, buttons and positive rotation direction
from pybricks.tools import wait, StopWatch                    # import delay function and timer
from array import array                                       # import compact arrays for storing measurements
import math                                                   # import angle conversion and trigonometric functions

R = 2.8                                                       # store wheel radius in cm
TARGET = 27                                                   # set target tape-edge reflection to 27%
SPEED = 120                                                   # set base wheel speed to 120 deg/s
KP = 3                                                        # set proportional steering gain to 3
SLOW_ERR = 14                                                 # set averaged error where forward base speed becomes 0
REVERSE = 0.4                                                 # limit reverse base speed to 40% of SPEED
AVG = 0.1                                                     # set new error weight to 0.1 for smoothing
STUCK_MS = 2000                                               # set movement history interval for stuck detection to 2 seconds
PUSH_MS = 400                                                 # set recovery push duration to 400 milliseconds
PUSH = 0.4                                                    # set recovery base speed to 40% of SPEED
CROSS_ANGLE = 50                                              # set crossing turn angle magnitude to 50 degrees
CROSS_SIDE = 3.5                                              # set required sideways crossing movement to 3.5 cm
LOOP_MS = 10                                                  # set wait after controller step to 10 milliseconds
DRIFT_MS = 10000                                              # measure stationary gyro drift for 10 seconds
PRINT_EVERY = 10                                              # print every tenth logged row; 0 disables live row output
                   # if the robot drives worse while printing, use 5 or 10

ev3 = EV3Brick()                                              # create EV3 brick object
left = Motor(Port.B)                                          # connect left motor to port B
right = Motor(Port.C)                                         # connect right motor to port C
sensor = ColorSensor(Port.S3)                                 # connect color sensor to port S3
gyro = GyroSensor(Port.S1, Direction.COUNTERCLOCKWISE)        # connect gyro to S1 and make counterclockwise rotation positive

clock = StopWatch()                                           # create timer for log timestamps and drift correction
# log in compact integer arrays (4 bytes per number instead of a whole Python object)
# distances are stored in mm, written to the CSV in cm
T, A, X, Y, H = array('i'), array('i'), array('i'), array('i'), array('i')  # create integer arrays for time, signed distance, x, y and heading
RF, LP, MD = array('b'), array('b'), array('b')               # create signed-byte arrays for reflection, lap and mode
WL, WR = array('i'), array('i')                               # create integer arrays for left and right encoder angles
x = y = arc = 0.0                                             # set estimated position and signed travel distance to 0 cm
prev_l = prev_r = 0                                           # set previous encoder readings to 0 degrees
lap = 1                                                       # start exploration on lap 1
drift = 0.0                                                   # set initial gyro drift estimate to 0 degrees/s


def step(mode):                                               # function for reading sensors, updating pose and storing one measurement row
    """Read sensors, update position, log one row. mode 0 = following, 1 = crossing."""
    global x, y, arc, prev_l, prev_r                          # use global position, signed travel distance and previous wheel angles
    refl = sensor.reflection()                                # read reflected light intensity from color sensor
    h = int(round(gyro.angle() - drift * clock.time() / 1000))  # subtract estimated drift and round gyro heading to whole degrees
    l, r = left.angle(), right.angle()                        # read current left and right motor encoder angles
    ds = math.radians((l - prev_l) + (r - prev_r)) / 2 * R    # convert average wheel rotation change into signed center movement in cm
    prev_l, prev_r = l, r                                     # save current wheel angles for next position update
    x += ds * math.cos(math.radians(h))                       # add estimated movement along x axis using corrected heading
    y += ds * math.sin(math.radians(h))                       # add estimated movement along y axis using corrected heading
    arc += ds                                                 # add signed step distance to accumulated travel estimate
    T.append(clock.time())                                    # store elapsed time in milliseconds
    A.append(int(arc * 10))                                   # convert signed travel from cm to integer mm for compact storage
    X.append(int(x * 10))                                     # convert x position from cm to integer mm for compact storage
    Y.append(int(y * 10))                                     # convert y position from cm to integer mm for compact storage
    H.append(h)                                               # store corrected heading in degrees
    RF.append(refl)                                           # store current reflection reading
    LP.append(lap)                                            # store current lap number
    MD.append(mode)                                           # store mode, 0 for following and 1 for crossing or turning
    WL.append(l)                                              # store raw left encoder angle in degrees
    WR.append(r)                                              # store raw right encoder angle in degrees
    if PRINT_EVERY and len(T) % PRINT_EVERY == 0:             # check if this row should also be printed live
        print("%d,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d" % (       # format a live row using the same column order as the CSV
            T[-1], A[-1] / 10, X[-1] / 10, Y[-1] / 10, h, refl, lap, mode, l, r))  # print latest time, distances in cm, heading, sensor values and wheel angles
    return refl                                               # return reflection reading to following controller


err_avg = 0.0                                                 # set initial smoothed reflection error to 0
hist = []                                                     # create empty movement history for stuck detection
push_until = 0                                                # set recovery push deadline to 0 milliseconds


def follow(tape_left):                                        # function for following selected tape edge with recovery behavior
    global err_avg, push_until                                # use global smoothed error and recovery push deadline
    refl = step(0)                                            # update pose and log a row in following mode
    err = refl - TARGET                                       # subtract target reflection from measured reflection
    # speed follows the AVERAGE error: quick left-right wiggles cancel out (keeps
    # moving), only a corner that stays dark/white for a while slows it down
    err_avg += AVG * (err - err_avg)                          # move smoothed error 10% toward current reflection error
    turn = KP * err if tape_left else -KP * err               # set steering sign according to which side holds the tape
    # far off the edge -> go slowly BACKWARDS while turning: lets the sensor
    # get round sharp (hairpin) corners instead of jumping over the tape
    v = SPEED * max(-REVERSE, 1 - abs(err_avg) / SLOW_ERR)    # reduce base speed as smoothed error grows, allowing limited reverse motion
    # stuck detector: hardly moved AND hardly turned for STUCK_MS -> short push forward
    now = clock.time()                                        # read current elapsed time in milliseconds
    hist.append((now, arc, gyro.angle()))                     # store time, signed travel and raw gyro heading for progress check
    while hist and now - hist[0][0] > STUCK_MS:               # remove history entries older than the stuck detection interval
        old_t, old_arc, old_h = hist.pop(0)                   # take oldest recorded time, travel distance and heading
        if abs(arc - old_arc) < 1.0 and abs(gyro.angle() - old_h) < 20 and now > push_until:  # check for under 1 cm net movement and under 20 degrees turning with no active push
            push_until = now + PUSH_MS                        # set recovery push deadline 400 milliseconds from now
            ev3.speaker.beep(800, 50)                         # make short 800 Hz beep to indicate recovery push
    if now < push_until:                                      # check if recovery push is currently active
        v = PUSH * SPEED                                      # set forward base speed to 40% of normal base speed
        turn = turn / 2                                       # reduce steering correction by half during recovery push
    left.run(v - turn)                                        # run left motor at base speed minus steering correction in deg/s
    right.run(v + turn)                                       # run right motor at base speed plus steering correction in deg/s
    wait(LOOP_MS)                                             # wait 10 milliseconds before next following step


def turn_by(deg):                                             # function for rotating robot by requested gyro angle in degrees
    target = gyro.angle() + deg                               # set target using current raw gyro angle plus requested rotation
    while abs(target - gyro.angle()) > 2:                     # keep turning while absolute gyro error is above 2 degrees
        e = target - gyro.angle()                             # calculate remaining gyro angle error
        w = max(60, min(200, 4 * abs(e)))                     # limit wheel turning speed magnitude to 60-200 deg/s
        if e < 0:                                             # check if required rotation has negative direction
            w = -w                                            # reverse wheel speed sign for negative rotation
        left.run(-w)                                          # run left motor opposite to signed turning wheel speed
        right.run(w)                                          # run right motor at signed turning wheel speed
        step(1)                                               # update pose and log turning movement with mode 1
        wait(LOOP_MS)                                         # wait 10 milliseconds before next turn correction
    left.hold()                                               # stop left motor and hold final position
    right.hold()                                              # stop right motor and hold final position


def cross(tape_left):                                         # function for crossing tape to follow its opposite side
    """Move the robot to the other side of the tape, same heading."""
    a = CROSS_ANGLE if tape_left else -CROSS_ANGLE            # choose signed 50-degree turn toward tape
    turn_by(a)                                                # turn robot toward tape using gyro control
    start = arc                                               # save signed travel distance before crossing movement
    dist = CROSS_SIDE / math.sin(math.radians(CROSS_ANGLE))   # calculate diagonal distance needed for 3.5 cm sideways movement
    while arc - start < dist:                                 # drive until estimated crossing distance is reached
        left.run(150)                                         # run left motor forward at 150 deg/s
        right.run(150)                                        # run right motor forward at 150 deg/s
        step(1)                                               # update pose and log crossing movement with mode 1
        wait(LOOP_MS)                                         # wait 10 milliseconds before next crossing step
    left.hold()                                               # stop left motor and hold its position
    right.hold()                                              # stop right motor and hold its position
    turn_by(-a)                                               # undo initial crossing rotation to restore original heading
    del hist[:]                                               # clear progress history so crossing pause is not treated as stuck motion


def button_released():                                        # function for waiting until all EV3 buttons are released
    while ev3.buttons.pressed():                              # keep waiting while any button is pressed
        wait(10)                                              # wait 10 milliseconds before checking button state again


ev3.screen.print("TASK 5 EXPLORE")                            # show exploration program title on EV3 screen
ev3.screen.print("CENTER = start")                            # show button used to start experiment
while Button.CENTER not in ev3.buttons.pressed():             # wait until CENTER button is pressed
    wait(10)                                                  # wait 10 milliseconds before checking again
button_released()                                             # wait until all buttons are released
# measure the gyro drift: the robot stands still, so any change is fake rotation
ev3.screen.clear()                                            # clear screen before drift measurement instructions
ev3.screen.print("GYRO CHECK 10 s")                           # show duration of stationary gyro check
ev3.screen.print("DON'T TOUCH")                               # ask user to keep robot still
wait(1000)                                                    # wait 1 second before taking initial gyro reading
a0 = gyro.angle()                                             # save gyro angle before stationary drift measurement
wait(DRIFT_MS)                                                # wait for the 10-second drift measurement interval
drift = (gyro.angle() - a0) / (DRIFT_MS / 1000)               # calculate gyro angle change per second while robot is still
print("gyro drift: %.2f deg/s" % drift)                       # print measured gyro drift in degrees/s
ev3.screen.clear()                                            # clear screen before displaying drift result
ev3.screen.print("drift %.2f deg/s" % drift)                  # display measured drift in degrees/s
if abs(drift) > 1:                                            # check if absolute measured drift exceeds 1 degree/s
    ev3.screen.print("BAD GYRO: replug")                      # show instruction to reconnect gyro; program does not stop here
    ev3.screen.print("it while still")                        # ask user to keep gyro still while reconnecting
gyro.reset_angle(0)                                           # reset gyro angle to 0 degrees
left.reset_angle(0)                                           # reset left motor encoder to 0 degrees
right.reset_angle(0)                                          # reset right motor encoder to 0 degrees
ev3.speaker.beep()                                            # make beep before exploration starts
clock.reset()                                                 # reset experiment timer before logging movement
if PRINT_EVERY:                                               # check if live row output is enabled
    print("time_ms,arc_cm,x_cm,y_cm,heading_deg,reflection,lap,mode,left_deg,right_deg")  # print column names for live exploration measurements

tape_left = True                                              # begin with tape on the robot's left side
error = None                                                  # start with no captured exploration error
try:                                                          # run exploration while allowing ordinary exceptions to be captured
    while True:                                               # repeat following until user ends second lap
        follow(tape_left)                                     # follow current tape edge and log robot measurements
        if ev3.buttons.pressed():                             # check if user pressed any EV3 button to mark lap completion
            left.hold()                                       # stop left motor and hold its position
            right.hold()                                      # stop right motor and hold its position
            button_released()                                 # wait until buttons are released before changing lap
            if lap == 1:                                      # check if user has just ended first lap
                cross(tape_left)                              # cross to other side of tape while preserving overall heading
                lap = 2                                       # switch recorded lap number to 2
                tape_left = False                             # follow with tape on robot's right side for second lap
                ev3.speaker.beep()                            # make beep to indicate second lap has started
            else:                                             # otherwise user has ended second lap
                break                                         # leave exploration loop and proceed to saving data
except Exception as e:                                        # capture ordinary exceptions raised during exploration
    error = e                                                 # remember exception for terminal and screen messages

left.stop()                                                   # stop left motor after exploration ends
right.stop()                                                  # stop right motor after exploration ends
ev3.screen.clear()                                            # clear EV3 screen before status and saving messages
if error is not None:                                         # check if exploration ended with a captured exception
    print("STOPPED BY ERROR:", repr(error))                   # print complete exception representation in terminal
    ev3.screen.print("ERROR:")                                # display error label on EV3 screen
    ev3.screen.print(repr(error)[:20])                        # display first 20 characters of exception representation
ev3.screen.print("SAVING...")                                 # show that CSV writing is starting
ev3.screen.print("do not stop!")                              # ask user not to interrupt file writing
n = len(T)                                                    # store number of logged measurement rows
with open("explore.csv", "w") as f:                           # open explore.csv for writing in current working directory
    f.write("time_ms,arc_cm,x_cm,y_cm,heading_deg,reflection,lap,mode,left_deg,right_deg\n")  # write CSV header and measurement units
    for k0 in range(0, n, 200):                               # write measurements in groups of at most 200 rows
        f.write("".join("%d,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d\n" % (  # format rows in this group and join them into one write
            T[k], A[k] / 10, X[k] / 10, Y[k] / 10, H[k], RF[k], LP[k], MD[k], WL[k], WR[k])  # convert stored millimeters back to cm and include all recorded fields
            for k in range(k0, min(k0 + 200, n))))            # generate row indices for current group without passing final row
ev3.screen.clear()                                            # clear screen after CSV writing completes
if error is not None:                                         # check if an exploration exception was captured
    ev3.screen.print("ERROR - see output")                    # show reminder to inspect terminal error output
ev3.screen.print("saved explore.csv")                         # show name of saved exploration file
ev3.screen.print("{} rows".format(n))                         # show total number of rows saved
wait(5000)                                                    # keep completion message visible for 5 seconds
