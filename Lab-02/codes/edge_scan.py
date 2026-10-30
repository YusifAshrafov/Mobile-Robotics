#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick                            # import EV3 brick functions
from pybricks.ev3devices import ColorSensor                   # import color sensor functions
from pybricks.parameters import Port, Button                  # import sensor port names and EV3 button names
from pybricks.tools import wait                               # import wait function, time is given in milliseconds

ev3 = EV3Brick()                                              # create EV3 brick object
sensor = ColorSensor(Port.S3)                                 # connect color sensor to port S3

# Positions measured from tape edge, towards floor
POSITIONS = [0, 5, 10, 15, 20]                                # store manual sensor positions in mm from tape edge toward floor

SAMPLES = 10                                                  # take 10 reflection readings at each position
results = []                                                  # create empty list for positions and average readings

print("========================")                             # print separator before scan information
print("EDGE REFLECTION SCAN")                                 # print scan title in terminal
print("========================")                             # print separator after scan title
print("CENTER = measure")                                     # show button used to start measurement
print("DOWN = stop")                                          # show button used to stop scan while waiting
print("")                                                     # print empty line after instructions

for position in POSITIONS:                                    # repeat measurement for each listed sensor position

    ev3.screen.clear()                                        # clear EV3 screen before showing next position
    ev3.screen.print("Position:")                             # print position label on EV3 screen
    ev3.screen.print(position)                                # print current requested position
    ev3.screen.print("mm")                                    # print position unit, millimeters
    ev3.screen.print("CENTER = read")                         # show button used to read sensor

    print("")                                                 # print empty line before next position
    print("Move sensor to", position, "mm")                   # ask user to move sensor manually to requested position
    print("Press CENTER when ready")                          # ask user to press CENTER when sensor is ready

    while True:                                               # wait for CENTER or DOWN button
        buttons = ev3.buttons.pressed()                       # read currently pressed EV3 buttons

        if Button.DOWN in buttons:                            # check if DOWN button is pressed
            break                                             # leave button waiting loop

        if Button.CENTER in buttons:                          # check if CENTER button is pressed
            break                                             # leave waiting loop to start measurement

        wait(20)                                              # wait 20 milliseconds before checking buttons again

    if Button.DOWN in buttons:                                # check if user requested scan to stop
        break                                                 # leave position loop and print results collected so far

    while ev3.buttons.pressed():                              # wait until all EV3 buttons are released
        wait(20)                                              # wait 20 milliseconds before checking release

    wait(500)                                                 # wait 500 milliseconds before taking readings

    readings = []                                             # create empty list for readings at this position

    for i in range(SAMPLES):                                  # repeat sensor reading 10 times
        value = sensor.reflection()                           # read reflected light value from color sensor
        readings.append(value)                                # add current reading to list
        wait(100)                                             # wait 100 milliseconds between readings

    average = sum(readings) / SAMPLES                         # add readings and divide by sample count to get average

    results.append((position, round(average, 2)))             # save position and average rounded to 2 decimal places

    print("Readings:", readings)                              # print all readings collected at this position
    print("Average:", round(average, 2))                      # print average reflection rounded to 2 decimal places

    ev3.speaker.beep()                                        # make beep to indicate position measurement is finished


print("")                                                     # print empty line before final results
print("========================")                             # print separator before results title
print("FINAL DATA")                                           # print final results title
print("========================")                             # print separator after results title

print("position_mm,reflection")                               # print comma-separated column names for copying data

for position, reflection in results:                          # go through all saved position and reflection pairs
    print(str(position) + "," + str(reflection))              # print position and average reflection separated by comma

print("========================")                             # print separator after results
print("SCAN FINISHED")                                        # print message that scan has finished

ev3.speaker.beep()                                            # make first beep to indicate end of scan
ev3.speaker.beep()                                            # make second beep to indicate end of scan
