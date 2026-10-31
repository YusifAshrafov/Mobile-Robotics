#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import ColorSensor                   # import color sensor functions
from pybricks.parameters import Port                         # import sensor port names
from pybricks.tools import wait                              # import wait function, time is given in milliseconds

sensor = ColorSensor(Port.S3)                                 # connect color sensor to port S3

SAMPLES = 10                                                 # take 10 reflection readings for each surface
SAMPLE_DELAY = 100                                           # wait 100 milliseconds between readings


def measure_surface(name):                                   # function for measuring average reflection from one surface
    print("")                                                # print empty line before measurement instructions
    print("Place sensor over", name)                         # show which surface should be under the sensor
    print("Starting measurement in 3 seconds...")             # give time to check sensor position
    wait(3000)                                               # wait 3 seconds before taking readings

    readings = []                                            # create empty list for reflection readings

    for i in range(SAMPLES):                                 # repeat measurement 10 times
        value = sensor.reflection()                          # read reflected light value from 0 to 100 percent
        readings.append(value)                               # add current reading to the list
        print("Sample", i + 1, "=", value, "%")               # print sample number and measured reflection
        wait(SAMPLE_DELAY)                                   # wait before taking the next reading

    average = sum(readings) / SAMPLES                        # add all readings and divide by the number of samples
    print(name, "AVERAGE =", average, "%")                    # print average reflection for this surface
    return average                                           # return average for the final target calculation


print("------------------------------")                      # print separator before calibration title
print("EV3 COLOR SENSOR CALIBRATION")                        # print program title in terminal
print("------------------------------")                      # print separator after calibration title

FLOOR = measure_surface("FLOOR")                             # measure average reflection from the floor first

print("")                                                    # print empty line before moving instructions
print("Now move sensor to BLACK.")                           # ask user to move sensor over the black tape
print("You have 5 seconds.")                                 # show time available for moving the sensor
wait(5000)                                                   # wait 5 seconds before starting the black-surface sequence

BLACK = measure_surface("BLACK")                             # measure average reflection from the black tape

TARGET = (FLOOR + BLACK) / 2                                  # calculate midpoint between floor and black reflection

print("")                                                    # print empty line before final results
print("------------------------------")                      # print separator before results title
print("RESULTS")                                             # print results title
print("------------------------------")                      # print separator after results title
print("FLOOR =", FLOOR)                                      # print measured floor average
print("BLACK =", BLACK)                                      # print measured black average
print("TARGET =", TARGET)                                    # print calculated midpoint for reference
print("------------------------------")                      # print separator after final results
