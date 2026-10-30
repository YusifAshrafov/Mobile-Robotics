#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait

ev3 = EV3Brick()
sensor = ColorSensor(Port.S3)

# Positions measured from tape edge, towards floor
POSITIONS = [0, 5, 10, 15, 20]

SAMPLES = 10
results = []

print("========================")
print("EDGE REFLECTION SCAN")
print("========================")
print("CENTER = measure")
print("DOWN = stop")
print("")

for position in POSITIONS:

    ev3.screen.clear()
    ev3.screen.print("Position:")
    ev3.screen.print(position)
    ev3.screen.print("mm")
    ev3.screen.print("CENTER = read")

    print("")
    print("Move sensor to", position, "mm")
    print("Press CENTER when ready")

    while True:
        buttons = ev3.buttons.pressed()

        if Button.DOWN in buttons:
            break

        if Button.CENTER in buttons:
            break

        wait(20)

    if Button.DOWN in buttons:
        break

    while ev3.buttons.pressed():
        wait(20)

    wait(500)

    readings = []

    for i in range(SAMPLES):
        value = sensor.reflection()
        readings.append(value)
        wait(100)

    average = sum(readings) / SAMPLES

    results.append((position, round(average, 2)))

    print("Readings:", readings)
    print("Average:", round(average, 2))

    ev3.speaker.beep()


print("")
print("========================")
print("FINAL DATA")
print("========================")

print("position_mm,reflection")

for position, reflection in results:
    print(str(position) + "," + str(reflection))

print("========================")
print("SCAN FINISHED")

ev3.speaker.beep()
ev3.speaker.beep()
