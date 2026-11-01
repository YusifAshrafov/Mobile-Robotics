# Lab 04 — Arena Navigation and Behavior-Based Control

This laboratory explores autonomous arena navigation using a LEGO EV3 robot equipped with a color sensor and an ultrasonic sensor. The robot combines boundary escape, obstacle avoidance, and random wandering. An original controller is compared with an upgraded version that detects obstacles earlier, uses larger turns at critical distances, and adds scanning turns when an obstacle remains ahead.

## Folder contents

```text
Lab-04/
├── README.md
├── codes/
│   ├── main.py
│   └── main_new.py
├── data/
│   ├── terminal1.txt
│   ├── terminal2.txt
│   ├── terminal3.txt
│   └── terminal4.txt
└── graphs/
    ├── Graph1.png
    └── Graph2.png
```

## Programs

| File | Description |
| --- | --- |
| [main.py](codes/main.py) | Original controller. Combines boundary escape, timed obstacle-avoidance turns, and random wandering. The saved version runs for 120 seconds. |
| [main_new.py](codes/main_new.py) | Upgraded controller. Adds early and critical obstacle detection, angle-based avoidance turns, additional scanning, and event counters. The saved version runs for 180 seconds. |

Both programs use a referee to select the highest-priority behavior that returns a motion command. A command contains linear speed in mm/s and turning rate in degrees/s. Returning `None` allows the next behavior to be considered. Once a behavior supplies a command, the lower-priority behaviors are not evaluated during that iteration.

| Priority | Behavior | Action |
| --- | --- | --- |
| 1 — highest | `escape` | Detect black boundary tape, reverse, and turn away. |
| 2 | `avoid` | Rotate to avoid an obstacle detected by the ultrasonic sensor. |
| 3 — lowest | `wander` | Drive forward and periodically make a random turn. |

The behaviors share a memory dictionary containing their states, distance or angle markers, and timing information. The programs do not plan a fixed route through the arena.

## Robot configuration

| Parameter | Original: `main.py` | Upgraded: `main_new.py` |
| --- | --- | --- |
| Controller | LEGO EV3 | LEGO EV3 |
| Runtime | Pybricks EV3 MicroPython | Pybricks EV3 MicroPython |
| Left motor | Port B | Port B |
| Right motor | Port C | Port C |
| Color sensor | Port S3 | Port S4 |
| Ultrasonic sensor | Port S4 | Port S3 |
| Wheel diameter used by DriveBase | 54.8 mm | 54.8 mm |
| Distance between wheels used by DriveBase | 120 mm | 120 mm |
| Tape detection | Reflection below 25% | Reflection below 25% |
| Forward/reverse speed magnitude | 110 mm/s | 110 mm/s |
| Reverse distance after tape detection | 150 mm | 150 mm |
| Turning rate magnitude | 120 degrees/s | 120 degrees/s |
| Boundary escape turn | Random 90–180° | Random 90–180° |
| Obstacle detection | Distance below 250 mm | Early below 400 mm; critical below 160 mm |
| Confirmation count | 2 avoidance readings | 2 avoidance readings |
| Avoidance turn | 600 ms timed window | Angle target: 80–110° early or 160–180° critical |
| Wandering drive interval | Random 3–6 seconds | Random 3–6 seconds |
| Wandering turn | Random 30–90° | Random 30–90° |
| Controller delay | 50 ms after each loop | 50 ms after each loop |
| Saved run duration | 120 seconds | 180 seconds |
| Periodic terminal status | Approximately every 1 second | Approximately every 5 seconds |

The sensor ports are exchanged between versions, so use the wiring appropriate to the selected script. `robot.drive()` receives linear speed in mm/s and turning rate in degrees/s. The 50 ms wait is additional to sensor reading and processing time.

The earlier lab discussion also describes moving the color sensor farther forward. That physical mounting change is not encoded by a parameter in these programs, and setup photographs are not currently included in this folder.

## Behavior details

When the color sensor detects reflection below 25%, boundary escape takes control. The robot reverses until its estimated reverse distance reaches 150 mm, then turns through a randomly selected 90–180° angle. Distance and angle checks use DriveBase estimates from the wheel encoders.

In the original controller, two confirming readings below 250 mm activate a 600 ms in-place avoidance turn. The controller reuses the most recent boundary-escape turn direction when available; otherwise, it chooses a random direction.

The upgraded controller confirms readings below 400 mm for early avoidance and below 160 mm for critical avoidance. It selects a random turn direction and an angle target of 80–110° or 160–180°, respectively. A confirmed critical obstacle can increase an existing smaller turn target. When the target angle is reached but an obstacle remains within 400 mm, the controller extends the rotation by another 60–90°. Boundary escape continues to take priority over avoidance.

When neither higher-priority behavior requests control, wandering drives forward. At a randomly selected deadline 3–6 seconds later, it makes a 30–90° turn before continuing forward.

## Running the programs

1. Prepare the EV3 with the Pybricks EV3 MicroPython environment used in the course.
2. Transfer the selected script from `codes/` to the EV3 using your existing EV3 workflow.
3. Check motor connections, sensor ports, wheel diameter, and wheel spacing against the selected version.
4. Place the robot inside the arena with the color sensor over the floor, away from the black boundary tape.
5. Launch the program. It beeps, resets its distance, angle, and timer, then starts the controller automatically without waiting for a start-button press.
6. Save terminal output and record the trial duration, robot count, configuration, boundary crossings, and physical contacts using the trial records or video.

The original program stops after its configured 120-second run. The upgraded program uses a `finally` block to stop the robot and print its elapsed time and counters when the control loop exits, including when it is interrupted by an exception.

For a new controlled comparison, use the same run duration and robot count for both configurations and retain the exact settings used. The saved 180-second upgraded version does not directly reproduce the 120-second comparison described in the earlier lab discussion. Random turn choices also mean repeated runs will follow different paths.

## Terminal output

The original version periodically prints elapsed seconds, the selected behavior, reflected light intensity, and ultrasonic distance. The upgraded version periodically prints elapsed seconds, the selected behavior, and ultrasonic distance, with additional event messages:

| Message | Meaning |
| --- | --- |
| `EDGE` | A tape detection has started boundary escape. |
| `EARLY` | Confirmed obstacle detection has started an early avoidance turn. |
| `CRITICAL` | Critical-distance detection has started or escalated an avoidance turn. |
| `SCAN` | An avoidance turn has been extended because an obstacle remains ahead. |

At the end, the upgraded version prints elapsed time and counts for tape-triggered escapes, early avoidance, and critical avoidance. These are controller events, not direct measurements of boundary crossings or collisions. Neither program automatically calculates a competition score or saves a CSV file.

Four terminal records are archived in `data/`:

| File | Recorded output |
| --- | --- |
| [terminal1.txt](data/terminal1.txt) | Original-style status output with elapsed time, selected behavior, reflection, and ultrasonic distance. |
| [terminal2.txt](data/terminal2.txt) | Upgraded-style status and event output, including early avoidance at 321 mm at 15 seconds. |
| [terminal3.txt](data/terminal3.txt) | Upgraded-style status and event output, including critical avoidance at 156 mm at 42 seconds. |
| [terminal4.txt](data/terminal4.txt) | Upgraded-style status and event output, including critical avoidance at 84 mm at 54 seconds. |

All four records end with a stop-button message and contain entries beyond 120 seconds. They do not identify their corresponding scored run numbers or the exact 120-second scoring windows. The launch filenames are `shorter_main.py` in the first record and `main.py` in the other three; these names alone do not establish the exact historical source versions.

## Recorded arena results

[Graph1.png](graphs/Graph1.png) compares five trials labelled Old R4, Old R5, New R6, New R7, and New R8. The following scores are read from the plotted bars:

| Trial | Configuration | Score shown (points) |
| --- | --- | ---: |
| Run 4 | Original | 70 |
| Run 5 | Original | 70 |
| Run 6 | Upgraded | 100 |
| Run 7 | Upgraded | 80 |
| Run 8 | Upgraded | 90 |

The mean plotted score is 70 points for the original configuration and 90 points for the upgraded configuration, an increase of 20 points:

```text
original_mean = (70 + 70) / 2 = 70 points
upgraded_mean = (100 + 80 + 90) / 3 = 90 points
```

These values summarize the graph. Terminal logs are included, but the raw scoring table, scoring rules, and videos are not currently included in this folder. The logs have not been linked to the individual scored runs, so they do not independently reconstruct each score or verify boundary crossings and contacts. Only two original and three upgraded trials are represented, and changes to both sensor mounting and software prevent isolating the contribution of either change.

## Ultrasonic detection records

[Graph2.png](graphs/Graph2.png) shows upgraded detection zones and selected readings labelled as events from terminal logs:

| Distance | Configuration represented | Interpretation | Source record and logged time |
| --- | --- | --- | --- |
| 84 mm | Upgraded | Inside the critical-distance zone. | [terminal4.txt](data/terminal4.txt), 54 seconds, `CRITICAL`. |
| 156 mm | Upgraded | Inside the critical-distance zone. | [terminal3.txt](data/terminal3.txt), 42 seconds, `CRITICAL`. |
| 217 mm | Original | Below the original 250 mm avoidance threshold. | [terminal1.txt](data/terminal1.txt), 18 seconds, `winner: avoid`. |
| 321 mm | Upgraded | Inside the early-avoidance zone. | [terminal2.txt](data/terminal2.txt), 15 seconds, `EARLY`. |

In the upgraded code, distances below 160 mm are critical; distances from 160 mm up to, but not including, 400 mm are in the early zone. A distance of 400 mm or more is outside the obstacle-detection threshold, although an already active turn can continue until its angle target is reached. Confirmation readings are still required before a new avoidance activation.

The 475 mm value is the graph's display limit, not the sensor's maximum range. The selected readings illustrate detection events; they do not constitute a continuous distance trace or a collision count.

## Graphs

| File | Description |
| --- | --- |
| [Graph1.png](graphs/Graph1.png) | Competition scores across the five original/upgraded trials shown. |
| [Graph2.png](graphs/Graph2.png) | Ultrasonic detection zones and selected avoidance readings. |

The plotting scripts and raw scoring table are not currently included in this folder. The selected ultrasonic readings can be traced to the terminal records linked above. Preserve the input records and plotting settings alongside any future regenerated figures.
