# Mobile Robotics

Laboratory assignments for the Mobile Robotics course using a LEGO EV3 robot and Pybricks EV3 MicroPython. The experiments progress from basic motion and odometry to line following, feedback control, arena behaviors, and maze mapping.

The repository contains robot programs, recorded measurements, terminal screenshots, figures, and documentation. Each laboratory has its own README describing the setup, program settings, experiment procedure, results, and limitations.

## Laboratory assignments

| Lab | Topic | Main concepts |
| --- | --- | --- |
| [Lab 01](Lab-01/README.md) | Square Navigation and Odometry | Differential-drive motion, wheel encoders, estimated position, and proportional heading correction. |
| [Lab 02](Lab-02/README.md) | Color Sensor Calibration and Line Following | Reflected-light measurements, tape-edge scanning, proportional steering, and locked turning states. |
| [Lab 03](Lab-03/README.md) | Ultrasonic Distance Control with P, PD and PID | Gain selection, derivative filtering, conditional integration, speed saturation, and moving-target experiments. |
| [Lab 04](Lab-04/README.md) | Arena Navigation and Behavior-Based Control | Priority-based selection of boundary escape, obstacle avoidance, and random wandering. |
| [Lab 05](Lab-05/README.md) | Maze Exploration, Mapping and Route Planning | Gyro-assisted odometry, drift compensation, node detection, graph construction, and Dijkstra's algorithm. |

## Repository structure

```text
Mobile-Robotics/
├── README.md
├── Lab-01/    # Motion and odometry
├── Lab-02/    # Color sensor and line following
├── Lab-03/    # Ultrasonic P, PD and PID control
├── Lab-04/    # Arena behaviors
└── Lab-05/    # Maze exploration and mapping
```

Inside the lab folders:

| Location | Contents |
| --- | --- |
| `README.md` | Lab overview, hardware configuration, running instructions, and evidence-backed results. |
| `codes/` | EV3 programs and, in Lab 05, the computer-side mapping script. |
| `data/` | Recorded CSV logs or terminal screenshots where available. |
| `graphs/` | Saved graphs, maps, and reference drawings where available. |

Folders vary according to the material available for each experiment. Program variants retain their original filenames so they can be connected to the corresponding trial records.

## Hardware and software

| Component | Use |
| --- | --- |
| LEGO EV3 brick and two drive motors | Robot platform used across all labs. |
| Motor encoders | Wheel rotation measurements and motion estimation. |
| Color sensor | Tape-edge following, boundary detection, and maze exploration. |
| Ultrasonic sensor | Distance control and arena obstacle detection. |
| Gyroscope | Heading measurement for maze exploration. |
| Pybricks EV3 MicroPython | Runtime for programs executed on the robot. |
| Computer-side Python, NumPy, and Matplotlib | Lab 05 mapping, graph construction, and figure generation. |

Sensor ports, dimensions, calibration values, and movement settings differ between labs and program versions. Use the configuration in the selected lab's README and source file.

## Getting started

Download the repository or clone it:

```bash
git clone https://github.com/YusifAshrafov/Mobile-Robotics.git
cd Mobile-Robotics
```

1. Open the README for the experiment you want to run.
2. Prepare the EV3 with the Pybricks EV3 MicroPython environment used in the course.
3. Check the selected program's motor ports, sensor connections, robot dimensions, and calibration settings.
4. Transfer the robot program to the EV3 and follow that lab's start, stop, and measurement procedure.
5. Preserve the terminal output or generated logs with the exact program version and trial settings.

The EV3 programs use hardware-specific Pybricks imports. The computer-side mapping program in Lab 05 uses standard Python with NumPy and Matplotlib.

Using the archived final exploration CSV, the basic Lab 05 mapping workflow is:

```bash
python -m pip install numpy matplotlib
cd Lab-05
python codes/chart.py data/explore.csv
```

The input is included at `Lab-05/data/explore.csv`. For a new experiment, retrieve its CSV from the EV3 and supply that file's path instead. See the Lab 05 README for heading selection, optional drift correction, manual nodes, and route-planning options.

## Selected recorded results

| Lab | Result summarized in the lab documentation |
| --- | --- |
| Lab 01 | Estimated square closing errors of approximately 10.8–13.4 mm, calculated from the rounded encoder-based positions in four terminal screenshots. |
| Lab 02 | Best recorded completion time of 49.59 s, compared with an initial 85.27 s; a repeated run recorded 50.17 s. |
| Lab 03 | In the archived controller comparisons, PID recorded the lowest whole-log mean absolute error in Test B at 12.79 mm, while P recorded the lowest in Test C at 12.54 mm. |
| Lab 04 | The saved comparison graph shows mean scores of 70 points for two original trials and 90 points for three upgraded trials. |
| Lab 05 | The saved reconstructed map reports estimated loop-closure drift of 3.8 cm for lap 1 and 8.4 cm for lap 2 before per-lap position-closure adjustment. |

These results describe the archived records and figures. Estimated positions are distinct from independent physical measurements, and the trials do not establish one controller or configuration as best under every condition. The individual lab READMEs explain how the results were obtained and what comparisons the evidence supports.

## Reproducing the experiments

Lab 03 includes the CSV inputs used for its numerical result tables. Labs 01 and 02 include terminal measurement screenshots, while Labs 02–05 include saved figures. Lab 04 includes four terminal logs. Lab 05 includes the final and earlier exploration logs, generated node and edge tables, and the mapping script. Some figure-generation scripts and original trial records are not yet archived, including the separate procedure used to generate Lab 05's raw-map figure.

The Lab 03 README documents an existing startup issue in the saved PID Test B program. The Lab 04 README distinguishes the original and upgraded programs' different run durations. Review these details before repeating those experiments.

Lab 05 supports shortest-route planning in software. The supplied EV3 program performs exploration; it does not execute the generated route as a physical shortest-path test.
