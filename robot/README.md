# Robot quickstart and usage

The goal of the robot is to patrol an area following a serie of waypoints, and go to a module's location when receiving an alert from it. After reaching the module's location, the robot will search for a fire around itself, thanks to a "fire detection server" that tells him the direction of the fire.

To accomplish this, the robot has multiple states:

- **PATROL**: The default state, where the robot will patrol and area by following waypoints.
- **ALERT**: The state reached when receiving an alert from a module.  
  When entering this state, the robot will compute the most efficient waypoint path to go close to the alert position, and then go to the alert position.  
  Finally, it will automatically enter the SEARCH state when reaching the alert postion.  
  _Note: The shortest path is found from the closest waypoints to the alert and to the curent position, minus the final waypoints if the alert position is closer to the robot than the final waypoint._
- **SEARCH**: The state reached after reaching an alert position. The robot will connect to the "fire detection server" and locate the fire around itself. When located, the robot will resume its patrol.

## Quickstart

This guide will assume that you have a working ROS environment and that a simulation (or a robot) is already running.

The following command will assume you have the Hirataka's campus map.

A makefile is available to quickly iterate over the work. From within the `./robot/` folder:

1. Start the simulation with `make start` (pay attention to the map used, as it may be unavailable)
2. Run your script with `make run`

## Usage

Given that everything is set up, you can start the program using the following command:

```sh
rosrun fire-hazard-detection-system main.py --waypoints <waypoints_file_path>
```

A few CLI argument are available:

### Requiered

- `--waypoints <file_path>`: The relative or absolute path to the waypoints for a given map

### Optional

- `--debug`: The PATROL mode will be disabled, and the ALERT mode will only display the alert without going to the alert destination,
- `--goto <waypoint_idx>`: The robot will start by going to the waypoint of index `<waypoint_idx>`. It will then proceed normally,
- `--host <fire_detection_server_host>` and `--port <fire_detection_server_port>`: The host and port of the fire detection server. If not given, the robot will print a warning and exit SEARCH mode as soon as it enters it.

## Waypoints

New waypoints can easily be added, as well as new waypoints files for other maps, by adding a file to the [`./waypoints/`](./waypoints/) folder.

Waypoint files are formatted as follow:

```txt
waypoint_1: <X_coordinate>, <Y_coordinates>
waypoint_2: <X_coordinate>, <Y_coordinates>
...
```

## Development

The robot's scripts are developped in Python **3.8** for **ROS 1**.

All the scripts are located in the [`./scripts/`](./scripts/) folder.

Breakdown of the files usage:

- [`main.py`](./scripts/main.py): The startup functions; subscribe to the topics (from [`callbacks.py`](./scripts/callbacks.py)), initialize the common variables from [`global_vars.py`](./scripts/global_vars.py), handles command line arguments and start the [`statemachine.py`](./scripts/statemachine.py)
- [`callbacks.py`](./scripts/callbacks.py): a collection of ROS topics to subscribe to.
- [`global_vars.py`](./scripts/global_vars.py): a collection of variables shared and used in different functions.
- [`statemachine.py`](./scripts/statemachine.py): the state machine piloting the robot, as well as the different functions to handle each states of the robot
- [`states.py`](./scripts/states.py): an enumeration of the different robot' states
- [`navigation.py`](./scripts/navigation.py): the core logic behind the robot movements on the map
- [`motion.py`](./scripts/motion.py): the basic commands to move the robot
- [`search.py`](./scripts/search.py): the logic for connecting to the "fire detection server". Blocking, so typically run in another process
- [`receiver.py`](./scripts/receiver.py): the logic for receiving LoRa data from a LoRa module. Blocking, so typically run in another process

## Miscellaneous

Within this folder, multiple 3D models are available:

- The IR Camera used in this project (PureThermal Mini): [IR_cam.ipt](./IR_cam.ipt)
- The Webcam (Logitech C922 Pro) and IR Camera case made to be mounted on the robot: [webcam_case.ipt](webcam_case.ipt)
