---
layout: default
title: How to simulate
parent: Simulation
nav_order: 1
---

| :warning: **Attention please: This page is outdated.**                                                                                           |
| :---                                                                                                                                             |
| The MRS UAV System 1.5 is being released and this page needs updating. Please, keep in mind that the information on this page might not be valid. |

# How to start the simulation
If you have everything installed and compiled, the next step is to test it in the simulation.
Running the simulation consists of several steps, which are usually **automated** using a **tmuxinator** script.
Examples of tmuxinator files can be found in subfolders of [simulation/example_tmux_scripts](https://github.com/ctu-mrs/simulation/tree/master/example_tmux_scripts), e.g., [one_drone_gps](https://github.com/ctu-mrs/simulation/blob/master/example_tmux_scripts/one_drone_gps/session.yml).

Assuming the simulation was cloned to the **~/git** folder, you can launch an example simulation using the following code.
```bash
cd ~/git/simulation/example_tmux_scripts/one_drone_gps
./start.sh
```


# Individual steps explanation

Here we will describe each step required to make the drone fly.

| :warning: **Attention please: Following commands are not meant to be issued manually.**    |
| :---                                                                                       |
|  Typically, we automate all of it; please follow to the end of this page for more info.    |

## 1. running the Gazebo simulator

To start the simulator, we use ROS launch file
```bash
roslaunch mrs_simulation simulation.launch gui:=true
```

| :information_source: Note: The above command will launch default [grass plane world](https://github.com/ctu-mrs/mrs_gazebo_common_resources/blob/master/worlds/grass_plane.world). See the [custom world](https://ctu-mrs.github.io/docs/simulation/custom_world.html) wiki page for more information about world customization. |

## 2. spawn a drone (drones)

| :information_source: **Note: Starting Gazebo with** `simulation.launch` **will automatically start the** `mrs_drone_spawner` **node. If you use a custom launch file to start the simulation, you may need to start the** `mrs_drone_spawner` **manually:** |
| :---                                                                                                                                                                                                                                                        |
| ```roslaunch mrs_simulation mrs_drone_spawner.launch```                                                                                                                                                                                                     |

A drone is added to the running simulation dynamically by calling a service:
```bash
rosservice call /mrs_drone_spawner/spawn "...parameters..."
```

To display the manual (help page) containing a list of all available parameters, perform a dry-run of the script:

```bash
rosrun mrs_simulation mrs_drone_spawner
```

This will only display the help page. To start the `mrs_drone_spawner` node, pass the argument `no_help` to the script. Use the argument `verbose` to display all available parameters on node startup. The `mrs_drone_spawner.launch` uses `no_help` and `verbose` by default.

The spawner parameters are also listed in the `mrs_simulation/config/spawner_params.yaml` file. Note that not all sensors are available for all the vehicle types. The config file stores the available configurations in the following format: `parameter: [default_value, help_description, [compatible_vehicles]]`

The command
```bash
waitForSimulation; rosservice call /mrs_drone_spawner/spawn "1 --enable-rangefinder"

```
will wait for the simulation, then it will spawn a UAV with frame Tarot T650 (default body type) with ID 1 (named uav1), and it enables a down-looking rangefinder sensor.
After it is spawned, you can already see its ROS interface by looking at ROS topics, e.g., by running
```bash
rostopic list
```
The UAV should also be visible in the simulator itself.

## 3. run the control core

The control [core](http://github.com/ctu-mrs/uav_core) has to be run for each drone separately.
The name of the drone is specified by command
```bash
export UAV_NAME=uav*
```
where uav* stands for the name of drone (e.g., uav1, uav2, etc.).
The control can be run by issuing command
```bash
export UAV_NAME=uav1; waitForOdometry; roslaunch mrs_uav_general core.launch
```

## 4. the launch sequence

To the drone take off, several commands have to be issued in the following sequence:
```bash
rosservice call /uav1/control_manager/motors 1
rosservice call /uav1/mavros/cmd/arming 1
rosservice call /uav1/mavros/set_mode 0 offboard
rosservice call /uav1/uav_manager/takeoff
```
However, we also use an *AutomaticStart* program which detects the *armed* state and automatically issues the *motors* and *takeoff* commands, after the *offboard* mode is switch manually.

## 5. commanding the drone

Now, the drone should be in the air and ready to take your commands.
Continue to [UAV-ROS interface](https://ctu-mrs.github.io/docs/system/uav_ros_interface.html) page to learn more, but first, continue to read, how to start the simulation using a tmuxinator script.

## 6. automating it using tmux

* ! this still not the final solution for simulation. But it is essential to know about it.

The commands we described are not meant to be issued manually every time you want to test your software.
That would be tedious and also very impractical.
If you have more than one drone, an environment variable **UAV_NAME** has to be set for each drone differently, which would make manual launching too complicated.
To automate it, we launch a shell script, which creates a tmux session and pre-fills every command to a separate sub-window terminal.
A template of such script can be found in [simulation/example_tmux_scripts/just_flying_bare_tmux](https://github.com/ctu-mrs/simulation/tree/master/example_tmux_scripts/just_flying_bare_tmux):
It is upon you to look inside and get a grasp of what is going on inside.
This *bare tmux* script is typically used only on real UAV.

## 7. automating it using tmuxinator

The script, showcased in the previous section, is not very user-friendly.
Although there is a better solution (as described below), we still use it for a real UAV, since it allows to automated logging of terminals to separate text files (if you are interested, see scripts `~/git/uav_core/tmux_scripts/template.sh` which contains the additional logging part).
We still present the script in this bare form, because it might be useful in some situations.
For example, it allows nesting tmux sessions since the script is written in a way, that "adds the windows" to an existing tmux session if you start it being within an existing tmux session.

But a much cleaner solution for simulations is to use [Tmuxinator](https://github.com/tmuxinator/tmuxinator).
Tmuxinator is a utility designed specifically for defining tmux sessions.
Tmux session is defined using a single `yml` file with much cleaner syntax.
We suggest reading the tmuxinator tutorial and playing with its template.
Command `tmuxinator new test_session` will create a new session **test_session** and fills in the template.

Examples of tmuxinator files can be found in subfolders of [simulation/example_tmux_scripts](https://github.com/ctu-mrs/simulation/tree/master/example_tmux_scripts), e.g., [one_drone_gps](https://github.com/ctu-mrs/simulation/blob/master/example_tmux_scripts/one_drone_gps/session.yml).
A new session can be launched using the bash script **start.sh** located along the **session.yml** in the subfolders.
Since [tmuxinator](https://github.com/tmuxinator/tmuxinator) normally stores its sessions in **~/.tmuxinator**, we launch them using start.sh from the local location to be still able to version them using git.
