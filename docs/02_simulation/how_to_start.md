---
layout: default
title: Buttons
parent: Simulation
---

# How to start simulation

If you have everything [compiled](how_to_compiled), the next step is to test it in the simulation.
The running simulation consists of several steps, which are usually automated using a **tmux** script.
Here we will describe each step required to make the drone fly.

## Attention please

**Following commands are not meant to be issued manually.**
Normally, we automate all of it; please follow to the end of this page for more info.

## 1. running the Gazebo simulator

To start the simulator, we use ROS launch file
```bash
roslaunch simulation simulation.launch world_file:='$(find simulation)/worlds/grass_plane.world' gui:=true
```

## 2. spawn a drone (drones)

To add drones to the world, we have a bash command **spawn**.
It has its help (spawn --help), so we will examine just one of many uses.
The command
```bash
waitForSimulation; spawn 1 --run --delete --enable-rangefinder --enable-ground-truth & spawn 3 --run --delete --enable-rangefinder --enable-ground-truth && fg

```
will wait for the simulation, spawn two drones, named "1" and "3" and enables a rangefinder sensor and ground-truth.
After they are spawned, you can already see their ROS interface by looking at ROS topics, e.g., by running
```bash
rostopic list
```
The drones should be also visible in the simulator itself, have a look at **Models** in the left pane of the simulator.

## 3. run mrs_uav_manager node

The core control nodes have to be run for each drone seperately. The name of the drone is specified by command
```bash
export UAV_NAME=uav*
```
where uavx stands for the name of drone (e.g., uav1, uav2, etc.). So the control and estimation can be run by issuing command
```bash
export UAV_NAME=uav1; waitForOdometry; roslaunch mrs_general core.launch 
```

## 4. the launch sequence

To the drone take off, several commands have to be issued in a following sequence:

```bash
rosservice call /uav1/control_manager/motors 1
rosservice call /uav1/mavros/cmd/arming 1
rosservice call /uav1/mavros/set_mode 0 offboard
rosservice call /uav1/uav_manager/takeoff
```

## 5. commanding the drone

Now, the drone should be in the air and ready to take your commands.
Continue to [how to command the drone](commanding_the_drone) page to learn more, but first, continue to read, how to start the simulation using a tmux script.

## 6. automating it using tmux

The commands we described are not meant to be issued manually every time you want to test your software.
That would be tedious and also very impractical.
If you have more than one drone, an environment variable **UAV_NAME** has to be set for each drone differently, which would make manual launching almost impossible.
To automate it, we launch a shell script, which creates a tmux session and pre-fills every command to a separate terminal.
A template of such script can be found in **~/git/simulation/start_simulation/just_flying.sh**.
It is upon you to look inside and get a grasp of what is going on inside.

Such script can be created for each drone you want to work with.

Contents of a custom sample tmux script:
```bash
#!/bin/bash

SESSION_NAME=uav1
UAV_NAME=$SESSION_NAME

# following commands will be executed first, in each window
pre_input="export UAV_NAME=$UAV_NAME; export ATHAME_ENABLED=0"

# define commands
# 'name' 'command'
input=(
  'Gazebo' "roslaunch simulation simulation.launch world_name:=worlds/grass_plane.world gui:=true
"
  'Spawn' "waitForSimulation; spawn 1 --run --delete --enable-rangefinder --enable-ground-truth
"
  'Control' "waitForOdometry; roslaunch mrs_general core.launch
"
  "PrepareUAV" "waitForControl; rosservice call /$UAV_NAME/control_manager/motors 1; rosservice call /$UAV_NAME/mavros/cmd/arming 1; rosservice call /$UAV_NAME/mavros/set_mode 0 offboard; rosservice call /$UAV_NAME/uav_manager/takeoff;
"
  'Camera_follow' "waitForOdometry; gz camera -c gzclient_camera -f $UAV_NAME
"
  'GoTo' "rosservice call /$UAV_NAME/control_manager/goto \"goal: [15.0, 15.0, 2.0, 0.0]\""
  'GoToRelative' "rosservice call /$UAV_NAME/control_manager/goto_relative \"goal: [5.0, 5.0, 1.0, 3.14]\""
)

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  SESSION_NAME="$(tmux display-message -p '#S')"
fi

# create arrays of names and commands
for ((i=0; i < ${#input[*]}; i++));
do
  ((i%2==0)) && names[$i/2]="${input[$i]}" 
	((i%2==1)) && cmds[$i/2]="${input[$i]}"
done

# run tmux windows
for ((i=0; i < ${#names[*]}; i++));
do
	tmux new-window -t $SESSION_NAME:$(($i+10)) -n "${names[$i]}"
done

sleep 4

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
	tmux send-keys -t $SESSION_NAME:$(($i+10)) "${pre_input};
${cmds[$i]}"
done

sleep 4

tmux select-window -t $SESSION_NAME:0
tmux -2 attach-session -t $SESSION_NAME

clear
```

## Tmux or tmuxinator?

The script, showcased in the previous section, is not very user-friendly.
Although there is a better solution (as described bellow), we still use it for a real hardware, since it allows to automated logging terminals to separate text files (if you are interested, see scripts `~/git/uav_core/tmux_scripts/template.sh` which contains the additional logging part).
We still present the script in this bare form, because it might be useful in some situations.
For example, it allows nesting tmux sessions since the script is written in a way, that just "adds the windows" to an existing tmux session if you start it being withing an existing tmux session.

The much cleaner solution is to use [Tmuxinator](https://github.com/tmuxinator/tmuxinator), a utility designed specifically for defining tmux sessions.
Tmux session is defined using a single `yml` file with much cleaner syntax than the previous solution has.
We suggest to read tmuxinator tutorial and play with its template.
Command `tmuxinator new test_session` will create new session **test_session** and fills in the template.

Examples of tmuxinator files can be found in subfolders of `~/git/simulation/start_simulation/`, e.g. **one_drone_gps**.
New session can be launched using the bash script **start.sh** located along the **session.yml** in the subfolders.
Since [tmuxinator](https://github.com/tmuxinator/tmuxinator) normally stores it sessions in **~/.tmuxinator**, we launch them using start.sh to be still able to version them using git.

The simplest tmuxinator script might look like the following:

```yaml
name: simulation
root: ./

startup_window: status
pre_window: export UAV_NAME=uav1
windows:
  - roscore:
      layout: even-vertical
      panes:
        - roscore
  - gazebo:
      layout: even-vertical
      panes:
        - waitForRos; roslaunch simulation simulation.launch gui:=true
  - status:
      layout: even-vertical
      panes:
        - waitForRos; roslaunch mrs_status status.launch
  - spawn:
      layout: even-vertical
      panes:
        - waitForSimulation; spawn 1 --run --delete --enable-rangefinder --enable-ground-truth
  - control:
      layout: even-vertical
      panes:
        - waitForOdometry; roslaunch mrs_general core.launch
  - takeoff:
      layout: even-vertical
      panes:
        #{ uav1
        - 'waitForControl;
          rosservice call /$UAV_NAME/control_manager/motors 1;
          rosservice call /$UAV_NAME/mavros/cmd/arming 1;
          rosservice call /$UAV_NAME/mavros/set_mode 0 offboard;
          rosservice call /$UAV_NAME/uav_manager/takeoff'
        #}
  - goto:
      layout: even-vertical
      panes:
        - 'history -s rosservice call /$UAV_NAME/control_manager/goto \"goal: \[0.0, 10.0, 1.5, 0.0\]\"'
  - gazebo_camera_follow:
      layout: even-vertical
      panes:
        - waitForOdometry; gz camera -c gzclient_camera -f uav1; history -s gz camera -c gzclient_camera -f uav1
```
