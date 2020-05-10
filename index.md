
# Welcome to the MRS UAV system Github pages

## First steps (Beginner user)

We encourage everyone one to familiarise with the following topics since it will make your start with UAVs much easier.
If you find some information **missing**, ask us personally or email us to **tomas.baca@fel.cvut.cz** or **vojtech.spurny@fel.cvut.cz**.

## Suggested reading for newcomers

Are you new here? Start with [Suggested reading for newcomers](suggested-readings).
It will help you with further work.

## File structure

After you had installed everything by our automated install script, it is time to learn about what appeared in your file system.
Learn about the [file structure](file_structure) of our development environment.

## How to compile?

Do you have everything downloaded and cloned?
The next step is to learn how to compile all our workspaces.
Follow to [how to compile](how_to_compile) page.

## How to start the simulation

Is everything compiled and ready to run?
[How to start the simulation](how_to_start_simulation) page will help you with the next step.

## How to command the drone from terminal

Is your drone flying in the simulator?
Continue to [how to command the drone](commanding_the_drone) page to learn how to control it from within the terminal.

## How to write your own ROS package in C++?

Take a look at our [ROS example](https://github.com/ctu-mrs/example_ros_uav) package. It is written as an example ROS package which issues basic commands to the UAV.

## How to do some cool vision stuff with cameras?

Take a look at the [ROS vision example](https://github.com/ctu-mrs/example_ros_vision) package. It subscribes camera images, performs basic image processing and draws the result to the screen.

## Are there some good practices to follow when writing my ROS nodes?

Some recommendations which can save a lot of debugging are listed [here](https://github.com/ctu-mrs/example_ros_uav#good-practices).

## How should I name my variables, so that people other than me can understand my code?

Descriptive variable names are preferred. See [naming variables](https://github.com/ctu-mrs/example_ros_uav#naming-variables) for some examples.

# Deeper look (Advanced user)

If you are familiar with the basic, you can dive deeper into the structure of our system.

  * [Detailed structure](repositories_structure) of the control pipeline repositories.

## Managing submodules in uav_core and simulation

Sub-repositories in *uav_core* and *simulation* are managed by [Gitman](docs/gitman).

## Good examples of C++ ROS usage

Has examples from basic topic and subscribers management to do basic linear algebra or transformations.

'https://github.com/wsnewman/learning_ros'

See documentations of:

[MRS library](/mrs_lib)

[MRS messages](/mrs_msgs)
