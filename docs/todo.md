Drone guide

# Construction part

## Hardware assembly

  * Frame assembly
  * ESC wiring and soldering

## RC radio

  * Transmitter+Receiver pairing
  * Setting up transmitter channels
  * Setting up receiver's failsafe
  * Description of the sticks+channels

## Pixhawk stabilization

 * Description of the pixhawk's inputs (features)
   * Offboard mode
 * Proper wiring
 * What needs to be set outside of the default settings
   * Channel mappings
   * Min throttle value
   * Failsafe
     * Land on a current spot
   * Gain tunning (tight gains)
 * SDCARD configuration file

## USB Hub

## Voltage regulators

## FTDI

  * Pixhawk
  * Teraranger
  * Dan's board

## Batteries

  * What type and parameters of batteries
  * Proper storage and charging
  * In-Flight battery chacking
    * Use battery beeper
    * have a battery reserve (30%) is important
    * "know your flight time"

## SW+HW integration

  * Installing the NUC computer
    * running the SCRIPT
    * setting up time synchronization (chrony)
    * setting up the Multimaster
      * setting up the /etc/hosts (so the drones know their neighbors IPs and Hostnames)
      * ... Vojta?
    * set up a local git server
  * Setting up the FTDI links (udev rules)
  * BlueFox camera ID (getting it and setting it to .bashrc) 

## Antenna placing

  * making sure that wifi antennas are not close to RC radio antennas

# On-table testing and verification (ozivovani)

## List of submodules with their respective "routines"

 * RTK GPS board (Dan's board)
   * software, flashing, description
 * Sensors
   * Garmin Lidar
     * description, parameters, datasheet
   * Cameras
     * links
   * Teraranger rangefinder

# RTK GPS

  * Base station setting (Dan's chapter)
    * setting up the home position
    * debugging problems
    * ...

## Startup procedures

  * How to fly the drone manually
    * arming
    * modes of the transmitter
  * *Pre-flight checklist*
  * Pre-experimental session checklist
    * sensor calibration
    * manual flight with aim to test stabilization and general soundness of the drone
  * In-situ drone handling (what to do before touching drone)
  * Basic types of emergency situations and how to react
  * Basic safety rules (where to fly, where not to fly, and safety around flying area)
  * Procedure for the automatic takeoff
    * connecting to the drone
    * running the programs
    * verifying the sensor data
    * turning on the controllers
    * arming
    * offboard mode
    * takeoff sequence (takeoff, switching to the right tracker)
    * ... do whatever you want
  * non-standard situations during the experiment
    * NUC will crash
    * program crash
    * RC ROS failsafe

