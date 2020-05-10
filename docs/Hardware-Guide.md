# Hardware guide 

This text will describe the hardware of the MRS autonomous drones, all the used sensors and their interfaces, and can serve as a drone-building guide.


# Basics - how a drone flies
Drones do not have any wings or aerodynamic control surfaces, all the control is done only by varying the speed of the individual propellers. If the speed of the front facing propellers is reduced, and the speed of the back propellers is increased, the drone pitches forward and starts accelerating forward. Similarly, by varying the speed of the left and right propellers, the drone can roll left and right. By increasing/reducing the speed of all the propellers, the drone can ascend/descend. Changing the yaw angle works on a slightly different principle. Each propeller on a drone spins in the opposite direction to its neighbors, so half the propellers spins in one direction, and the other half spins in the other direction. If we increase the speed of all the props spinning in one direction, and decrease the speed of the ones spinning in the other direction, the drone yaws. The yawing moment is caused by the greater air resistance experienced by the faster spinning propellers.

By combining all of this, we can control the yaw, pitch, roll and the total thrust. Note that this is only 4 degrees of freedom, while the drone itself has 6 of them, meaning that the standard drone is under-actuated. It cannot accelerate forward, without also pitching forward, etc.

# Motors and ESCs
Most modern drones, including all of the MRS drones use BLDC (brush less direct current) motors. These motors consists of three coils which form the stator, and a permanent (usually neodymium) magnet rotor. While a current is flowing through one of the coils, the magnet will align with it. By applying current in different directions through the three coils, a rotating magnetic field is created, which causes the rotor to spin. But the rotating magnetic field has to be synchronized with the rotation of the rotor, if they spin at different speeds, the motor will stall and stop.

To control the BLDC motor, we need an ESC - electronic speed controller. The ESC takes the battery voltage and switches it through the three coils of the motor. The ESC also has to somehow measure the current position of the rotor, so that it can switch the coils at the correct time. This is usually done by detecting the back EMF which is caused by the magnetic rotor passing a non-energized coil. The ESC itself accepts a signal from the flight controller, which defines the required motor throttle.
