# BitcrazeDrone Following Program
This Repository Includes all Scripts used to run 2 bitcraze drones in a cat and mouse configuration where one follows the other. This repository still requires the CFLib and CFClient repositories from Crazy Flie.

## position_hl_commander.py
This script is a modified version of the standard one found in the cflib but with the following changes:
1. The blocking function of the go_to command has been disabled so you can constantly send new position information to the drone allowing it to always move towards the current target point.
2. The go_to command has had the yaw parameter added in to allow for the user to set all 4 DOF of the drone.

## followingScript.py
This script contains all the functions required to run the scipt as well as several test patterns that we used on the code. To Use this script as is:
1. Calibrate the drones to have the same reference frame using the CFClient.
2. Change the URIs to match your drones.
3. Change the boundary coordinates.

If you are planning on modifying the Script make sure to follow the following requirements:
1. Keep the update frequency low, we found that 40Hz works well. if the frequency is set too high it will result in latency of several seconds from what we think is a build of of commands in the radio.
2. Constantly call the persue function to make sure that the collision system is running and that the point the drone is targetting is up to date.
3. Set Tar_rad slightly bigger than min_rad or else the drone will enter the min rad and activate the avoidance script continuously.
4. If you are using pred slowly test increasing the value as too much noise can result in the drone becoming more unstable.


