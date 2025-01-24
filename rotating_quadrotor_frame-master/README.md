# meam523
Final project for MEAM 523 by Aaron Weinstein and Nadia Kreciglowa

Included Files:
runsim.m    Run this file in matlab to run the simulation. When prompted,
enter a filename to save a .avi video in the animations folder. If no name
is provided, no video will be saved. Modify line 8 to experiment with 
different structures

init_hive.m     This script generates the data structure called "hive" 
which stores the state of the structure as well as it's configuration

calculate_control.m  This function calculates the change in the control 
inputs to track a desired jerk for the structure

traj_eval.m     This function evaluates the trajectory at a given timestep. 
Modify this script to change the trajectory for the structure.

sym_solve.m     This function solves the symbolic jacobian matrices which 
are used throughout the simulation

plotting        This folder contains all of the plotting functions used to 
make the animation look pretty

update_deriv.m  This function simulates the dynamics of the structure by 
linearly integrating the derivatives of the state