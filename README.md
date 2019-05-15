

This Readme was very much added in anticipation of UWARG reviewing my project, but it was about time to add one anyway, so here goes:





What is this project ?

The AAQuad-Evolution is a flight computer I've built to control a Quadcopter. 
It is NOT autonomous and requires continued pilot input to operate, the computer only takes care of continued stabilisation in flight. 
It includes Hardware design of a custom pcb and all the C code that runs on the Atmega328pb processor. 
It is a remake of the Original AAQuad project, written in C++. That project got unmanagable due to poor design and implementation.



Directories

- All c code is contained in the src directory with the exception of main.c which is in the root directory.
- The UnitTests folder contains the source code for all unit tests performed. These are run by ceedling which is managed by the project.yml file in the root directory.
- The design folder contains early notes and ideas about the problems posed and their solutions.
- the Stimuli folder contains stimulus test cases used to test the external and pin-change interrupts early on.
- The PidTests folder contains a simulation framework I built in C to test the PID algorythms aswell as a basic physical analysis of the Quadcopter.
-The schematics folder contains the KiCad design files for the current PCB aswell as some pdf's of the schematics.




Developer: Anthony Berbari


- I want to give a shoutout to Anny Wang, who was the original inspiration for the project and who's name appears on the current PCB :) 