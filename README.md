# AAQuad-Evolution



What is this project ?

	The AAQuad-Evolution is a flight computer I've built to control a Quadcopter. Flight is something of a passion of mine and this long running project has been a great opportunity to keep implementing some cool things I learn as I progress in my schooling and career.
	It is currently not autonomous and requires pilot input to operate, the computer currently only takes care of continued stabilisation in flight. 
	It includes Hardware design of a custom PCB, all the C code that runs on the Atmega328pb processor and simulink simulations used to design and tune the pid algorithms.
	It is the successor of the Original AAQuad project. That project became unmanageable due to poor design and testing practices.


Directories

	- All C code is contained in the src directory with the exception of main.c which is in the root directory.
	- The PCB folder contains the KiCad design files and some pictures of the finished PCBs.
	- The MatlabModel folder contains the simulink models for the quadcopter.
	- The design folder contains some notes and ideas about the problems posed and their solutions aswell as little plans of execution.
	- the Stimuli folder contains stimulus test cases used to test the external and pin-change interrupts to ensure proper operation of the PilotInstructions module.

Progression

	- The current state of the project is one where I am converting the matlab pid models into C code and integrating that with the rest of the modules.
	- The drivers for the IMU, ultrasonic distance sensor, PWM chip and receiver for the newest itteration of the PCB are completed and have all been tested.
	- Because the integration is not complete at this time, the code on the master branch will currently not be able to fly the newest AAQUAD.
	- The last working version can be seen at this commit https://github.com/Fberbari/AAQuad-Evolution/tree/36f1a275297e55dd9b30459dbdbe85f911d86360. Everything ran on the version 2 hardware. At that time, the flight computer could keep the quadcopter relatively stable in hover, but had trouble steering the aircraft.These problems are being delt with with the updated PID algorithms.

## Developer: Anthony Berbari


I want to give a shoutout to Anny Wang, who was the original inspiration for the project and whose name appears on the current PCB. 
