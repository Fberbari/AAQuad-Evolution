# AAQuad-Evolution



What is this project ?

	The AAQuad-Evolution is a flight computer I've built to control a Quadcopter. 
	It is NOT autonomous and requires pilot input to operate, the computer takes care of continued stabilisation in flight. 
	It includes Hardware design of a custom PCB, all the C code that runs on the Atmega328pb processor and a PID test framework which can be used to une the algorythm to different mechanical constructions. 
	It is a remake of the Original AAQuad project, which was written in C++. That project got unmanagable due to poor design, testing and implementation.



Directories

	- All C code is contained in the src directory with the exception of main.c which is in the root directory.
	- The schematics folder contains the KiCad design files for the current PCB aswell as some pdf's of the schematics and layout.
	- The PidTests folder contains a simulation framework I built in C to test the PID algorythms aswell as a physical analysis of my Quadcopter.
	- The UnitTests folder contains the source code for all unit tests performed. These are run by ceedling which is managed by the project.yml file in the root directory.
	- The design folder contains some notes and ideas about the problems posed and their solutions aswell as little plans of execution.
	- the Stimuli folder contains stimulus test cases used to test the external and pin-change interrupts to ensure working of the PilotInstructions module.




User instructions 

	Operation mode

		- In operation mode, the quad is flying, or ready to fly. This mode is indicated by a flashing green led on the circuit board.
		The following controls are available in operation mode :
		- The pilot uses the aileron stick to control rotation in the y axis. Each position of the stick is matched to a specific angle, which the computer will attempt to achieve. The default position maps to level.
		- The pilot uses the elevator stick to control rotation in the x axis. Each position of the stick is matched to a specific angle, which the computer will attempt to achieve. The default position maps to level.
		- The pilot uses the rudder stick to control rotation speed in the z axis. each position of the stick matches to a specific angular velocity (in degrees / s) which the omputer will attempt to achieve.The default position maps to 0 degrees/ second.
		- The pilot uses the throttle stick to control the average power of the 4 motors (as of right now, there are no sensors on board to aid in this process)


	Calibration mode

		- There exists a mode in which the sensors recalibrate themselves in an effort to eliminate all systematic error. To enter this mode, the pilot must place his throttle stick to maximum before plugging in the battery.
		- To indicate successfull entering of Calibration mode, a single green led will be solid on the circuit board.
		- While the green led is solid, the pilot should ensure that the quad is in a level position and is not moving in any axis. He should also ensure that all sticks on his radio, except throttle, are at their default position and that there is no trim on any channel.
		- Once the above step is complete, the pilot should lower his throttle stick back to 0. After a slight delay, the green led will begin flashing, indicating that the calibration was successfully saved and that the quad has transitioned to Operation mode.



## Developer: Anthony Berbari


I want to give a shoutout to Anny Wang, who was the original inspiration for the project and who's name appears on the current PCB. 