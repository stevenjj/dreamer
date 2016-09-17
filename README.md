# Dreamer Head Redesign, v57.0+

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;We went back to work redesigning Dreamer's head and this time we're committing to finishing it. This project aims to run all twelve DOFs using the simplest possible combination of control hardware. Low cost would be nice. Most importantly, the final product needs to be reliable. We want the head running at 100% capacity, which means four neck actuators, four eye actuators, four ear actuators, two ear lights, and two eye cameras all running continuously and predictably.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;The project uses TI TM4C Launchpad MCUs, Maxon ESCON motor drivers, the original Contelec Vert-X magnetic encoders, and custom PCBs. This all fits neatly into the original head space, is powered from the Meka A2's power systems, and is controlled via UART across a single USB cable. All onboard code is written in C and ARM Cortex-M4 Assembly. 
