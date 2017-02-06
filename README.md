# Dreamer Head Redesign, v57.0+

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;As of 2017, Dreamer's head is running pretty nicely. Twelve DOFs are all running smoothly and reliably and at this point in time we're working on mid/high-level software.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;The onboard MCU can do everything we will need it to do in the long term, which means communicating in both directions with an external controller. Currently the MCU starts up, runs a random motion demo, and waits for external commands. Once external commands are received, it waits for further commands forever. We plan on improving the onboard code so that onboard demos can be restarted if desired without resetting the MCU.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;As for external control, we have a few simple remote control scripts, some of which includes machine vision components. We pan on writing a much more comprehensive external framework that provides data visualization tools for joint tuning, the ability to remotely update low level gains, joint ranges, and other onboard configuration values. This would require us to store all config values in the onboard EEPROM, so we'd need to come up with a safety system to ensure that we know what values we're using at any given time.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Our remote control program should also include much more powerful motion planning tools, or at least an interface that allows us to connect the head to a sophisticated motion planning framework such as ROS, Matlab, or other arbitrary Python code.
