Dreamer Head Redesign #....5?

2016.06.19
   Started redesigning Dreamer's Head Control Systems a few days ago using TI 
TM4C chips instead of Arduinos. They're vastly more powerful, have many times 
more features, and are cheaper. Most importantly, it occurred to me on Friday 
that we don't need one MCU per one or two joints, as Meka used them. A single 
TM4C has enough features and speed to control all twelve joints for position 
control at several hundred hertz. It's nowhere near as fast as the Ethercat 
system Meka set up, but it doesn't need to be and it should work many, many 
times more reliably.
   v0.1 code was designed to prove all needed features. We have twelve PWM 
outputs for motor control of twelve joints and we use GPIO pins to read twelve 
encoders in parallel. We could add on MOSFETS, as Contelec recommends, or 
tristate buffers, as we did in past redesigns, but the TM4C is fast enough to 
just switch between output and input as needed, so we're planning on doing that 
and not requiring and additional hardware. All encoder signals are produced by 
bit banging, a thousand lines of timing control spins per message.
   v0.2 code is designed to use timer interrupts instead of spinning between 
signal changes. The only actual advantage to this is that the code is more 
attractive. There's nothing wrong with ugly bit banging.
   v0.3 code is a continuation of v0.1. After testing v0.2, it was found that 
nice, clean, well abstracted code was far too slow, and completely procedural
bit banging was necessary. We'll be using this ugly but fast technique from now 
on.

2016.06.21
   Did a lot of work on v0.3. Encoder reading and PWM writing is complete. Can 
do 12-dof P control at 2200Hz with no problems. Have a bit of code left to 
write.
   Need to create:
-I and D error functions so we have proper PID control
-interpreter to accept commands from external controller
-ear lights control
-read e-stop position
   Considering different options for D error calculation. Want it to be as fast
as possible, under 5-10 cycles delay. Will try calculating and storing multiple
point-to-point derivatives and running them through median filter.
Interpreter needs to take position commands from controller and give status
feedback to controller. Want to run TM4C at 2.2KHz in final form if possible, so
will probably not do motion planning on board there. Will merely take position
commands from master. Will provide feedback of joint positions, velocities, and
head e-stop status.
   v0.3 is stable and complete in current form. Archiving, moving to v0.4.

2016.06.22
   Added code to update lights every cycle. Currently only accept one color for 
all lights, both ears. Might add option to write individual colors to individual
LEDs later on, but that takes up a lot of bandwidth and the ears intentionally
diffuse the LED light, so not sure how useful that would be. Making ears
different solid colors might also be useless, because you can't see both of them
at the same time. Have not yet tested signal output because don't have a scope
with me, but it doesn't make anything crash.
   Added code to read hardware e-stop and write software e-stop. Will place
software controlled relay in series with hardware button. Works perfectly.
   Last feature to add is interpreter. Will accept all commands as uint32 
values. This is big enough to contain any single command we want to send. Each 
joint position is <= 14 bits, so we can include two joints per command. Ear 
colors are 24 bits. Software EStop commands are one bit. We'll put a 4-bit 
identifier at the beginning of each command.
   Archiving v0.4 even though lights are untested, moving to v0.5.

2016.06.23
   Finished testing current v0.5. lights and e-stop work well. Will assemble
single dof testbed, fix any bugs, then assemble entire head.

2016.06.27
   Updated ear lights bit timing code. Works, but isn't perfect. Individual bits
are too slow. Instead of running at 800KHz as intended, runs at ~600KHz. Timer 
Handler function is already pretty well optimized for speed. Can't do much more 
there. Lights work fine now, showing single colors at 10-20Hz update rate. If we
want multiple colors or more precise signal timing, we'll need to offload LED 
signals to SPI or I2C instead of bit banging via GPIO. We'll worry about that if
it becomes a problem.
   Fixed typos and truncation errors in a few places. Spent some time trying to 
diagnose problems associated with PA6 and PE0. Realized these pins were damaged,
worked fine on different boards.
   Have now tested PID control on all twelve channels. EStop works, encoders 
work, PID works, PWM works, controller is complete and ready to be installed.
Have to 3d print some parts for ears and assemble optocoupler/level shifter PCB 
to interface TM4C with Maxon drivers. After that's done, will install in 
Dreamer's head.

2016.07.10
   Did a lot more testing, debugging, redesigning, etc. At this point, everything works. We have twelve DOFs running at a determininstic 550Hz. We're reading all the encoders with no problems. We're writing all the PWM control signals. We're reading the e-stop position and controlling the ear lights. We've 3d printed replacement parts for the eyes and ears. We've installed the new eye encoders. We've tested everything on breadboards. All that's left to do is check the PCB designs for the tenth time and order them. When those arrive, we should have Dreamer's head up and running at 100% capacity for the first time ever. Should be cool.
   Changes: For a while we were planning to use two TM4C boards inside the head. One would read encoders and write PWMs. The other would handle motion planning, e-stop, and lights. We decided not to do that. It's unnecessarily complicated. We also tried some interesting memoryless motion planning equations that would ensure that we always have smooth, speed- and acceleration-limited movement. But no matter how that's implemented, it would always restrict our abilities. It makes more sense to have the simplest possible controls built in and sealed off at the MCU level. Currently we're back to the original idea of using GPIO pins to bitbang signals to the ear lights. There's nothing wrong with that, but it uses up more than half of our CPU time. We'll try to offload the light signals to I2C or SPI in the future. That will require new PCBs, so it will be low priority.


