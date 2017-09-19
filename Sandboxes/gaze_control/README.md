###Launch sequence:
This package depends on 
ar_track_alvar, freenect_launch drivers, dn_object_detect, and node_people_bounding_boxes
# Enable Serial Communication
## Find the port number and enable serial commands to be sent
### For Mac:
````
ls /dev/tty.*
#Typically it is pretty obvious which device name is Dreamer's head. Mine shows up to be
/dev/tty.usbmodem0E21EE91
````
Change serial:
````
ser = serial.Serial('/dev/tty.usbmodem0E21EE91', 115200, parity='N')
````

### For Linux:
````
ls /dev/tty*
# In linux it's not quite obvious, but for my machine this is the device name:
/dev/ttyACM0
````
To enable serial output on your linux computer, do:
```
sudo adduser USERNAME dialout
````
and then log out and log back in.

Now change serial to:
````
ser = serial.Serial('/dev/ttyACM0', 115200, parity='N')
````

# ROS Commands:
### Run people detection
````
roslaunch dn_object_detect objdetect.launch
rosrun people_bounding_boxes node_people_bounding_boxes 
````
### Prepare Rviz
````
roslaunch gaze_control init_demo_setup.launch
````
## To run the head control, first initialize the simulator node
````
roslaunch gaze_control init_demo_setup.launch
````
### Run both the command GUI and high level controller
````
rosrun gaze_control GUI_dreamer.py
rosrun gaze_control dreamer_high_level_control.py
````
### Connect to the microcontroller
````
rosrun gaze_control UART_com_v2.py
````
### Start remote control
````
rosrun gaze_control RunRemCtrl.py
````

# Prepare full joint command then send
To use the low level fifo that saves joint commands, run a behavior while making sure low level control is off
After the behavior is complete, press the "Low Level Publish" button
"State to Idle" will clear the low level fifo
