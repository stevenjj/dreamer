Launch sequence:
This package depends on 
ar_track_alvar, freenect_launch drivers, dn_object_detect, and node_people_bounding_boxes

# Run people detection first
roslaunch dn_object_detect objdetect.launch
rosrun people_bounding_boxes node_people_bounding_boxes 

# Prepare Rviz
roslaunch gaze_control init_demo_setup.launch

# When ready, run orientation_control.py


# To run the head control, first initialize the simulator node
roslaunch gaze_control init_demo_setup.launch

# Run both the command GUI and high level controller
rosrun gaze_control GUI_dreamer.py
rosrun gaze_control dreamer_high_level_control.py

# Connect to the microcontroller
rosrun gaze_control UART_com_v2.py

# Start remote control
rosrun gaze_control RunRemCtrl.py

# To use the low level fifo that saves joint commands, run a behavior while making sure low level control is off
# After the behavior is complete, press the "Low Level Publish" button
# "State to Idle" will clear the low level fifo
