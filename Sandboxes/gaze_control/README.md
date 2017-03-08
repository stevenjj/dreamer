Launch sequence:
This package depends on 
ar_track_alvar, freenect_launch drivers, dn_object_detect, and node_people_bounding_boxes

# Run people detection first
roslaunch dn_object_detect objdetect.launch
rosrun people_bounding_boxes node_people_bounding_boxes 

# Prepare Rviz
roslaunch gaze_control init_demo_setup.launch

# When ready, run orientation_control.py