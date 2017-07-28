#!/usr/bin/env python
# GUI Command list
GUI_CMD_TOPIC = 'GUI_cmd_int'
INVALID_CMD = -2
NO_COMMAND, NO_COMMAND_STRING = -1, "No Command"
LOW_LEVEL_OFF, LOW_LEVEL_OFF_STRING  = 0, "Low Level OFF"
LOW_LEVEL_ON,  LOW_LEVEL_ON_STRING   = 1, "Low Level ON"
STATE_TO_IDLE, STATE_TO_IDLE_STRING  = 2, "State to Idle"
GO_HOME, 	   GO_HOME_STRING 		 = 3, "Go Home"
DO_SQUARE_FIXED_EYES,   	  DO_SQUARE_FIXED_EYES_STRING 		   		 = 4,  "Square w/ Fixed Eyes" # Eye Priority
DO_SQUARE_FIXED_HEAD,   	  DO_SQUARE_FIXED_HEAD_STRING 		 		 = 5,  "Square w/ Fixed Head" # Head Priority

TRACK_NEAR_PERSON,	    	 	  TRACK_NEAR_PERSON_STRING   		         = 6,  "Track Near Person" # Eye Priority
TRACK_NEAR_PERSON_EYES,	    	  TRACK_NEAR_PERSON_EYES_STRING 	         = 7,  "Track Near Person Eyes" # Eye Priority
TRACK_NEAR_PERSON_BEST,	    	  TRACK_NEAR_PERSON_BEST_STRING 	         = 8,  "Track Near Person Best" # Eye Priority

AVOID_NEAR_PERSON,	    	  AVOID_NEAR_PERSON_STRING   		         = 9, "Avoid Near Person" # Eye Priority
DO_WAYPOINT_TRAJ,   	 	  DO_WAYPOINT_TRAJ_STRING   		         = 10, "Go To Waypoints" # Eye Priority

CIRCLE_TRAJ,   	 	  CIRCLE_TRAJ_STRING   		         = 11, "Trace Circle" # Head Priority
LOW_LEVEL_PUBLISH,   	 	  LOW_LEVEL_PUBLISH_STRING   		         = 12, "Publish To Low Level" 
CALC_BEHAV_AT_RATE,			CALC_BEHAV_AT_RATE_STRING		= 13, "Calc Joint at Rate"
