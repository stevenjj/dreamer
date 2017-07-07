#!/usr/bin/env python

# Dreamer simple GUI
from GUI_params import *

import rospy
import sys
#import rospkg
import yaml
from std_msgs.msg import String
from std_msgs.msg import Int8
from PyQt4 import QtGui, QtCore

class DreamerGui(QtGui.QWidget):

  def __init__(self):
      QtGui.QWidget.__init__(self)
      newFont = QtGui.QFont("Helvetica", 24, QtGui.QFont.Bold)

      # Add a main layout
      mainLayout = QtGui.QVBoxLayout(self)
      #mainLayout->setMeubBar(menuBar)
      # Add buttons with the commands
      grid = QtGui.QGridLayout()
      grid.setSpacing(20)

      # Initialize rosnode
      rospy.init_node("dreamer_gui") 

      #rospack = rospkg.RosPack()
      default_pub_topic = GUI_CMD_TOPIC

      # Set Commands
      self.commands = [LOW_LEVEL_OFF_STRING,
                       LOW_LEVEL_ON_STRING, 
                       STATE_TO_IDLE_STRING, 
                       GO_HOME_STRING,
                       DO_SQUARE_FIXED_EYES_STRING,
                       DO_SQUARE_FIXED_HEAD_STRING,
                       TRACK_NEAR_PERSON_STRING,
                       TRACK_NEAR_PERSON_EYES_STRING,
                       TRACK_NEAR_PERSON_BEST_STRING,                       
                       AVOID_NEAR_PERSON_STRING,
                       DO_WAYPOINT_TRAJ_STRING,
                       CIRCLE_TRAJ_STRING,
                       LOW_LEVEL_PUBLISH_STRING
                       ] 
      
      positions = [(i,j) for i in range(len(self.commands)) for j in range(3)]
           
      for position, name in zip(positions, self.commands):
          button = QtGui.QPushButton(name)
          button.setObjectName('%s' % name)
          button.setFont(newFont)
          button.setStyleSheet("background-color: #FFA500")
          button.clicked.connect(self.handleButton)
          grid.addWidget(button, *position)

      mainLayout.addLayout(grid)
      mainLayout.addStretch()
      
      # Show the GUI 
      self.adjustSize()
      self.setWindowTitle("GUI Dreamer")
      self.move(400,400)
      self.show()
      self.raise_()

      # # Create the publisher to publish the commands to
      self.pub = rospy.Publisher(default_pub_topic, Int8, queue_size=1)

      rospy.loginfo("Finished initializing Dreamer GUI")

  # Button handler after its clicked
  def handleButton(self):
      clicked_button = self.sender()

      num_cmd = INVALID_CMD
      send_command = False
      # # Publish everytime a command is selected from the combo box
      command = str(clicked_button.objectName())


      if command in self.commands:
        send_command = True

      if command == LOW_LEVEL_OFF_STRING: 
        num_cmd = LOW_LEVEL_OFF

      elif command == LOW_LEVEL_ON_STRING: 
        num_cmd = LOW_LEVEL_ON 

      elif command == STATE_TO_IDLE_STRING: 
        num_cmd = STATE_TO_IDLE

      elif command == GO_HOME_STRING: 
        num_cmd = GO_HOME

      elif command == DO_SQUARE_FIXED_EYES_STRING: 
        num_cmd = DO_SQUARE_FIXED_EYES                        

      elif command == DO_SQUARE_FIXED_HEAD_STRING: 
        num_cmd = DO_SQUARE_FIXED_HEAD

      elif command == TRACK_NEAR_PERSON_STRING: 
        num_cmd = TRACK_NEAR_PERSON

      elif command == TRACK_NEAR_PERSON_EYES_STRING: 
        num_cmd = TRACK_NEAR_PERSON_EYES        

      elif command == TRACK_NEAR_PERSON_BEST_STRING: 
        num_cmd = TRACK_NEAR_PERSON_BEST
      
      elif command == AVOID_NEAR_PERSON_STRING: 
        num_cmd = AVOID_NEAR_PERSON

      elif command == DO_WAYPOINT_TRAJ_STRING: 
        num_cmd = DO_WAYPOINT_TRAJ
      
      elif command == CIRCLE_TRAJ_STRING: 
        num_cmd = CIRCLE_TRAJ

      elif command == LOW_LEVEL_PUBLISH_STRING: 
        num_cmd = LOW_LEVEL_PUBLISH
      
      else:
        num_cmd = INVALID_CMD
      
      rospy.loginfo(command)

      if send_command:
        msg = Int8()
        msg.data = num_cmd
        self.pub.publish(msg)

def gui_start():
    app = QtGui.QApplication(sys.argv)
    sg = DreamerGui()
    sys.exit(app.exec_())


gui_start()
