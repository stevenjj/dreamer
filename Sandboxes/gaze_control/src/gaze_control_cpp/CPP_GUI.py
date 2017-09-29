#!/usr/bin/env python

# Dreamer simple GUI
from CPP_GUI_params import *

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
      newFont = QtGui.QFont("Lucida", 20, QtGui.QFont.Bold)

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
                       EYE_PRIORITY_TEST_STRING,
                       HEAD_PRIORITY_TEST_STRING,
                       TRACK_NEAR_PERSON_STRING,
                       FOLLOW_TRAJECTORY_STRING
                       ] 
      
      positions = [(i,j) for i in range(len(self.commands)) for j in range(3)]
           
      for position, name in zip(positions, self.commands):
          button = QtGui.QPushButton(name)
          button.setObjectName('%s' % name)
          button.setFont(newFont)
          button.setStyleSheet("background-color: #5af720")
          button.clicked.connect(self.handleButton)
          grid.addWidget(button, *position)

      mainLayout.addLayout(grid)
      mainLayout.addStretch()
      
      # Show the GUI 
      self.adjustSize()
      self.setWindowTitle("Dreamer CPP version GUI")
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

      elif command == EYE_PRIORITY_TEST_STRING: 
        num_cmd = EYE_PRIORITY_TEST                    

      elif command == HEAD_PRIORITY_TEST_STRING: 
        num_cmd = HEAD_PRIORITY_TEST

      elif command == TRACK_NEAR_PERSON_STRING: 
        num_cmd = TRACK_NEAR_PERSON

      elif command == FOLLOW_TRAJECTORY_STRING: 
        num_cmd = FOLLOW_TRAJECTORY

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
