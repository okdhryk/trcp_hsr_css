#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# How to use: The following is an example. Please check it in your own environment.
#  $ cd ~/catkin_ws/src/dspl/pumas_navigation/navigation_start/launch
#  $ roslaunch navigation_start navigation.launch
#  $ rosrun okd_tutorial navigate3.py
#
# Author: Hiroyuki Okada
# Date last modified: 2022/1/9
# Date created: 2022/1/1
# Software License Agreement (BSD License)
# Copyright (c) 2021, Tamagawa University.
# All rights reserved.

import rospy
import smach
import smach_ros
import tf2_ros
from tf.transformations import quaternion_matrix, quaternion_from_euler, quaternion_from_matrix
from geometry_msgs.msg import Vector3Stamped, Pose, PoseStamped, PointStamped
from geometry_msgs.msg import WrenchStamped, Twist
import math
import numpy as np
import random
import traceback
import sys
import math
import os


import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import WrenchStamped
import rospy
from sensor_msgs.msg import JointState
from tmc_control_msgs.msg import (
    GripperApplyEffortAction,
    GripperApplyEffortGoal
)

# TMC
import hsrb_interface
from hsrb_interface import geometry
from tmc_msgs.msg import (
    TalkRequestAction,
    TalkRequestGoal,
    Voice
)

# TRCP
from common import speech
#from common.smach_states import *
from common import rospose_to_tmcpose

# from TMC 
# created by Okada
from TMClib import *

# Pumas
from takeshi_tools.nav_tool_lib import nav_module

# ロボット(HSR)機能を使うための準備
robot = hsrb_interface.Robot()
whole_body = robot.get("whole_body")

# TMC or Pumas
#omni_base = robot.get("omni_base")
omni_base = nav_module("pumas")  # New initalisation (Pumas)


# 移動する場所の定義
_ARENA = {}
_ARENA['shelf'] = {'pos':{'x':2.325, 'y':4.102, 'yaw':3}, 'name':[u'棚', u'shelf']}
_ARENA['entrance']  = {'pos':{'x':1.0, 'y':1.0, 'yaw':0}, 'name':[u'入口', u'entrance']}



# 状態の定義
# 指定した場所に移動する
class navi(smach.State):
    def __init__(self, x, y, yaw, timeout=0):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self._x = x
        self._y = y
        self._yaw = yaw
        self._timeout = timeout

    def execute(self, userdata):
        try:
            # INSERT YOUR CODE HERE!!
            # FOR EXAMPLE:

            # move to origin
            # x, y, yaw, timeout
            omni_base.go_abs(self._x, self._y, self._yaw, self._timeout)
          
            return 'success'
        except:
            return 'failure'


def main():
    # 初期姿勢に遷移
    try:
        whole_body.move_to_neutral()
    except:
        rospy.logerr('Fail move_to_neutral')

    print()

    # 状態機械の生成
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    # 状態遷移の追加
    with sm:

        smach.StateMachine.add('Goal1', navi(_ARENA['shelf']['pos']['x'], _ARENA['shelf']['pos']['y'], _ARENA['shelf']['pos']['yaw']),
                               transitions={'success': 'Goal2', 'failure': 'failure'})
    
        smach.StateMachine.add('Goal2', navi(_ARENA['entrance']['pos']['x'], _ARENA['entrance']['pos']['y'], _ARENA['entrance']['pos']['yaw']),
                               transitions={'success': 'Goal1', 'failure': 'failure'})
    
    
    # 状態機械の実行
    sm.execute()


if __name__ == '__main__':
    main()
