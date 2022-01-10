#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# userdataの受け渡し
#
# How to use: The following is an example. Please check it in your own environment.
#  $ cd ~/catkin_ws/src/dspl/pumas_navigation/navigation_start/launch
#  $ roslaunch navigation_start navigation.launch
#  $ rosrun trcp_hsr_css navigate5.py
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
import codecs
import yaml

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
try:
    with open('WRS_location.yaml') as file:
        _ARENA = yaml.safe_load(file)
     #   print(_ARENA['shelf']['name'][0])
except Exception as e:
    print('Exception occurred while loading YAML...', file=sys.stderr)
    print(e, file=sys.stderr)
    sys.exit(1)


# 状態の定義
# 指定した場所に移動する
class navi(smach.State):
    def __init__(self, x, y, yaw, timeout=0):
        smach.State.__init__(self, outcomes=['success', 'failure', 'exit'], input_keys=[
                             'counter_in'], output_keys=['counter_out'])
        self._x = x
        self._y = y
        self._yaw = yaw
        self._timeout = timeout

    def execute(self, userdata):
        rospy.loginfo('Counter = %f' % userdata.counter_in)
        if userdata.counter_in < 5:
            userdata.counter_out = userdata.counter_in+1
            try:
                # INSERT YOUR CODE HERE!!
                # FOR EXAMPLE:
                # move to origin
                # x, y, yaw, timeout
                omni_base.go_abs(self._x, self._y, self._yaw, self._timeout)
                return 'success'
            except:
                return 'failure'
        else:
            return 'exit'


def main():
    # 初期姿勢に遷移
    try:
        whole_body.move_to_neutral()
    except:
        rospy.logerr('Fail move_to_neutral')

    # 状態機械の生成
    sm = smach.StateMachine(outcomes=['success', 'failure', 'exit'])
    sm.userdata.sm_counter = 0

    # 状態遷移の追加
    with sm:

        smach.StateMachine.add('Goal1', navi(_ARENA['shelf']['pos']['x'], _ARENA['shelf']['pos']['y'], _ARENA['shelf']['pos']['yaw']),
                               transitions={'success': 'Goal2',
                                            'failure': 'failure',
                                            'exit': 'exit'},
                               remapping={'counter_in': 'sm_counter',
                                          'counter_out': 'sm_counter'}
                               )

        smach.StateMachine.add('Goal2', navi(_ARENA['entrance']['pos']['x'], _ARENA['entrance']['pos']['y'], _ARENA['entrance']['pos']['yaw']),
                               transitions={'success': 'Goal1',
                                            'failure': 'failure',
                                            'exit': 'exit'},
                               remapping={'counter_in': 'sm_counter',
                               'counter_out': 'sm_counter'}
                               )

    # 状態機械の実行
    sm.execute()


if __name__ == '__main__':
    main()
