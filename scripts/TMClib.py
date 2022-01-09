#!/usr/bin/env python
# -*- coding: utf-8 -*-
# TMClib.py
# トヨタが作ったサンプルからの便利な関数集
# How to use: The following is an example. Please check it in your own environment.
#
# Author: Hiroyuki Okada
# Date last modified: 1/1/2022
# Date created: 1/1/2022
# Software License Agreement (BSD License)
# Copyright (c) 2021, Tamagawa University.
# All rights reserved.
import math
import os
import sys

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import WrenchStamped
import rospy
from sensor_msgs.msg import JointState
from tmc_control_msgs.msg import (
    GripperApplyEffortAction,
    GripperApplyEffortGoal
)
from tmc_manipulation_msgs.srv import (
    SafeJointChange,
    SafeJointChangeRequest
)
from tmc_msgs.msg import (
    TalkRequestAction,
    TalkRequestGoal,
    Voice
)
from std_msgs.msg import ColorRGBA


class HSRled():
    # LEDの設定
    _LED_INPUT_DATA_SIZE = 256

    def __init__(self, _DEFAULT_COLOR=ColorRGBA(g=0.2, b=0.6)):
        self._DEFAULT_COLOR = _DEFAULT_COLOR

        # Create Publisher to change status led color
        status_led_topic = '/hsrb/command_status_led_rgb'
        self.led_pub = rospy.Publisher(status_led_topic,
                                       ColorRGBA, queue_size=100)

        # Wait for connection
        while self.led_pub.get_num_connections() == 0:
            rospy.sleep(0.1)
        return

    def changeGradual(self):
        # 50Hz is enough frequency to do gradual color change
        rate = rospy.Rate(50)

        # Value r changes gradually (value g and b are fixed)
        color = self._DEFAULT_COLOR
        for num in range(self._LED_INPUT_DATA_SIZE):
            color.r = num / float(self._LED_INPUT_DATA_SIZE - 1)
            self.led_pub.publish(color)
            rate.sleep()


def compute_difference(pre_data_list, post_data_list):
    """引数で与えた同じ長さのリスト間の差分を計算する関数．
    物を把持する前後での力成分の差分を計算するためなどに使う．

    Args:
        pre_data_list ([type]): 先
        post_data_list ([type]): 後

    Raises:
        ValueError: リストの長さが異なる

    Returns:
        float : 計算された差分
    """
    if (len(pre_data_list) != len(post_data_list)):
        raise ValueError('Argument lists differ in length')
    # Calcurate square sum of difference
    square_sums = sum([math.pow(b - a, 2)
                       for (a, b) in zip(pre_data_list, post_data_list)])
    return math.sqrt(square_sums)


class ForceSensorCapture(object):
    """力覚センサデータの購読と保持"""
# アクションサーバへの接続待ち時間
    _CONNECTION_TIMEOUT = 10.0

    def __init__(self):
        self._force_data_x = 0.0
        self._force_data_y = 0.0
        self._force_data_z = 0.0

        # Subscribe force torque sensor data from HSRB
        ft_sensor_topic = '/hsrb/wrist_wrench/raw'
        self._wrist_wrench_sub = rospy.Subscriber(
            ft_sensor_topic, WrenchStamped, self.__ft_sensor_cb)

        # Wait for connection
        try:
            rospy.wait_for_message(ft_sensor_topic, WrenchStamped,
                                   timeout=self._CONNECTION_TIMEOUT)
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

    def get_current_force(self):
        """保持している力覚センサデータにアクセスする関数

        Returns:
            float list : センサにかかる力成分の各軸 (x, y, z) ごとの値
        """
        return [self._force_data_x, self._force_data_y, self._force_data_z]

    def __ft_sensor_cb(self, data):
        self._force_data_x = data.wrench.force.x
        self._force_data_y = data.wrench.force.y
        self._force_data_z = data.wrench.force.z


class JointController(object):
    """
    姿勢遷移サービスとグリッパ制御アクションを利用
    姿勢とグリッパの制御
    """
    # アクションサーバへの接続待ち時間
    _CONNECTION_TIMEOUT = 10.0

    def __init__(self):
        joint_control_service = '/safe_pose_changer/change_joint'
        grasp_action = '/hsrb/gripper_controller/grasp'
        self._joint_control_client = rospy.ServiceProxy(
            joint_control_service, SafeJointChange)

        self._gripper_control_client = actionlib.SimpleActionClient(
            grasp_action, GripperApplyEffortAction)

        # Wait for connection
        try:
            self._joint_control_client.wait_for_service(
                timeout=self._CONNECTION_TIMEOUT)
            if not self._gripper_control_client.wait_for_server(rospy.Duration(
                    self._CONNECTION_TIMEOUT)):
                raise Exception(grasp_action + ' does not exist')
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

    def move_to_joint_positions(self, goal_joint_states):
        """姿勢遷移サービスを呼び出す関数
        このサービスを使うとHSRを目標姿勢まで自己干渉しないように姿勢遷移させることができます

        Args:
            goal_joint_states ([type]): 目標姿勢

        Returns:
            [type]: [description]
        """

        try:
            req = SafeJointChangeRequest(goal_joint_states)
            res = self._joint_control_client(req)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return False
        return res.success

    def grasp(self, effort):
        """物を把持するためにグリッパ制御アクションを呼び出す関数
            引数で与えられたトルクの大きさに到達するまでグリッパを締め付けます。
        Args:
            effort ([type]): トルクの大きさ，[Nm]

        Returns:
            bool: True（成功），False（失敗）
        """
        goal = GripperApplyEffortGoal()
        goal.effort = effort

        # Send message to the action server
        if (self._gripper_control_client.send_goal_and_wait(goal) ==
                GoalStatus.SUCCEEDED):
            return True
        else:
            return False


class Speaker(object):
    """
    HSRの発話を管理するためのクラス
    PCの環境変数を使って HSRの発話言語を切り替えています。
    以下のコマンドで 一時的にコンピュータの環境変数を変更してください。
    - HSRの使用言語が日本語の場合
      <hsrb>~$ export LANG=ja_JP.UTF-8
    - HSRの使用言語が英語の場合
      <hsrb>~$ export LANG=en_US.UTF-8
    """
    # アクションサーバへの接続待ち時間
    _CONNECTION_TIMEOUT = 10.0

    def __init__(self):
        talk_action = '/talk_request_action'
        self._talk_request_client = actionlib.SimpleActionClient(
            talk_action, TalkRequestAction)
        # Wait for connection
        try:
            if not self._talk_request_client.wait_for_server(
                    rospy.Duration(self._CONNECTION_TIMEOUT)):
                raise Exception(talk_action + ' does not exist')
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

        # Detect robot's language
        if os.environ['LANG'] == 'ja_JP.UTF-8':
            self._lang = Voice.kJapanese
        else:
            self._lang = Voice.kEnglish

    def get_language(self):
        """使用している言語を得る
        Returns:
            int: 0（日本語），1（英語）
        """
        return self._lang

    def speak_sentence(self, sentence):
        """与えられた文字列を発話する
             Index 0: in Japanese, 1: in English
             _EXPLAIN1 = [u'グリッパの間に重さをはかりたいものを持ってきてください',
                          u'Please set the object between my gripper']
             _EXPLAIN2 = [u'グリッパを閉じます', u'I close my hand now']
             _ANSWER = [u'これは{0}グラムです', u'This is {0} gram']
            .speak_sentence(_EXPLAIN1[speaker.get_language()])             
        Args:
            sentence (string): 発話する文字列

        Returns:
            bool: True or False
        """
        goal = TalkRequestGoal()
        goal.data.language = self._lang
        goal.data.sentence = sentence

        if (self._talk_request_client.send_goal_and_wait(goal) ==
                GoalStatus.SUCCEEDED):
            return True
        else:
            return False
