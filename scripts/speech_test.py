#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# How to use: The following is an example. Please check it in your own environment.
#  $ rosrun okd_tutorial speech_test.py
#
# Author: Hiroyuki Okada
# Date last modified: 2022/1/9
# Date created: 2022/1/1
# Software License Agreement (BSD License)
# Copyright (c) 2021, Tamagawa University.
# All rights reserved.
#
# from TMC 
# created by Okada
#
from TMClib import *
from spoken_text import *

_EXPLAIN1 = [u'発話のテストプラグラムです．テストをしています．',
             u'Please set the object between my gripper']


def main():
    rospy.init_node('hsrb_speech_test')
    speaker = Speaker()
    speaker.speak_sentence(_EXPLAIN1[speaker.get_language()])
    #発話が終わるまで待ち合わせる
    print("end")



if __name__ == '__main__':
    main() 