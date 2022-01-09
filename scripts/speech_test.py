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

_EXPLAIN1 = [u'グリッパの間に重さをはかりたいものを持ってきてください',
             u'Please set the object between my gripper']


def main():
    speaker = Speaker()
    speaker.speak_sentence(_EXPLAIN1[speaker.get_language()])

    pass


if __name__ == '__main__':
    main()