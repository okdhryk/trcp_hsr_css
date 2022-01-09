#!/usr/bin/env python
#発話文の例文
#日本語と英語
#
# How to use: The following is an example. Please check it in your own environment.
# 
# _ANSWER = [u'これは{0}グラムです', u'This is {0} grams']
# weight=90.1
# speaker.speak_sentence(_ANSWER[speaker.get_language()].format(weight))
# print(_ANSWER[0].format(weight))
# print(_ANSWER[1].format(weight))
#
# Author: Hiroyuki Okada
# Date last modified: 2020/1/4
# Date created: 2020/1/4
# Software License Agreement (BSD License)
# Copyright (c) 2021, Tamagawa University.
# All rights reserved.

# 重さを答える
_Instructions_for_weighing1 = [u'グリッパの間に重さをはかりたいものを持ってきてください',u'Please set the object between my gripper']
_Instructions_for_weighing2 = [u'グリッパを閉じます', u'I close my hand now']
_ANSWER_WEIGHT = [u'これは{0}グラムです', u'This is {0} grams']

# 長さを答える
_ANSWER_LENGTH = [u'これは{0}メートルです', u'This is {0} meters']

# 挨拶
_HELLO = [u'こんにちは', u'Hello']
_MORNING = [u'おはようございます', u'Good morning']
_AFTERNOON = [u'こんにちは', u'Good afternoon']
_EVENING = [u'こんばんわ', u'Good evening']

# 〜しました
_REACH = [u'{0}に到着しました', u'just reached the  {0}']


# 失敗しました
