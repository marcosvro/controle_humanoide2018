# -*- coding:utf-8 -*-

import rospy
import numpy as np
import os
import sys

rospy.init_node('param_setter', anonymous=True)

param_name, value = sys.argv[0:2]

rospy.set_param(param_name, int(value))

