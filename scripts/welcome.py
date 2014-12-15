#!/usr/bin/env python
import sys
import os
import roslib
roslib.load_manifest('sound_play')
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

def wel(filename):
    #absolute path for this
    print filename
    fileloc = str( str(os.path.dirname(__file__)) +'/'+ filename)
    f = open(fileloc, 'r')
    #rospy.init_node('say', anonymous = True)
    soundhandle = SoundClient()
    rospy.sleep(1)
    s = f.read()
    print s
    soundhandle.say(s,'voice_kal_diphone')
    #may be loop with readline so there is a pause at every line.
    rospy.sleep(1)
