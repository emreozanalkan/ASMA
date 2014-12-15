#!/usr/bin/python
import welcome
import roslib; roslib.load_manifest('asma')
import rospy
import subprocess
import shlex
import psutil
import smach
import smach_ros
import rospy
import actionlib
import roslaunch
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from math import radians, pi
from visualization_msgs.msg import Marker
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from sensor_msgs.msg import Joy
from face_recognition.msg import FaceRecognitionActionFeedback,FRClientGoal
import os
import signal

currTask = 1
nextLoc  = 1
p = rospy.Publisher('cmd_vel', Twist)

class StartRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['StartPatrol'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Start Robot')
        rospy.sleep(1)
        #TODO: Give self explanation do somethign here like speak or move

        return 'StartPatrol'

class StartPatrol(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['DetectTag'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Start Patrol')
        rospy.sleep(1)
        #TODO: Implement patrolling here, like moving the next location, e.g to the 'nextLoc'
        return 'DetectTag'

class DetectTag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Task1','Task2','Task3','Task4','Error','FailDetectTag'])

    def detectARTag(self):
        return 3 #currTask

    def execute(self, userdata):

        rospy.loginfo('Executing state Detect Tag')
        rospy.sleep(1)

        val = self.detectARTag()

        if val == 0 :
            return 'Task1'
        elif val == 1 :
            return 'Task2'
        elif val == 2 :
            return 'Task3'
        elif val == 3 :
            return 'Task4'
        elif val == -1 :
            return 'Error'
        else :
            return 'FailDetectTag'


class FailDetectTag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['TaskCompleted','Error'])
    def informFailDetect(self):
        return

    def execute(self, userdata):
        rospy.loginfo('Executing '+ type(self).__name__)
        rospy.sleep(1)
        self.informFailDetect()
        return 'TaskCompleted'

class Error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['StartPatrol'])

    def informErrorDetected(self):
        return

    def execute(self, userdata):
        rospy.loginfo('Executing '+ type(self).__name__)
        rospy.sleep(1)
        self.informErrorDetected()
        return 'StartPatrol'

class TaskCompleted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Location1','Location2','Location3','Location4','Error'])
    def findNextLocation(self):
        return nextLoc

    def execute(self, userdata):
        rospy.loginfo('Executing '+ type(self).__name__)
        rospy.sleep(1)
        val = self.findNextLocation()
        if val == 0 :
            return 'Location1'
        elif val == 1 :
            return 'Location2'
        elif val == 2 :
            return 'Location3'
        elif val == 3 :
            return 'Location4'
        elif val == -1 :
            return 'Error'
        else :
            return 'Error'


class TaskFailed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['TaskCompleted','Error'])
    def informTaskFailed(self):
        return

    def execute(self, userdata):
        rospy.loginfo('Executing '+ type(self).__name__)
        rospy.sleep(1)
        self.informTaskFailed()
        return 'TaskCompleted'


class Task1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['TaskCompleted','TaskFailed','Error'])
    def performTask(self):
	#TODO: Task1
        speak('Executing '+ type(self).__name__)
        return 1

    def execute(self, userdata):
        rospy.loginfo('Executing '+ type(self).__name__)
        rospy.sleep(1)
        val = self.performTask()
        if val == 1 :
            return 'TaskCompleted'
        elif val == 2 :
            return 'TaskFailed'
        elif val == 0 :
            return 'Error'



class Task2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['TaskCompleted','TaskFailed','Error'])

        self.doJoy=1

    def callbackJoy(self,data):
        print data.buttons[4]
        if(data.buttons[4] == 1) :
            print 'data button 4 1'		
            self.doJoy= 2
            self.joysub.unregister()
            self.processJoy.stop()
            print self.processJoy.is_alive()
        rospy.loginfo(rospy.get_name()+ str(data.axes))
        t = Twist();
        t.linear.x = data.axes[1] / 10;
        t.angular.z = data.axes[2];
        p.publish(t)

    def performTask(self):
        speak('Executing '+ type(self).__name__)

        package = 'joy'
        executable = 'joy_node'
        joyNode = roslaunch.core.Node(package, executable)
        launchJoy = roslaunch.scriptapi.ROSLaunch()
        launchJoy.start()
        self.processJoy = launchJoy.launch(joyNode)

        self.joysub =  rospy.Subscriber("joy", Joy, self.callbackJoy)
        self.doJoy= 1
        while(self.doJoy==1) :
            rospy.sleep(5)
        return 1

    def execute(self, userdata):
        rospy.loginfo('Executing '+ type(self).__name__)
        rospy.sleep(1)
        val = self.performTask()
        if val == 1 :
            return 'TaskCompleted'
        elif val == 2 :
            return 'TaskFailed'
        elif val == 0 :
            return 'Error'

class Task3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['TaskCompleted','TaskFailed','Error'])
    def performTask(self):
        speak('Executing '+ type(self).__name__)
        #TODO: Try catch and then handel error
        self.procServo = subprocess.Popen('roslaunch visp_auto_tracker turtletrack.launch', shell=True, preexec_fn=os.setsid)
        self.procVisp = subprocess.Popen('roslaunch demo_pioneer myservo.launch', shell=True, preexec_fn=os.setsid)
        print self.procServo.pid
        print self.procVisp.pid
        self.pS = psutil.Process(self.procServo.pid)
        self.pV = psutil.Process(self.procVisp.pid)
        rospy.sleep(30)
        #self.procServo.kill()
        #self.procVisp.kill()
        #self.pS.kill()
        #self.pV.kill()
        os.killpg(self.procVisp.pid, signal.SIGTERM)
        os.killpg(self.procServo.pid, signal.SIGTERM)
        return 1

    def execute(self, userdata):
        rospy.loginfo('Executing '+ type(self).__name__)
        rospy.sleep(1)
        val = self.performTask()
        if val == 1 :
            return 'TaskCompleted'
        elif val == 2 :
            return 'TaskFailed'
        elif val == 0 :
            return 'Error'

class Task4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['TaskCompleted','TaskFailed','Error'])
        self.clientMSG = FRClientGoal()
        self.clientMSG.order_id = 1
        self.clientMSG.order_argument = "none"

     def callbackFeedback(self,data):
        print data.feedback.names[0]

    def performTask(self):
        speak('Executing '+ type(self).__name__)
        self.procServ = subprocess.Popen('rosrun face_recognition Fserver _confidence_value:=0.5', shell=True, preexec_fn=os.setsid)
        self.procClient = subprocess.Popen('rosrun face_recognition Fclient', shell=True, preexec_fn=os.setsid)
        print self.procServ.pid
        print self.procClient.pid
        self.ClientGoal = rospy.Publisher('/face_recognition/fr_order', FRClientGoal)
        self.ClientGoal.publish(self.clientMSG)
        self.faceFeedback =  rospy.Subscriber("/face_recognition/feedback", FaceRecognitionActionFeedback, self.callbackFeedback)
        self.pS = psutil.Process(self.procServ.pid)
        self.pC = psutil.Process(self.procClient.pid)
        rospy.sleep(30)
        os.killpg(self.procClient.pid, signal.SIGTERM)
        os.killpg(self.procServ.pid, signal.SIGTERM)
        return 1

    def execute(self, userdata):
        rospy.loginfo('Executing '+ type(self).__name__)
        rospy.sleep(1)
        val = self.performTask()
        if val == 1 :
            return 'TaskCompleted'
        elif val == 2 :
            return 'TaskFailed'
        elif val == 0 :
            return 'Error'


class LocationFailed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Location1','Location2','Location3','Location4','Error'])
    def findNextLocation(self):
        return nextLoc

    def execute(self, userdata):
        rospy.loginfo('Executing '+ type(self).__name__)
        rospy.sleep(1)
        val = self.findNextLocation()
        if val == 0 :
            return 'Location1'
        elif val == 1 :
            return 'Location2'
        elif val == 2 :
            return 'Location3'
        elif val == 3 :
            return 'Location4'
        elif val == -1 :
            return 'Error'
        else :
            return 'Error'

class Location1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['DetectTag','LocationFailed','Error'])
    def moveToPosition(self):
	global nextLoc
	mp.movePoint(nextLoc)
	nextLoc =1;
        return 1

    def execute(self, userdata):
        rospy.loginfo('Executing '+ type(self).__name__)
        rospy.sleep(1)
        val = self.moveToPosition()
        if val == 1 :
            return 'DetectTag'
        elif val == 2 :
            return 'LocationFailed'
        elif val == 0 :
            return 'Error'

class Location2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['DetectTag','LocationFailed','Error'])
    def moveToPosition(self):
	global nextLoc
	mp.movePoint(nextLoc)
	nextLoc =2;
        return 1

    def execute(self, userdata):
        rospy.loginfo('Executing '+ type(self).__name__)
        rospy.sleep(1)
        val = self.moveToPosition()
        if val == 1 :
            return 'DetectTag'
        elif val == 2 :
            return 'LocationFailed'
        elif val == 0 :
            return 'Error'
class Location3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['DetectTag','LocationFailed','Error'])
    def moveToPosition(self):
	global nextLoc
	mp.movePoint(nextLoc)
	nextLoc =3;
        return 1

    def execute(self, userdata):
        rospy.loginfo('Executing '+ type(self).__name__)
        rospy.sleep(1)
        val = self.moveToPosition()
        if val == 1 :
            return 'DetectTag'
        elif val == 2 :
            return 'LocationFailed'
        elif val == 0 :
            return 'Error'
class Location4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['DetectTag','LocationFailed','Error'])
    def moveToPosition(self):
	global nextLoc
	mp.movePoint(nextLoc)
	nextLoc =0;
        return 1

    def execute(self, userdata):
        rospy.loginfo('Executing '+ type(self).__name__)
        rospy.sleep(1)
        val = self.moveToPosition()
        if val == 1 :
            return 'DetectTag'
        elif val == 2 :
            return 'LocationFailed'
        elif val == 0 :
            return 'Error'

class MoveBasePoints():

    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        self.waypoints = list()
        # How big is the square we want the robot to navigate?
        square_size = rospy.get_param("~square_size", 1.0) # meters

        # Create a list to hold the target quaternions (orientations)
        quaternions = list()

        # First define the corner orientations as Euler angles
        euler_angles = (-1.556,-3.104,1.559,-0.003)

        # Then convert the angles to quaternions
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)

        # Create a list to hold the waypoint poses


	oy=2.7;
        ox = 1;
        # Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        self.waypoints.append(Pose(Point(1.227-ox, 5.028-oy, 0.0), quaternions[0]))
        self.waypoints.append(Pose(Point(1.185-ox, 3.260-oy, 0.0), quaternions[1]))
        self.waypoints.append(Pose(Point(-1.245-ox, 2.570-oy, 0.0), quaternions[2]))
        self.waypoints.append(Pose(Point(-1.289-ox, 5.040-oy, 0.0), quaternions[3]))

        # Initialize the visualization markers for RViz
        self.init_markers()

        # Set a visualization marker at each waypoint
        for waypoint in self.waypoints:
            p = Point()
            p = waypoint.position
            self.markers.points.append(p)

        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")

        # Initialize a counter to track waypoints
        i = 0

    def movePoint(self, i):
        self.marker_pub.publish(self.markers)

        # Intialize the waypoint goal
        goal = MoveBaseGoal()

        # Use the map frame to define goal poses
        goal.target_pose.header.frame_id = 'map'

        # Set the time stamp to "now"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the goal pose to the i-th waypoint
        goal.target_pose.pose = self.waypoints[i]

        # Start the robot moving toward the goal
        self.move(goal)



    def move(self, goal):
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(goal)

        # Allow 1 minute to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))

        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")

    def init_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.2
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}

        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker)

        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        self.markers.type = Marker.SPHERE_LIST
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']

        self.markers.header.frame_id = 'map'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

# Classes
# - StartRobot - outcomes=['StartPatrol']
# - StartPatrol - outcomes=['DetectTag']
# - DetectTag - outcomes=['Task1', 'Task2', 'Task3', 'Task4', 'Error', 'FailDetectTag']
# - Error - outcomes=['StartPatrol']
# - FailDetectTag - outcomes=['TaskCompleted', 'Error']
# - TaskCompleted - outcomes=['Location1', 'Location2', 'Location3', 'Location4', 'Error']
# - TaskFailed - outcomes=['TaskCompleted','Error']
# - Task1 - outcomes=['TaskCompleted', 'TaskFailed', 'Error']
# - Task2 - outcomes=['TaskCompleted', 'TaskFailed', 'Error']
# - Task3 - outcomes=['TaskCompleted', 'TaskFailed', 'Error']
# - Task4 - outcomes=['TaskCompleted', 'TaskFailed', 'Error']
# - LocationFailed - outcomes=['Location1', 'Location2', 'Location3', 'Location4', 'Error']
# - Location1 - outcomes=['DetectTag', 'LocationFailed', 'Error']
# - Location2 - outcomes=['DetectTag', 'LocationFailed', 'Error']
# - Location3 - outcomes=['DetectTag', 'LocationFailed', 'Error']
# - Location4 - outcomes=['DetectTag', 'LocationFailed', 'Error']
# - MoveBasePoints

# main
def speak(s):
    #absolute path for this
    soundhandle = SoundClient()
    rospy.sleep(1)
    print s
    soundhandle.say(s,'voice_kal_diphone')
    #may be loop with readline so there is a pause at every line.
    rospy.sleep(1)

def tagDetected(data):
    print data.id
    global currTask
    currTask = data.id

def main():
    rospy.init_node('asma')
    global mp
    mp = MoveBasePoints()
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Stop'])

    rospy.Subscriber("visualization_marker", Marker, tagDetected)


    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('StartRobot', StartRobot(),
                               transitions={'StartPatrol':'StartPatrol'})

        smach.StateMachine.add('StartPatrol', StartPatrol(),
                               transitions={'DetectTag':'DetectTag'})

        smach.StateMachine.add('DetectTag', DetectTag(),
                               transitions={'Task1':'Task1',
					    'Task2':'Task2',
					    'Task3':'Task3',
					    'Task4':'Task4',
					    'Error':'Error',
				            'FailDetectTag':'FailDetectTag'})

        smach.StateMachine.add('Error', Error(),
                               transitions={'StartPatrol':'StartPatrol'})

        smach.StateMachine.add('FailDetectTag', FailDetectTag(),
                               transitions={'TaskCompleted':'TaskCompleted',
					    'Error':'Error'})

        smach.StateMachine.add('TaskCompleted', TaskCompleted(),
                               transitions={'Location1':'Location1',
					    'Location2':'Location2',
					    'Location3':'Location3',
					    'Location4':'Location4',
					    'Error':'Error'})

        smach.StateMachine.add('TaskFailed', TaskFailed(),
                               transitions={'TaskCompleted':'TaskCompleted',
					    'Error':'Error'})

        smach.StateMachine.add('Task1', Task1(),
                               transitions={'TaskCompleted':'TaskCompleted',
					    'TaskFailed':'TaskFailed',
					    'Error':'Error'})

        smach.StateMachine.add('Task2', Task2(),
                               transitions={'TaskCompleted':'TaskCompleted',
					    'TaskFailed':'TaskFailed',
					    'Error':'Error'})

        smach.StateMachine.add('Task3', Task3(),
                               transitions={'TaskCompleted':'TaskCompleted',
					    'TaskFailed':'TaskFailed',
					    'Error':'Error'})

        smach.StateMachine.add('Task4', Task4(),
                               transitions={'TaskCompleted':'TaskCompleted',
					    'TaskFailed':'TaskFailed',
					    'Error':'Error'})

        smach.StateMachine.add('LocationFailed', LocationFailed(),
                               transitions={'Location1':'Location1',
					    'Location2':'Location2',
					    'Location3':'Location3',
					    'Location4':'Location4',
					    'Error':'Error'})

        smach.StateMachine.add('Location1', Location1(),
                               transitions={'DetectTag':'DetectTag',
					    'LocationFailed':'LocationFailed',
					    'Error':'Error'})

        smach.StateMachine.add('Location2', Location2(),
                               transitions={'DetectTag':'DetectTag',
					    'LocationFailed':'LocationFailed',
					    'Error':'Error'})

        smach.StateMachine.add('Location3', Location3(),
                               transitions={'DetectTag':'DetectTag',
					    'LocationFailed':'LocationFailed',
					    'Error':'Error'})

        smach.StateMachine.add('Location4', Location4(),
                               transitions={'DetectTag':'DetectTag',
					    'LocationFailed':'LocationFailed',
					    'Error':'Error'})




        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()

        #rosrun smach_viewer smach_viewer.py to view the states
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

#print 'Welcome'
#welcome.wel('welcome.txt')
