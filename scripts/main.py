#!/usr/bin/python
import welcome
import roslib; roslib.load_manifest('asma')
import rospy
import smach
import smach_ros
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi
from visualization_msgs.msg import Marker


# define state Foo
class Welc(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['s1','s3'])
        self.isDone = 0
        self.mvpoints = MoveBasePoints()


    def execute(self, userdata):
        rospy.sleep(1)
        if self.isDone == 0:
            self.isDone = 1
            rospy.loginfo('Executing state Welcome')
            #welcome.wel('welcome.txt')
            self.mvpoints.movePoint(2)
            return 's1'
        else:
            rospy.loginfo('Already executed state Welcome')
            self.mvpoints.movePoint(3)
            return 's3'



# define state Bar
class StartMe(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['s2'])
        self.mvpoints = MoveBasePoints()

    def execute(self, userdata):
        rospy.loginfo('Executing state Start')
        rospy.sleep(1)
        self.mvpoints.movePoint(1)

        return 's2'


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
# main
def tagDetected(data):
 rospy.loginfo( "I heard %s", data)
def main():
    rospy.init_node('asma')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['STOP'])

    rospy.Subscriber("visualization_marker", Marker, tagDetected)


    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Start', StartMe(),
                               transitions={'s2':'Welcome'})

        smach.StateMachine.add('Welcome', Welc(),
                               transitions={'s1':'Start',
                                            's3':'STOP'})
        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()


    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    welcome.wel('welcome.txt')
    #main()

#print 'Welcome'
#welcome.wel('welcome.txt')
