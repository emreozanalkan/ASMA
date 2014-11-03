#!/usr/bin/python
import welcome
import roslib; roslib.load_manifest('asma')
import rospy
import smach
import smach_ros

# define state Foo
class Welc(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.isDone = 0

    def execute(self, userdata):
        rospy.sleep(1)
        if self.isDone  == 0:
            self.isDone  =  1
            rospy.loginfo('Executing state Welcome')
            welcome.wel('welcome.txt')
            return 'outcome1'
        else:
            rospy.loginfo('Already executed state Welcome')
            return 'outcome2'


# define state Bar
class StartMe(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartMe')
        rospy.sleep(1)

        return 'outcome2'


# main
def main():
    rospy.init_node('asma')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['STOP'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('StartMe', StartMe(),
                               transitions={'outcome2':'Wel_come'})

        smach.StateMachine.add('Wel_come', Welc(),
                               transitions={'outcome1':'StartMe',
                                            'outcome2':'STOP'})


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()

#print 'Welcome'
#welcome.wel('welcome.txt')
