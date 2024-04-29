import rospy
import smach
from tasks.src.movement import * # Import the UpdatePoseToObjectState class
import tasks.src.utils as utils


# Return the State Machine for Unit Testing purposes
def main(debug=True):
    rospy.init_node('multidof6_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', 'aborted', 'preempted'])

    # Define the MoveIt! move group
    move_group_name = "move_group"
    object_topic = rospy.get_param("~object_topic")
    image_topic = rospy.get_param("~image_topic")
    object_class = rospy.get_param("~object_class")



    with sm:
        smach.StateMachine.add("Check_Bouy", utils.CheckImageVisibleState(object_topic, object_class),
                               transitions={'undetected': 'aborted',
                                            'detected': 'Follow_Path',})
        smach.StateMachine.add('Go_to_Buoy', UpdatePoseToObjectState(move_group_name, object_topic, object_class),
                               transitions={'success': 'success',
                                            'aborted': 'aborted',
                                            'preempted': 'Check_Path_Pinger'},)
    # Execute the state machine
    if not debug:
        outcome = sm.execute()
    else:
        return sm


if __name__ == '__main__':
    main(debug=False)