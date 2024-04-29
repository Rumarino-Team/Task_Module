import rospy
import smach
from tasks.src.movement import * # Import the UpdatePoseToObjectState class
import tasks.src.utils as utils

 # Import the CheckImageVisibleState class
def main(debug = False):
    rospy.init_node('multidof6_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', 'aborted', 'preempted'])

    # Define the MoveIt! move group
    move_group_name = "move_group"
    object_topic = rospy.get_param("~object_topic")
    object_class = rospy.get_param("~object_class")



    with sm:
        # Fix the Todo in lower depth state
        smach.StateMachine.add('Lower_Depth', LowerDepth(move_group_name),
                               transitions={'success': 'CHECK_IMAGE_VISIBLE',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'},)
        smach.StateMachine.add('CHECK_IMAGE_VISIBLE', utils.CheckImageVisibleState(object_topic, 'gate'),
                               transitions={'undetected': 'Rotate90DegreesState',
                                            'detected': 'Survey_Gate',
                                            'preempted': 'preempted'},)
        smach.StateMachine.add('Rotate90DegreesState', Rotate90DegreesState(move_group_name),
                                 transitions={'success': 'CHECK_IMAGE_VISIBLE',
                                              'aborted': 'aborted',
                                              'preempted': 'preempted'},)
        #TODO:: Make a state machine that take account the two types of gates 
        smach.StateMachine.add('SURVEY_GATE', UpdatePoseToObjectState(move_group_name, object_topic, object_class),
                                 transitions={'success': 'completed',
                                              'aborted': 'aborted',
                                              'preempted': 'preempted'},)

    # Execute the state machine
    if not debug:
        outcome = sm.execute()
    else:
        return sm

if __name__ == '__main__':
    main(debug=False)