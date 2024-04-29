# This is the example state machine  for
# constructing the others machines with their respective 
# missions.

from data import shared_data, initialize_subscribers
import smach
import rospy
from movement import UpdatePoseState, UpdatePoseToObjectState
from geometry_msgs.msg import Pose, PoseStamped, Point
import os
"""
--------------------------------------------
DEFINE YOUR CUSTOM STATES IN HERE IF NECESSARY
--------------------------------------------
"""
# Define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failure'])

    def execute(self, userdata):
        print("Try Again to detect object")
        return "failure"
    




class YourStateMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success', 'failure'])
        # shared_data is initialized inisde the initialize_subscribers() function
        # This is a global variable that is shared between all the states.
        self.userdata.shared_data = shared_data

        # Implement a search for the name of the object e.g Path
        # Search for Point position 
        # Added to the Target Pose for giving it to the  StateMachine
        self.userdata.shared_data.zed_data["objects_stamped"]


        with self:
            smach.StateMachine.add('move_to_object', UpdatePoseState( edge_case_callback= lambda x: False), transitions={'success':'success', 'aborted':'failure', 'edge_case_detected':'failure'})


# Running the state machine
if __name__ == '__main__':
    rospy.init_node('your_state_machine_node')
    file_path = os.path.join(os.path.dirname(__file__), '../../config/topics.yaml')
    initialize_subscribers(file_path)
    sm = YourStateMachine()
    outcome = sm.execute()
