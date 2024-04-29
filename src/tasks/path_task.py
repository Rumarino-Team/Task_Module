import rospy
import smach
from tasks.src.movement import * # Import the UpdatePoseToObjectState class
import tasks.src.utils as utils


class FollowPathState(UpdatePoseToObjectState):
    def __init__(self, move_group_name, object_topic, desired_object_name):
        super.__init__(self, move_group_name, object_topic, desired_object_name)
        smach.State.__init__(self, outcomes=['success', 'aborted', 'preempted'])
        self.move_group_name = move_group_name
        self.object_sub = rospy.Subscriber(self.object_topic, self.pingerTrack)
        self.pingerTrack = []
        self.maxPingers = 5
    
    def pingerTrack(self,data):
        for object in data.objects:
            if data.name == "Pinger" and object.label_id not in self.pingerTrack:
                self.object_data = data.pose
                self.pingerTrack.append(object.label_id)
    def execute(self, userdata):
        move_group = MoveGroupCommander(self.move_group_name)

        # Get the current pose
        current_pose = move_group.get_current_pose().pose

        # Set the target pose for the end effector to the object pose

        # Plan and execute the movement
        move_group.set_pose_target(self.objevct_data)
        plan = move_group.go(wait=True)
        if self.object_data is not None:
            if plan:
                return 'success'
            else:
                return 'aborted'
            # 1) All pingers were detected
            # 2) No Pingers were detected
        else:
            if len(self.pingerTrack) == self.maxPingers:
                return 'success'
            else:
                return 'preempted'


def main():
    rospy.init_node('multidof6_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', 'aborted', 'preempted'])

    # Define the desired object name
    desired_object_name = "target_object"

    # Define the object topic name
    object_topic = "/your_object_topic"  # Replace with the actual topic name

    # Define the MoveIt! move group
    move_group_name = "move_group"


    with sm:
        smach.StateMachine.add("Check_Path_Pinger", utils.CheckImageVisibleState(image_topic, 'Pinger'),
                               transitions={'undetected': 'aborted',
                                            'detected': 'Follow_Path',})
        smach.StateMachine.add('Follow_Path', FollowPathState(move_group_name, object_topic, desired_object_name),
                               transitions={'success': 'success',
                                            'aborted': 'aborted',
                                            'preempted': 'Check_Path_Pinger'},)
    # Execute the state machine
    outcome = sm.execute()

if __name__ == '__main__':
    main()