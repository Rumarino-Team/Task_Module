import rospy
import smach
from movement import * 
from enum import Enum

bouy_types  = Enum('Color', ['Abyddos', 'Earth'])
class CheckImageVisibleState(smach.State):
    def __init__(self, image_topic, desired_object_name):
        smach.State.__init__(self, outcomes=['undetected', 'detected', 'preempted'])
        self.image_data = None
        self.image_topic = image_topic
        self.desired_object_name = desired_object_name

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber(self.image_topic, self.image_callback)

        def execute(self, userdata):
            if self.image_data is not None:
                if self.desired_object_name in self.image_data.objects:
                    return 'detected'
                else:
                    return 'undetected'
            else:
                return 'preempted'

