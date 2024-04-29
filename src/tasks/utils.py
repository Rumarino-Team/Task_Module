import rospy
import smach
from tasks.src.movement import * 
from enum import Enum
import math

bouy_types  = Enum('Color', ['Abyddos', 'Earth'])
class CheckImageVisibleState(smach.State):
    def __init__(self, image_topic, desired_object_name):
        smach.State.__init__(self, outcomes=['undetected', 'detected', 'preempted'])
        self.image_data = None
        self.image_topic = image_topic
        self.desired_object_name = desired_object_name


        def execute(self, userdata):
            if self.image_data is not None:
                if self.desired_object_name in self.image_data.objects:
                    return 'detected'
                else:
                    return 'undetected'
            else:
                return 'preempted'


# TODO
def BasicMovementEdgeCase(shared_data,**kwargs):
    for key, value in kwargs.items():
        print("{} -> {}".format(key, value)) 

def quaternions_angle_difference(q1, q2):
    """
    Calculate the angle between two quaternions
    """
    dot = q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w

    angle_difference = 2 * math.acos(dot)
    return angle_difference