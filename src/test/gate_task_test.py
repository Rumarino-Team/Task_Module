import unittest
import rospy as rp
from zed_interfaces.msg import Objects
import smach
from movement import LowerDepth, Rotate90DegreesState, UpdatePoseToObjectState
from gate_task import CheckImageVisibleState, main

class TestMultidof6StateMachine(unittest.TestCase):
    def setUp(self):
        rp.init_node('test_multidof6_state_machine', anonymous=True)
        self.sm = main()  # Assuming main() returns the state machine instance
        self.objects_subscriber = rp.Subscriber('/your_object_topic', Objects, self.objects_callback)
        self.received_objects = []

    def objects_callback(self, msg):
        self.received_objects.append(msg)

    def test_check_image_visible_state(self):
        # Test if the state machine transitions to the correct state based on the presence of the desired object in the received objects
        self.sm.set_initial_state(['CHECK_IMAGE_VISIBLE'])
        outcome = self.sm.execute(self.received_objects)
        expected_outcome = 'detected'  # Update this based on your desired outcome
        self.assertEqual(outcome, expected_outcome)

    def test_lower_depth_state(self):
        # Test the LowerDepth state
        self.sm.set_initial_state(['Lower_Depth'])
        outcome = self.sm.execute(self.received_objects)
        expected_outcome = 'success'  # Update this based on your desired outcome
        self.assertEqual(outcome, expected_outcome)

    def test_rotate_90_degrees_state(self):
        # Test the Rotate90DegreesState
        self.sm.set_initial_state(['Rotate90DegreesState'])
        outcome = self.sm.execute(self.received_objects)
        expected_outcome = 'success'  # Update this based on your desired outcome
        self.assertEqual(outcome, expected_outcome)

    def test_survey_gate_state(self):
        # Test the Survey_Gate state
        self.sm.set_initial_state(['SURVEY_GATE'])
        outcome = self.sm.execute(self.received_objects)
        expected_outcome = 'completed'  # Update this based on your desired outcome
        self.assertEqual(outcome, expected_outcome)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('your_package', 'test_multidof6_state_machine', TestMultidof6StateMachine)