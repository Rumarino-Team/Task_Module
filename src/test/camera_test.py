#!/usr/bin/env python
import sys
sys.path.append("/root/catkin_ws/src/Task_Module")
import unittest
import rospy
import os
import rosbag
import yaml
import rospy
from tasks.src.data import shared_data, initialize_subscribers
class Camera_Behaviours(unittest.TestCase):
    def setUp(self):
        print("Setting Up Camera Test")

        rospy.init_node('test_camera_node', anonymous=True)
        file_path = os.path.join(os.path.dirname(__file__), 'test_data/test_topics.yml')
        initialize_subscribers(file_path)
        rospy.sleep(5)
        # sm = YourStateMachine()
        # outcome = sm.execute()

    def test_camera_input(self):
        # wait 5 seconds for data to be published
        print("Testing Camera Input")
        self.assertIsNotNone(shared_data.zed_data['objects_stamped'])
        self.assertIsNotNone(shared_data.zed_data['imu'])
        self.assertIsNotNone(shared_data.zed_data['image'])
        self.assertIsNotNone(shared_data.zed_data['camera_info'])
        self.assertIsNotNone(shared_data.zed_data['pose'])
        self.assertIsNotNone(shared_data.zed_data['odom'])
        self.assertIsNotNone(shared_data.zed_data['path_odom'])
        self.assertIsNotNone(shared_data.zed_data['path_map'])

    def tearDown(self):
        # Clean up code here
        pass

if __name__ == '__main__':
    import rostest
    rostest.rosrun('tasks', 'test_camera_behaviours', Camera_Behaviours)
