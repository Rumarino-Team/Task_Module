
import yaml
import rospy
from std_msgs.msg import Float32
# from uuv_sensor_ros_plugins_msgs.msg import DVL_Message
from nav_sensors.msg import DVL_MSG
from zed_interfaces.msg import ObjectsStamped
from zed_interfaces.msg import RGBDSensors


def read_yaml_file(file_path):
    with open(file_path, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return None



# Class for accesing the Robots Data in the state machine
class SharedData:
    def __init__(self):
        #TODO: Add the 
        self.zed_data = {"ObjectsStamped": None, "RGBDSensors": None, "Pose": None}
        self.dvl_data = None
        self.imu_data = {'yaw': None, 'pitch': None, 'roll': None}
        # Add more attributes as needed

        # Match simulation Pose
        self.pose = None



# Global variable for accesing the shared_data 
shared_data = SharedData()

def zed_objects_callback(msg):
    shared_data.zed_data['ObjectsStamped'] = msg

def zed_rgbd_callback(msg):
    shared_data.zed_data["RGBDSensors"] = msg

def dvl_callback(msg):
    shared_data.dvl_data = msg

def imu_yaw_callback(msg):
    shared_data.imu_data['yaw'] = msg.data

def imu_pitch_callback(msg):
    shared_data.imu_data['pitch'] = msg.data

def imu_roll_callback(msg):
    shared_data.imu_data['roll'] = msg.data

def pose_callback(msg):
    shared_data.pose = msg



def initialize_subscribers(topics: str):
    topics_info = read_yaml_file(topics)
    if topics_info is None:
        print("Failed to read YAML file or file is empty.")
    else:
        print("YAML file read successfully:", topics_info)
        print(topics_info['zed_camera']['ObjectsStamped'])

    rospy.Subscriber(topics_info['zed_camera']['ObjectsStamped'],ObjectsStamped, zed_objects_callback)  # Update the message type
    rospy.Subscriber(topics_info['zed_camera']['RGBDSensors'],RGBDSensors,  zed_rgbd_callback)  # Update the message type
    rospy.Subscriber(topics_info['dvl']['DVL_Message'], DVL_MSG, dvl_callback)  # Update the message type
    rospy.Subscriber(topics_info['imu']['roll'],Float32, imu_roll_callback)
    rospy.Subscriber(topics_info['imu']['yaw'],Float32,  imu_yaw_callback)
    rospy.Subscriber(topics_info['imu']['pitch'],Float32 , imu_pitch_callback)