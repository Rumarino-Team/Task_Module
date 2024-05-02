import rospy
from std_msgs.msg import String
from flask import Flask

app = Flask(__name__)

@app.route('/')
def hello():
	return 'Hello, ROS!'

def callback(data):
	rospy.loginfo("Received data: %s", data.data)

def listener():
	rospy.init_node('flask_webpage_node', anonymous=True)
	rospy.Subscriber('data_topic', String, callback)
	rospy.spin()

if __name__ == '__main__':
	listener_thread = threading.Thread(target=listener)
	listener_thread.start()
	app.run()