import rospy
from std_msgs.msg import *
from ttastromech import TTAstromech


def speak_callback(string_msg, r2):
	print string_msg.data
	r2.speak(string_msg.data)


def main():
	r2 = TTAstromech()
	rospy.init_node('ros2d2_node')
	rospy.Subscriber('~speak', String, speak_callback, r2)
	rospy.spin()


if __name__ == '__main__':
	main()
