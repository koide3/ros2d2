#!/usr/bin/python
import numpy
import rospy
import rospkg
from std_msgs.msg import *
from ttastromech import TTAstromech


class ROS2D2Node:
	def __init__(self):
		self.r2 = TTAstromech()
		rospy.init_node('ros2d2_node')
		self.load_presets()

		self.preset_sub = rospy.Subscriber('~preset', String, self.preset_callback)
		self.speak_sub = rospy.Subscriber('~speak', String, self.speak_callback)
		self.cmd_pub = rospy.Publisher('~synth_cmd', String, queue_size=5)

	def load_presets(self):
		package_path = rospkg.RosPack().get_path('ros2d2')

		self.presets = {}
		data = numpy.loadtxt(package_path + '/data/presets', dtype=object, delimiter=':')
		for line in data:
			self.presets[line[0]] = line[1]

	def preset_callback(self, preset_msg):
		if preset_msg.data not in self.presets:
			print preset_msg.data, 'not in presets'
			return

		cmd_msg = String(data=self.presets[preset_msg.data])
		self.cmd_pub.publish(cmd_msg)

	def speak_callback(self, string_msg):
		self.r2.speak(string_msg.data)


def main():
	node = ROS2D2Node()
	rospy.spin()


if __name__ == '__main__':
	main()
