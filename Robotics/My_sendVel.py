import rospy
from geometry_msgs.msg import Twist
from serial import Serial
import struct

class My_driver:
    def __init__(self) -> None:
        rospy.Subscriber('my_velocity', Twist, self.callback)
        self.ser = Serial('/dev/ttyUSB0', 115200)
    

    def callback(self, data):
        vel = struct.pack('ff',data.linear.x, data.angular.z)
        self.ser.write(vel)
        print('Velocity: ', vel)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('my_driver')
    My_driver().run()