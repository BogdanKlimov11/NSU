import rospy
from turtlesim.msg import Pose
from math import *

step = 1

class Path:
    def __init__(self) -> None:
        # rospy.Subscriber('my_velocity', Twist, self.callback)
        self.pub = rospy.Publisher('my_path', Pose, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.counter = 0
        self.x = 5
        self.y = 5
        self.step = 1
    
    def timer_callback(self, data):
        self.counter+=self.step
        print('X: ', self.x, '\nY: ', self.y)
        self.talker()
    
    def run(self):
        rospy.spin()
    
    def talker(self):
        pose = Pose()
        # pose.x = 5 + 4 * cos(self.counter / 20)
        
        # pose.x = 5 + self.counter / 30
        # if (self.counter == 50):
        #     self.step = -1
        # if (self.counter == -50):
        #     self.step = 1
        self.x = 5

        # pose.y = 2 + 1 * sin(self.counter / 4)
        pose.y = 5
        self.y = pose.y
        self.pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('my_path')
    Path().run()
