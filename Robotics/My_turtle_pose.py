from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import *

import rospy

class My_turtle_pose:
    def __init__(self) -> None:
        rospy.Subscriber('my_velocity', Twist, self.callback)
        self.pub = rospy.Publisher('my_turtle_pose', Pose, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        self.cur_x = 5
        self.cur_y = 5
        self.cur_th = 0
        self.dt = 0.1

        self.vel_lin_x = 0
        self.vel_ang_z = 0
    
    def callback(self, data):
        self.vel_lin_x = data.linear.x
        self.vel_ang_z = data.angular.z
        # rospy.loginfo('Turtle_pose vel: %s, %s, %s', data.x, data.y, data.th)
    
    def timer_callback(self, event):
        my_pose = Pose()
        self.cur_th += self.vel_ang_z * self.dt
        self.cur_x += self.vel_lin_x * cos(self.cur_th) * self.dt
        self.cur_y += self.vel_lin_x * sin(self.cur_th) * self.dt

        my_pose.x = self.cur_x
        my_pose.y = self.cur_y
        my_pose.theta = self.cur_th

        print("My pose: ", my_pose.x, '; ', my_pose.y, '; ', my_pose.theta)
        # print("My vel: ", self.vel_lin_x, '; ', self.vel_ang_z)

        self.pub.publish(my_pose)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('my_turtle_pose')
    My_turtle_pose().run()
