from turtlesim.msg import Pose
import rospy
from math import *

class Path:
    def __init__(self) -> None:
        # Подписка на топик для получения пути
        self.pub = rospy.Publisher('my_path', Pose, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.counter = 0
        self.x = 5
        self.y = 5
        self.step = 1
    
    def timer_callback(self, data):
        # Обновление координат и публикация нового пути
        self.counter += self.step
        print('X: ', self.x, '\nY: ', self.y)
        self.talker()
    
    def run(self):
        rospy.spin()
    
    def talker(self):
        # Генерация нового пути
        pose = Pose()
        self.x = 5
        pose.y = 5
        self.y = pose.y
        self.pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('my_path')
    Path().run()
