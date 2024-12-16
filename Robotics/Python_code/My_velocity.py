from math import *
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

Kv = 1
Kw = 0.7

class My_velocity:
    def __init__(self) -> None:
        # Подписка на два топика и публикация на два топика
        rospy.Subscriber('my_path', Pose, self.path_callback)
        rospy.Subscriber('my_turtle_pose', Pose, self.callback)
        self.pub = rospy.Publisher('my_velocity', Twist, queue_size = 10)
        self.pub_turtle = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        
        # Начальные значения для координат и угла
        self.x_final = 0
        self.y_final = 0
        self.x_pose = 0
        self.y_pose = 0
        self.theta = 0

    def path_callback(self, path):
        # Обработчик для получения целевой точки пути
        self.x_final = path.x
        self.y_final = path.y

    def callback(self, data):
        # Обработчик для получения текущей позиции черепахи
        self.x_pose = data.x
        self.y_pose = data.y
        self.theta = data.theta

    def wrap_pi(self, angle):
        # Функция для корректировки угла в пределах [-pi, pi]
        while angle > pi:
            angle -= 2*pi
        while angle < -pi:
            angle += 2*pi
        return angle

    def timer_callback(self, event):
        # Основной цикл, публикующий команду скорости для черепахи
        my_vel = Twist()
        # Линейная скорость рассчитывается как расстояние до цели
        my_vel.linear.x = Kv * sqrt((self.x_final-self.x_pose)**2 + (self.y_final-self.y_pose)**2)
        # Угловая скорость рассчитывается как разница углов
        my_vel.angular.z = Kw * self.wrap_pi(atan2((self.y_final-self.y_pose),(self.x_final-self.x_pose)) - self.theta)

        print("V = ", my_vel.linear.x, "\nW = ", my_vel.angular.z)

        self.pub.publish(my_vel)
        self.pub_turtle.publish(my_vel)
    
    def run(self):
        rospy.spin()
    
if __name__=='__main__':
    rospy.init_node('my_velocity')
    My_velocity().run()
