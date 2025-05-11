import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtlePose:
    """Класс для обработки позиции черепахи на основе скоростей."""

    def __init__(self):
        """Инициализация подписки на скорости и публикации позиции."""
        self._velocity_sub = rospy.Subscriber('/turtle_velocity', Twist, self._velocity_callback)
        self._pose_pub = rospy.Publisher('/turtle_pose', Pose, queue_size=10)
        self._timer = rospy.Timer(rospy.Duration(0.1), self._timer_callback)

        # Начальные значения
        self._current_x = 5.0
        self._current_y = 5.0
        self._current_theta = 0.0
        self._dt = 0.1
        self._linear_velocity_x = 0.0
        self._angular_velocity_z = 0.0

    def _velocity_callback(self, data):
        """Обработка поступающих линейной и угловой скоростей."""
        self._linear_velocity_x = data.linear.x
        self._angular_velocity_z = data.angular.z

    def _timer_callback(self, event):
        """Обновление и публикация позиции черепахи."""
        pose = Pose()
        self._current_theta += self._angular_velocity_z * self._dt
        self._current_x += self._linear_velocity_x * math.cos(self._current_theta) * self._dt
        self._current_y += self._linear_velocity_x * math.sin(self._current_theta) * self._dt

        pose.x = self._current_x
        pose.y = self._current_y
        pose.theta = self._current_theta

        rospy.loginfo(f"Pose: x={pose.x:.2f}, y={pose.y:.2f}, theta={pose.theta:.2f}")
        self._pose_pub.publish(pose)

    def run(self):
        """Запуск узла и ожидание сообщений."""
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('turtle_pose_node')
    TurtlePose().run()
