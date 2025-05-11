import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class VelocityController:
    """Класс для управления скоростью черепахи на основе пути и позиции."""

    LINEAR_GAIN = 1.0
    ANGULAR_GAIN = 0.7

    def __init__(self):
        """Инициализация подписок и публикаций."""
        self._path_sub = rospy.Subscriber('/turtle_path', Pose, self._path_callback)
        self._pose_sub = rospy.Subscriber('/turtle_pose', Pose, self._pose_callback)
        self._velocity_pub = rospy.Publisher('/turtle_velocity', Twist, queue_size=10)
        self._turtle_cmd_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self._timer = rospy.Timer(rospy.Duration(0.1), self._timer_callback)

        # Начальные значения
        self._target_x = 0.0
        self._target_y = 0.0
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_theta = 0.0

    def _path_callback(self, path):
        """Обработка целевой точки пути."""
        self._target_x = path.x
        self._target_y = path.y

    def _pose_callback(self, data):
        """Обработка текущей позиции черепахи."""
        self._current_x = data.x
        self._current_y = data.y
        self._current_theta = data.theta

    def _wrap_to_pi(self, angle):
        """Корректировка угла в диапазон [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _timer_callback(self, event):
        """Вычисление и публикация скоростей."""
        velocity = Twist()
        distance = math.sqrt((self._target_x - self._current_x) ** 2 + (self._target_y - self._current_y) ** 2)
        velocity.linear.x = self.LINEAR_GAIN * distance
        target_angle = math.atan2(self._target_y - self._current_y, self._target_x - self._current_x)
        velocity.angular.z = self.ANGULAR_GAIN * self._wrap_to_pi(target_angle - self._current_theta)

        rospy.loginfo(f"Velocity: linear_x={velocity.linear.x:.2f}, angular_z={velocity.angular.z:.2f}")
        self._velocity_pub.publish(velocity)
        self._turtle_cmd_pub.publish(velocity)

    def run(self):
        """Запуск узла и ожидание сообщений."""
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('velocity_controller_node')
    VelocityController().run()
