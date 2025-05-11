import rospy
from turtlesim.msg import Pose
import math

class PathGenerator:
    """Класс для генерации и публикации пути черепахи."""

    def __init__(self):
        """Инициализация публикации пути и таймера."""
        self._path_pub = rospy.Publisher('/turtle_path', Pose, queue_size=10)
        self._timer = rospy.Timer(rospy.Duration(0.1), self._timer_callback)
        self._counter = 0
        self._current_x = 5.0
        self._current_y = 5.0
        self._step = 1

    def _timer_callback(self, event):
        """Обновление координат и публикация пути."""
        self._counter += self._step
        rospy.loginfo(f"Path: x={self._current_x:.2f}, y={self._current_y:.2f}")
        self._publish_path()

    def _publish_path(self):
        """Генерация и публикация новой точки пути."""
        pose = Pose()
        pose.x = self._current_x
        pose.y = self._current_y
        self._path_pub.publish(pose)

    def run(self):
        """Запуск узла и ожидание сообщений."""
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('path_generator_node')
    PathGenerator().run()
