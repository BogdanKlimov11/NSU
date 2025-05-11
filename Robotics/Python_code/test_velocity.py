import struct
import time
from serial import Serial
import rospy
from geometry_msgs.msg import Twist

class TestVelocity:
    """Класс для тестирования отправки скоростей через последовательный порт."""

    def __init__(self):
        """Инициализация последовательного порта."""
        self._serial = Serial('/dev/ttyUSB0', 115200)
        self._velocity = Twist()
        self._velocity.linear.x = 1.0
        self._velocity.angular.z = 0.5

    def run(self):
        """Отправка тестовых скоростей с интервалом 0.2 секунды."""
        try:
            while not rospy.is_shutdown():
                velocity_data = struct.pack('ff', self._velocity.linear.x, self._velocity.angular.z)
                self._serial.write(velocity_data)
                rospy.loginfo("Test velocity sent")
                time.sleep(0.2)
        except KeyboardInterrupt:
            rospy.loginfo("Test velocity stopped")
        finally:
            self._serial.close()


if __name__ == '__main__':
    rospy.init_node('test_velocity_node')
    TestVelocity().run()
