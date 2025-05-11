import rospy
from geometry_msgs.msg import Twist
from serial import Serial
import struct

class VelocityDriver:
    """Класс для отправки скоростей черепахи через последовательный порт."""

    def __init__(self):
        """Инициализация подписки и последовательного порта."""
        self._velocity_sub = rospy.Subscriber('/turtle_velocity', Twist, self._velocity_callback)
        self._serial = Serial('/dev/ttyUSB0', 115200)

    def _velocity_callback(self, data):
        """Упаковка и отправка скоростей через последовательный порт."""
        velocity_data = struct.pack('ff', data.linear.x, data.angular.z)
        self._serial.write(velocity_data)
        rospy.loginfo(f"Velocity sent: linear_x={data.linear.x:.2f}, angular_z={data.angular.z:.2f}")

    def run(self):
        """Запуск узла и ожидание сообщений."""
        try:
            rospy.spin()
        finally:
            self._serial.close()


if __name__ == '__main__':
    rospy.init_node('velocity_driver_node')
    VelocityDriver().run()
