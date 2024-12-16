import rospy
from geometry_msgs.msg import Twist
from serial import Serial
import struct

class My_driver:
    def __init__(self) -> None:
        # Подписка на топик для получения команд скорости
        rospy.Subscriber('my_velocity', Twist, self.callback)
        # Открытие последовательного порта для связи с устройством
        self.ser = Serial('/dev/ttyUSB0', 115200)
    
    def callback(self, data):
        # Упаковка данных о скорости в бинарный формат и отправка по порту
        vel = struct.pack('ff',data.linear.x, data.angular.z)
        self.ser.write(vel)
        print('Velocity: ', vel)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('my_driver')
    My_driver().run()
