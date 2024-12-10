import struct
from time import sleep
from serial import Serial

from geometry_msgs.msg import Twist

ser = Serial('/dev/ttyUSB0', 115200)

my_vel = Twist()
my_vel.linear.x = float(1)
my_vel.angular.z = float(0.5)

i = 0
d = 1
while 1:
    sleep(0.2)
    vel = struct.pack('ff', 1, 0)
    ser.write(vel)
    print("test is done")
    # if i <= 1:
    #     vel = struct.pack('ff', 1 - i, i)
    #     ser.write(vel)
    #     i = i + 0.1

    # if i > 1:
    #     a = i - 1
    #     vel = struct.pack('ff', i - 1, 2 - i)
    #     ser.write(vel)
    #     i = i + 0.1
    #     if i >= 2:
    #         i = 0
