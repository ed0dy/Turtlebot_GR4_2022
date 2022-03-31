# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

from std_msgs.msg import String

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, qos_profile_sensor_data

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

P_LIN=0.2
P_ROT=3.0
DIST_MIN = 0.5
DIST_MAX = 1.0
DIST_MOY = (DIST_MAX + DIST_MIN) / 2

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subsciber')
        self.subscription = self.create_subscription(LaserScan,'scan',self.listener_callback,qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist,'cmd_vel', 10)
        self.subscription  # prevent unused variable warning

        
    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % qos_profile_sensor_data.data)
        
        sum_x = 0
        sum_y = 0
        nb_valeur =0

        for i in range(360):
            print(msg.ranges[i], msg.intensities[i])
            print(msg.ranges[300], msg.angle_max*180/math.pi)
            print(msg.ranges[i], msg.scan_time)
            

            if i>339 or i<10:
                if msg.ranges[i]>DIST_MIN and msg.ranges[i]<DIST_MAX:
                    nb_valeur += 1
                    x = msg.ranges[i] * math.cos(math.radians(i))
                    y = msg.ranges[i] * math.sin(math.radians(i))
                    sum_x = sum_x + x
                    sum_y = sum_y + y
                    print(msg.ranges[i])



        if nb_valeur !=0:
            moyenne_x = sum_x / nb_valeur
            moyenne_y = sum_y / nb_valeur
                
            lin_vel = (moyenne_x + DIST_MOY) * P_LIN
            rot_vel = (moyenne_y) * P_ROT
        else:
            lin_vel = 0.0
            rot_vel = 0.0

        print("Vitesse lineaire : ",lin_vel)
        print("Vitesse rotationelle : ",rot_vel)


        twist = Twist()

        twist.linear.x = lin_vel # -0.22 > +0.22
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = rot_vel # -2.84 > +2.84

        self.pub.publish(twist)
        print("Publier au robot !")
       
        #TODO

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_subsciber')
        #self.pub = self.create_publisher(Twist,'cmd_vel', 10)
        #self.subscription  # prevent unused variable warning





def main(args=None):
    rclpy.init(args=args)
    #qos = QoSProfile(depth=10)
    
    minimal_subscriber = MinimalSubscriber()
    minimal_publisher = MinimalPublisher()


    #TODO

    rclpy.spin(minimal_subscriber)
    rclpy.spin(minimal_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
