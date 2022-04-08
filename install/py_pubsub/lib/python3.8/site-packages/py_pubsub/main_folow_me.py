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
from turtle import rt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time
import working_on_foxy

from std_msgs.msg import String

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, qos_profile_sensor_data

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

P_LIN= 0.08
P_ROT= 1.5

DIST_MIN = 0.4
DIST_MAX = 0.9 # Modif 0.9 > 1.0
DIST_MOY = (DIST_MAX + DIST_MIN) / 2

RTH = 0
COUNT = 0

class Node(Node):
    
    def __init__(self):

        super().__init__('minimal_subsciber')
        self.subscription = self.create_subscription(LaserScan,'scan',self.boucle,qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist,'cmd_vel', 10)
        self.subscription  # prevent unused variable warning
        #self.tempo_mode = time.time()

    
    def folow_me(self, msg):
        
        sum_x = 0
        sum_y = 0
        nb_valeur = 0

        for i in range(360):
            print(msg.ranges[i], msg.intensities[i])
            print(msg.ranges[300], msg.angle_max*180/math.pi)
            print(msg.ranges[i], msg.scan_time)
                

            if i>339 or i<20:
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

            # time.sleep(3) # Sleep for 3 seconds
        
            lin_vel = (moyenne_x + DIST_MOY) * P_LIN
            rot_vel = (moyenne_y) * P_ROT

        else:
            lin_vel = 0.0
            rot_vel = 0.0


        twist = Twist()

        twist.linear.x = lin_vel # -0.22 > +0.22
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = rot_vel # -2.84 > +2.84

        print("Vitesse lineaire : ",lin_vel)
        print("Vitesse rotationelle : ",rot_vel)
        self.pub.publish(twist)
        print("MODE FOLOW ME")
        return lin_vel
        

    def boucle(self, msg):

        vitesse = Node.folow_me(self, msg)
        print("Vitesse",vitesse)

        if vitesse == 0.0:
            time.sleep(3.0)
            vitesse = Node.folow_me(self, msg)
            if vitesse == 0.0:
                working_on_foxy.main()
                #print("MODE RETOUR BASE")


def main(args=None):

    rclpy.init(args=args)
    minimal_subscriber = Node()
    
    rclpy.spin(minimal_subscriber)

    #DESTRUCTION DU NODE
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
