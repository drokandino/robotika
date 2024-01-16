import rclpy
from rclpy.clock import Clock
import sys, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu

from turtlebot3_msgs.msg import Sound
import os
import turtlebot3_msgs

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
import numpy as np

class MoveRobot(Node):

    def __init__(self):
        super().__init__("MoveRobot")
        
        self.clock = Clock()

        # Laser scan qos profili
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE


        qos_profile2 = QoSProfile(depth=10)
        qos_profile2.reliability = QoSReliabilityPolicy.RELIABLE
        qos_profile2.durability = QoSDurabilityPolicy.VOLATILE

        self.scanSub = self.create_subscription(LaserScan, "/scan", self.scanCallback, qos_profile)
        self.speedPublisher = self.create_publisher(Twist, '/cmd_vel', 5)
        self.imuSub = self.create_subscription(Imu, "/imu", self.imuCallback, qos_profile2)
        #self.soundPub = self.create_publisher(Sound, "/sound", 5)

        self.get_logger().info("RUNNING")
        
        #self.get_logger().info(str(os.path.abspath(turtlebot3_msgs.__file__)))
        
        """
        sound = Sound()
        sound.value = 1
        self.soundPub.publish(sound)
        """
        self.navigatorPos = None
        self.obstacle = False
        self.obastacleTime = None
        self.boost = False
        self.boostTime = None

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def imuCallback(self, msg):
        speed = Twist()
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        roll, pitch, yaw = self.euler_from_quaternion(quaternion)


        if (roll > 0.05 and pitch < -0.06) or (roll < -0.05 and pitch < -0.06):
            self.get_logger().info("OBSTACLE ")
            self.get_logger().info("r p y: " + str((roll, pitch, yaw)))
            self.obstacle = True
            self.speedPublisher.publish(speed)    
            self.obastacleTime = self.clock.now().nanoseconds / 1e9

        """
        if msg.orientation.x > 0.03 or msg.orientation.x < -0.03 or msg.orientation.y > 0.03 or msg.orientation.y < -0.03 and self.obstacle == False:
            self.get_logger().info("OBSTACLE ")
            self.obstacle = True
            self.speedPublisher.publish(speed)    
            self.obastacleTime = self.clock.now().nanoseconds / 1e9

        """
        """
        elif msg.orientation.x < -0.03 and self.obstacle == False:
            self.get_logger().info("OBSTACLE")
            self.obstacle = True
            self.speedPublisher.publish(speed)
            self.obastacleTime = self.clock.now().nanoseconds / 1e9
        """
        if self.obstacle == True:
            if self.clock.now().nanoseconds / 1e9 - self.obastacleTime > 2:
                self.obstacle = False

    def scanCallback(self, msg):
        #angles = list()

        frontCnt = 0
        leftCnt = 0
        rightCnt = 0
        backCnt = 0
        backRightCnt = 0
        backLeftCnt = 0 
        speedScale = 1
       # self.get_logger().info("start angle " + str(msg.angle_min) )
        #self.get_logger().info("end angle " + str(msg.angle_max) )
        for i, distance in enumerate(msg.ranges):
            #self.get_logger().info("Distance" + str(distance) )
            #angles.append([i, distance])

            if i > 60 and i < 120:
                if distance < 0.15 and distance != 0.0:
                    leftCnt += 1

            if i > 150 and i < 210:
                if distance < 0.15 and distance != 0.0:
                    backCnt += 1

            if i > 240 and i < 300:
                if distance < 0.15 and distance != 0.0:
                    rightCnt += 1

            if i > 300 or i < 50:
                #self.get_logger().info("Uso")
                if distance < 0.15 and distance != 0.0:
                    frontCnt += 1

                if distance < 0.5 and distance > 0.4:
                    speedScale = 0.9
                elif distance < 0.4 and distance > 0.3:
                    speedScale = 0.8
                elif distance < 0.3 and distance > 0.1:
                    speedScale = 0.7
                elif distance < 0.1 and distance != 0.0:
                    self.get_logger().info(str(distance))
                    speedScale = 0.0
                    #self.movingAllowed = False
                else:
                    speedScale = 1

            if i > 210 and i < 240:
                if distance < 0.15 and distance != 0.0:
                    backRightCnt += 1
            
            if i > 120 and i < 150:
                if distance < 0.15 and distance != 0.0:
                    backLeftCnt += 1
            

                
       # self.get_logger().info(str(i))
        
        #self.get_logger().info("cnts: " + str(frontCnt) + " " + str(leftCnt) + " " + str(backCnt) + " " + str(rightCnt) + " " + str(backRightCnt) + " " + str(backLeftCnt))
        
        speed = Twist()
        """
        if max(frontCnt, leftCnt, backCnt, rightCnt) == frontCnt and frontCnt > 0:
            self.get_logger().info("Position: front")
            self.navigatorPos = "Front"
        """ 
        if max(frontCnt, leftCnt, backCnt, rightCnt) == leftCnt and leftCnt > 0:
            self.get_logger().info("Position: left")
            self.navigatorPos = "Left"
            speed.linear.x = 0.0
            speed.angular.z = -0.5 * speedScale

        elif max(frontCnt, leftCnt, backCnt, rightCnt) == backCnt and backCnt > 0:
            self.get_logger().info("Position: back")
            self.navigatorPos = "Back"

            """
            # check for boost
            if self.clock.now().nanoseconds - self.boostTime < 2:
                self.boost == True
                self.get_logger().info("Boost true")
            """

            speed.linear.x = 0.07 * speedScale
            self.get_logger().info("Speed linear " + str(speed.linear.x))

            if speed.linear.x > 0:
                if backLeftCnt > 5:
                    speed.angular.z = -0.2
                elif backRightCnt > 5:
                    speed.angular.z = 0.2
                else:
                    speed.angular.z = 0.0
                
        elif max(frontCnt, leftCnt, backCnt, rightCnt) == rightCnt and rightCnt > 0:
            self.get_logger().info("Position: right")
            self.navigatorPos = "Right"
            speed.linear.x = 0.0
            speed.angular.z = 0.5 * speedScale
            
        else:
            self.navigatorPos = None
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            self.get_logger().info("Position: None")
            #self.boostTime = self.clock.now().nanoseconds
            #self.boost = False

        if self.obstacle == False:    
            self.speedPublisher.publish(speed)
            

def main(args=None):
    rclpy.init(args=args)

    moveRobot = MoveRobot()

    rclpy.spin(moveRobot)

    #moveRobot.get_logger().info("RUNNING")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #scan_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    