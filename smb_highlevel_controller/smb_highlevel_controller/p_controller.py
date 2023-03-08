import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math 
class PController(Node):
    def __init__(self):
        super().__init__('p_controller')
        self.declare_parameter("controller.gain", 0)
        self.declare_parameter("controller.deltaT", 0.0)
        self.gain = self.get_parameter('controller.gain').value
        self.deltaT = self.get_parameter('controller.deltaT').value
        self.laser_scan = LaserScan
        self.twist_target = Twist
        self.setPoint = 0.1 # m
        self.vel_pub = self.create_publisher(Twist, '/pioneer3at/wheel/cmd_vel', 10)
        self.create_subscription(LaserScan, "/pioneer3at/sensor/laser_scan", self.laserScanCallback, 10)
        # self.timer_controller = self.create_timer(self.deltaT, self.timeCallback)
    def extractPosition(self):
        radius = 0.2
        ranges = np.array(self.laser_scan.ranges)
        angles = np.arange(self.laser_scan.angle_min, self.laser_scan.angle_max,
                           self.laser_scan.angle_increment)
        indexList = np.where(ranges < self.laser_scan.range_max)
        indexList = np.array(indexList[0])
        distanceList = np.zeros(indexList.shape[0])
        angleList = np.zeros(indexList.shape[0])
        for i in range(indexList.shape[0]):
            distanceList[i] = ranges[indexList[i]]
            angleList[i] = angles[indexList[i]]
        
        min_distance = np.min(distanceList)
        temp = np.where(distanceList == min_distance)
        angleOfMin = angleList[temp[0][0]]
        position = np.array([0.0, 0.0])
        position[0] = math.cos(angleOfMin) * (min_distance + radius)
        position[1] = math.sin(angleOfMin) * (min_distance + radius)
       
        return position
    def simplePController(self):
        ranges = np.array(self.laser_scan.ranges)
        indexList = np.where(ranges < self.laser_scan.range_max)
        indexList = np.array(indexList[0])
        distanceList = np.zeros(indexList.shape[0])
        for i in range(indexList.shape[0]):
            distanceList[i] = ranges[indexList[i]]
            
        error = self.setPoint - np.min(distanceList)
        controller_value = self.gain * error

        return controller_value
    def timeCallback(self):
        self.gain = self.get_parameter('controller.gain').value
        self.deltaT = self.get_parameter('controller.deltaT').value
        controller_value = self.simplePController()
        
        self.twist_target.linear.x = controller_value
        self.twist_target.linear.y = 0.0
        self.twist_target.linear.z = 0.0
        self.twist_target.angular.x = 0.0
        self.twist_target.angular.y = 0.0
        self.twist_target.angular.z = 0.0
        
        self.vel_pub.publish(self.twist_target)
    def laserScanCallback(self, msg):
        self.laser_scan = msg
        position = self.extractPosition()

        self.get_logger().info('Position: [%.2f, %.2f]'%(position[0], position[1]))

def main(args=None):
    rclpy.init(args=args)

    p_controller = PController()

    rclpy.spin(p_controller)
    p_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()