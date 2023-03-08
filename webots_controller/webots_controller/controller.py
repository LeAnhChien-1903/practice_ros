# ROS libraries
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
# Python libraries
import math
import numpy as np

class WebotsController():
    def init(self, webots_ros, properties):
        self.__robot = webots_ros.robot
        self.__timeStep = 16
        rclpy.init(args=None)
        self.__node = rclpy.create_node('webots_controller')
        # Robot constants
        self.wheel_base = 0.394 # m
        self.L = 0.31  # m
        self.wheel_radius = 0.11 # m
        self.max_speed = 6.4 # rad/s
        self.deltaT = 0.016
        # Variables
        self.__pose = [0.0, 0.0, 0.0] # Pose of robot x, y, theta
        self.__motor1 = self.__robot.getDevice('front_left_motor') #  front left wheel
        self.__motor2 = self.__robot.getDevice('back_left_motor') # back left wheel
        self.__motor3 = self.__robot.getDevice('front_right_motor') # front right wheel
        self.__motor4 = self.__robot.getDevice('back_right_motor') # back right wheel
        self.__prevMotor1Position = 0.0 # position of front left wheel (rad)
        self.__prevMotor2Position = 0.0 # position of back left wheel (rad)
        self.__prevMotor3Position = 0.0 # position of front right wheel (rad)
        self.__prevMotor4Position = 0.0 # position of back right wheel (rad)
        self.__lidar = self.__robot.getDevice('lidar') # front lidar
        self.__imu = self.__robot.getDevice('imu') # inertial sensor for roll, pitch, yaw 
        self.__imuData = Imu() 
        self.__laserScan = LaserScan() 
        self.__odom = Odometry() 
        self.__targetTwist = Twist()
        # Initialize sensors and motors
        self.motorInitialization()
        self.sensorsInitialization()
        # Create a node to subscribe and publish data
        self.__imuPub = self.__node.create_publisher(Imu, '/pioneer3at/sensor/imu', 10)
        self.__laserScanPub = self.__node.create_publisher(LaserScan, '/pioneer3at/sensor/laser_scan', 10)
        self.__laserScanPub_new = self.__node.create_publisher(LaserScan, '/scan', 10)
        self.__odomPub = self.__node.create_publisher(Odometry, '/pioneer3at/wheel/odometry', 10)
        self.__node.create_subscription(Twist, '/pioneer3at/wheel/cmd_vel', self.velocityCallback, 10)
    def motorInitialization(self):
        # Motor initialization
        self.__motor1.setPosition(float('inf'))
        self.__motor2.setPosition(float('inf'))
        self.__motor3.setPosition(float('inf'))
        self.__motor4.setPosition(float('inf'))

        self.__motor1.setVelocity(0.0)
        self.__motor2.setVelocity(0.0)
        self.__motor3.setVelocity(0.0)
        self.__motor4.setVelocity(0.0)
        # Position sensors initialization
        self.__motor1Sensor = self.__robot.getDevice('front_left_sensor')
        self.__motor2Sensor = self.__robot.getDevice('back_left_sensor')
        self.__motor3Sensor = self.__robot.getDevice('front_right_sensor')
        self.__motor4Sensor = self.__robot.getDevice('back_right_sensor')

        self.__motor1Sensor.enable(self.__timeStep)
        self.__motor2Sensor.enable(self.__timeStep)
        self.__motor3Sensor.enable(self.__timeStep)
        self.__motor4Sensor.enable(self.__timeStep)

    def sensorsInitialization(self):
        # Lidar initialization
        self.__lidar.enable(self.__timeStep)
        self.__lidar. enablePointCloud()
        # IMU initialization
        self.__imu.enable(self.__timeStep)
    def velocityCallback(self, twist):
        self.__targetTwist = twist
    def getImuData(self):
        imuDataQuaternion = self.__imu.getQuaternion()
        self.__imuData.header.stamp = self.__node.get_clock().now().to_msg()
        self.__imuData.header.frame_id = 'base_link'
        self.__imuData.orientation.x = imuDataQuaternion[0]
        self.__imuData.orientation.y = imuDataQuaternion[1]
        self.__imuData.orientation.z = imuDataQuaternion[2]
        self.__imuData.orientation.w = imuDataQuaternion[3]
        self.__imuPub.publish(self.__imuData)
    def getLidarLaserScan(self):
        self.__laserScan.header.stamp = self.__node.get_clock().now().to_msg()
        self.__laserScan.header.frame_id = 'lidar'
        self.__laserScan.angle_min = np.pi
        self.__laserScan.angle_max = -np.pi
        self.__laserScan.angle_increment = -2*np.pi/359
        self.__laserScan.time_increment = 0.0
        self.__laserScan.scan_time = 0.0
        self.__laserScan.range_min = 0.12
        self.__laserScan.range_max = 3.5
        self.__laserScan.ranges = self.__lidar.getRangeImage()
        
        self.__laserScanPub.publish(self.__laserScan)
        self.__laserScanPub_new.publish(self.__laserScan)
    def odometry(self):
        currMotor1Position = self.__motor1Sensor.getValue()
        currMotor2Position = self.__motor2Sensor.getValue()
        currMotor3Position = self.__motor3Sensor.getValue()
        currMotor4Position = self.__motor4Sensor.getValue()
        # Compute velocity of motors (rad/s)
        motor1_est_vel = (currMotor1Position - self.__prevMotor1Position) / self.deltaT
        motor2_est_vel = (currMotor2Position - self.__prevMotor2Position) / self.deltaT
        motor3_est_vel = (currMotor3Position - self.__prevMotor3Position) / self.deltaT
        motor4_est_vel = (currMotor4Position - self.__prevMotor4Position) / self.deltaT
        # Compute deltaX, deltaY, deltaTheta
        omega_left = (motor1_est_vel + motor2_est_vel) / 2
        omega_right = (motor3_est_vel + motor4_est_vel) / 2
        # Compute linear and angular velocity of robot (m/s and rad/s)
        linear_vel = (omega_left + omega_right) / 2
        angular_vel = (omega_right - omega_left)/ self.wheel_base
        
        # Compute pose of robot (x y theta) - (m, m, m)
        self.__pose[2] += angular_vel*self.deltaT # compute theta
        vx = linear_vel * math.cos(self.__pose[2]) 
        vy = linear_vel * math.sin(self.__pose[2])
        self.__pose[0] += vx*self.deltaT
        self.__pose[1] += vy*self.deltaT

        # Set header and child
        self.__odom.header.stamp = self.__node.get_clock().now().to_msg()
        self.__odom.header.frame_id = "odom"
        self.__odom.child_frame_id = "base_link"

        quaternion = self.quaternion_from_euler(0.0, 0.0, self.__pose[2])
        # Set pose
        self.__odom.pose.pose.position.x = self.__pose[0]
        self.__odom.pose.pose.position.y = self.__pose[1]
        self.__odom.pose.pose.position.z = 0.0
        self.__odom.pose.pose.orientation.x = quaternion[0]
        self.__odom.pose.pose.orientation.y = quaternion[1]
        self.__odom.pose.pose.orientation.z = quaternion[2]
        self.__odom.pose.pose.orientation.w = quaternion[3]
        # Set velocity
        self.__odom.twist.twist.linear.x = vx
        self.__odom.twist.twist.linear.y = vy
        self.__odom.twist.twist.linear.z = 0.0
        self.__odom.twist.twist.angular.x = 0.0
        self.__odom.twist.twist.angular.y = 0.0
        self.__odom.twist.twist.angular.z = angular_vel
        # Publish the odometry
        self.__odomPub.publish(self.__odom)

        self.__prevMotor1Position = currMotor1Position
        self.__prevMotor2Position = currMotor2Position
        self.__prevMotor3Position = currMotor3Position
        self.__prevMotor4Position = currMotor4Position

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        linear_vel = self.__targetTwist.linear.x
        angular_vel = self.__targetTwist.angular.z

        leftVelocity = (linear_vel - 0.5 * angular_vel *  self.wheel_base)/self.wheel_radius
        rightVelocity = (linear_vel + 0.5 * angular_vel * self.wheel_base)/self.wheel_radius
        if leftVelocity > self.max_speed:
            leftVelocity = self.max_speed
        if leftVelocity < -self.max_speed:
            leftVelocity = -self.max_speed
        if rightVelocity > self.max_speed:
            rightVelocity = self.max_speed
        if rightVelocity < -self.max_speed:
            leftVelocity = - self.max_speed
        self.__motor1.setVelocity(leftVelocity)
        self.__motor2.setVelocity(leftVelocity)
        self.__motor3.setVelocity(rightVelocity)
        self.__motor4.setVelocity(rightVelocity)
        self.getImuData()
        self.getLidarLaserScan()
        self.odometry()
    @staticmethod
    def quaternion_from_euler(ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

def main(args=None):
    rclpy.init(args=None)
    pioneer3at = WebotsController(args=args)
    rclpy.spin(pioneer3at)
    pioneer3at.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main() 