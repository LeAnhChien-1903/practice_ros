import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import numpy as np
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        # self.timer = self.create_timer(0.1, self.timer_callback)
        self.declare_parameter('laser_scan.queue_size', 0)
        self.declare_parameter('laser_scan.topic_name', '')
        laser_topic_name = self.get_parameter('laser_scan.topic_name').value
        laser_queue_size = self.get_parameter('laser_scan.queue_size').value
        self.subscription = self.create_subscription(
            LaserScan,
            laser_topic_name,
            self.listener_callback,
            laser_queue_size)
        self.subscription  # prevent unused variable warning
        self.point_cloud_subscription = self.create_subscription(
            PointCloud2,
            '/pioneer3at/lidar/point_cloud',
            self.point_cloud_callback,
            10)

    def listener_callback(self, msg):
        min_range = min(msg.ranges) # find min range 
        self.get_logger().info('Min range: %.2f' % min_range) # print min range to screen
    def point_cloud_callback(self, msg):
        point_cloud = np.array(msg.data)
        self.get_logger().info('Number of points: %d'% int(np.count_nonzero(point_cloud)/20))
    def timer_callback(self):
        pass
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()