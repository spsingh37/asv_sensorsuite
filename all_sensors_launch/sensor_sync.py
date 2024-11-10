import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from sensor_msgs.msg import PointCloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer

class SensorSync(Node):

    def __init__(self):
        super().__init__('sensor_sync')

        # Create subscribers
        self.lidar_sub = Subscriber(self, PointCloud2, '/rslidar_points')
        self.camera_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.imu_sub = Subscriber(self, Imu, '/vectornav/imu')

        # Approximate synchronizer with a tolerance of 0.1 seconds
        self.ts = ApproximateTimeSynchronizer([self.lidar_sub, self.camera_sub, self.imu_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)

    def callback(self, lidar_msg, camera_msg, imu_msg):
        # Process synchronized messages here
        lidar_time = lidar_msg.header.stamp
        camera_time = camera_msg.header.stamp
        imu_time = imu_msg.header.stamp
        self.get_logger().info(f"Received synchronized messages:\nLidar: {lidar_time}\nCamera: {camera_time}\nIMU: {imu_time}")

def main(args=None):
    rclpy.init(args=args)
    sensor_sync = SensorSync()
    rclpy.spin(sensor_sync)
    sensor_sync.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

