import rosbag2_py
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from sensor_msgs.msg import PointCloud2
import numpy as np
from rosbag2_py import StorageOptions, ConverterOptions

class BagReader(Node):

    def __init__(self, bag_file):
        super().__init__('bag_reader')
        self.bag_file = bag_file

        # Define storage options (for reading a ROS 2 bag file)
        storage_options = StorageOptions(uri=self.bag_file, storage_id="sqlite3")
        # Define converter options (for reading msg types)
        converter_options = ConverterOptions("", "")

        # Create bag reader and open the file with the correct options
        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(storage_options, converter_options)

        # Initialize lists to hold timestamps
        self.lidar_times = []
        self.camera_times = []
        self.imu_times = []

        # Read all messages from the bag file
        self.read_messages()

    def read_messages(self):
        while self.reader.has_next():
            topic, msg, t = self.reader.read_next()
            # Extract timestamps
            if topic == '/rslidar_points':
                self.lidar_times.append(t)
            elif topic == '/camera/camera/color/image_raw':
                self.camera_times.append(t)
            elif topic == '/vectornav/imu':
                self.imu_times.append(t)
        print("lidar: ", self.lidar_times[:5])
        print("camera: ", self.camera_times[:5])
        print("imu: ", self.imu_times[:5])

    def find_nearest_time(self, target_time, times):
        """Find the time closest to the target_time in the times list."""
        times = np.array([t / 1e9 for t in times])
        target_time = target_time / 1e9
        # Calculate absolute differences
        time_diffs = np.abs(times - target_time)
        # Return the index of the closest time
        return np.argmin(time_diffs)

    def compute_time_diffs(self):
        # Convert raw timestamps (assumed as integers) to seconds
        lidar_times = np.array([t / 1e9 for t in self.lidar_times])
        camera_times = np.array([t / 1e9 for t in self.camera_times])
        imu_times = np.array([t / 1e9 for t in self.imu_times])

        # Initialize lists to hold the time differences
        lidar_camera_diff = []
        lidar_imu_diff = []
        camera_imu_diff = []

        # Calculate time differences for each lidar timestamp
        for lidar_time in self.lidar_times:
            # Find the closest camera and IMU timestamps
            camera_idx = self.find_nearest_time(lidar_time, self.camera_times)
            imu_idx = self.find_nearest_time(lidar_time, self.imu_times)

            # Check index bounds and calculate time differences
            if camera_idx < len(camera_times):
                camera_time = camera_times[camera_idx]
                lidar_camera_diff.append(np.abs(camera_time - lidar_time / 1e9))
            if imu_idx < len(imu_times):
                imu_time = imu_times[imu_idx]
                lidar_imu_diff.append(np.abs(imu_time - lidar_time / 1e9))

        # Calculate time differences for each camera timestamp
        for camera_time in self.camera_times:
            lidar_idx = self.find_nearest_time(camera_time, self.lidar_times)
            imu_idx = self.find_nearest_time(camera_time, self.imu_times)

            # Check index bounds and calculate time differences
            if lidar_idx < len(lidar_times):
                lidar_time = lidar_times[lidar_idx]
                lidar_camera_diff.append(np.abs(lidar_time - camera_time / 1e9))
            if imu_idx < len(imu_times):
                imu_time = imu_times[imu_idx]
                camera_imu_diff.append(np.abs(imu_time - camera_time / 1e9))

        # Calculate time differences for each IMU timestamp
        for imu_time in self.imu_times:
            lidar_idx = self.find_nearest_time(imu_time, self.lidar_times)
            camera_idx = self.find_nearest_time(imu_time, self.camera_times)

            # Check index bounds and calculate time differences
            if lidar_idx < len(lidar_times):
                lidar_time = lidar_times[lidar_idx]
                lidar_imu_diff.append(np.abs(lidar_time - imu_time / 1e9))
            if camera_idx < len(camera_times):
                camera_time = camera_times[camera_idx]
                camera_imu_diff.append(np.abs(camera_time - imu_time / 1e9))

        return lidar_camera_diff, lidar_imu_diff, camera_imu_diff

    def plot_time_diffs(self):
        lidar_camera_diff, lidar_imu_diff, camera_imu_diff = self.compute_time_diffs()

        # Create a plot
        plt.figure(figsize=(10, 6))
        plt.plot(lidar_camera_diff, label='Lidar - Camera Time Diff (s)', color='r')
        plt.plot(lidar_imu_diff, label='Lidar - IMU Time Diff (s)', color='g')
        plt.plot(camera_imu_diff, label='Camera - IMU Time Diff (s)', color='b')
        plt.xlabel('Message Index')
        plt.ylabel('Time Difference (seconds)')
        plt.title('Time Differences Between Sensor Messages')
        plt.legend()
        plt.grid(True)
        
        # Save the plot to a file instead of showing it
        plt.savefig('time_differences_plot.png')  # Change the path as needed
        print("Plot saved as 'time_differences_plot.png'")

        # Optionally, close the plot after saving to free resources
        plt.close()

def main():
    # Initialize ROS 2
    rclpy.init()

    # Specify the path to your recorded bag file
    bag_file = 'test_sync_0.db3'

    # Create the BagReader node and plot the time differences
    bag_reader = BagReader(bag_file)
    bag_reader.plot_time_diffs()

    # Clean up and shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()

