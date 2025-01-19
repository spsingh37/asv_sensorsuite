# asv_sensorsuite
1. Create 'lidar_ws/src' and place the rslidar_sdk and rslidar_msg inside it (instructions given in rslidar_sdk repo)
2. For Vectornav IMU, follow their instructions. After building, when trying to run, u may face error like "can't connect/reconnect /dev/ttyUSB0" then do "sudo chmod -R 777 /dev/ttyUSB0"
3. For increasing the update rate, change the config to "vn_100_800hz.yaml" in launch/vectornav.launch.py
4. For installing realsense camera drivers, follow th eguide here "https://github.com/IntelRealSense/realsense-ros":
5. For Step 2, Option 1, Jetson, build from source i.e. this way "https://github.com/IntelRealSense/librealsense/blob/master/doc/libuvc_installation.md"
6. For Step 3, install from source
