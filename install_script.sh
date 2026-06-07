rosdep update && rosdep install --from-paths src/isaac_ros_h264_encoder --ignore-src -y
colcon build --symlink-install --packages-up-to isaac_ros_h264_encoder --base-paths src