import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   tis_camera = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
         get_package_share_directory('digit_json'),
        '/publish_tis_camera_launch.py'])
      )
   forward_chest_rgb = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
         get_package_share_directory('digit_json'),
         '/publish_forward_chest_rgb_launch.py'])
      )
   forward_chest_left_infrared = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
         get_package_share_directory('digit_json'),
         '/publish_forward_chest_left_infrared_launch.py']),
      )
   forward_chest_right_infrared = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
         get_package_share_directory('digit_json'),
         '/publish_forward_chest_right_infrared_launch.py']),
      )
   forward_chest_depth_image = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
         get_package_share_directory('digit_json'),
         '/publish_forward_chest_depth_image_launch.py']),
      )
   forward_chest_depth_points = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
         get_package_share_directory('digit_json'),
         '/publish_forward_chest_depth_points_launch.py']),
      )
   pointcloud = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
         get_package_share_directory('digit_json'),
         '/publish_pointcloud_launch.py']),
      )

   return LaunchDescription([
      tis_camera,
      forward_chest_rgb,
      forward_chest_left_infrared,
      forward_chest_right_infrared,
      forward_chest_depth_image,
      forward_chest_depth_points,
      pointcloud
   ])
