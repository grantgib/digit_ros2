import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   state = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
         get_package_share_directory('digit_json'),
        '/publish_state_real_launch.py'])
      )
   pointcloud = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
         get_package_share_directory('digit_json'),
         '/publish_pointcloud_launch.py']),
      )

   return LaunchDescription([
      state,
      pointcloud
   ])
