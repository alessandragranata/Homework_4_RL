
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
   
    navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('rl_fra2mo_description'),
                                    'launch',
                                    'fra2mo_explore.launch.py'])]
            )
        )
    
    aruco = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [PathJoinSubstitution([FindPackageShare('aruco_ros'),
                                    'launch',
                                    'single.launch.py'])]
            )
        )

    return LaunchDescription([navigation, aruco])