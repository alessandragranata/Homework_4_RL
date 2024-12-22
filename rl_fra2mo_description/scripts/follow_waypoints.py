#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler
from ament_index_python.packages import get_package_share_directory
import os
import yaml

goals_yaml_path=os.path.join(get_package_share_directory('rl_fra2mo_description'), "config", "goals.yaml")
with open(goals_yaml_path, 'r') as yaml_file:
        yaml_content = yaml.safe_load(yaml_file)



def main():
    rclpy.init()
    navigator = BasicNavigator()

    def create_pose(transform):
        name= transform["name"]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = transform["position"]["x"]   
        pose.pose.position.y = transform["position"]["y"]
        pose.pose.position.z = transform["position"]["z"]
        pose.pose.orientation.x = transform["orientation"]["x"] 
        pose.pose.orientation.y = transform["orientation"]["y"]
        pose.pose.orientation.z = transform["orientation"]["z"]
        pose.pose.orientation.w = transform["orientation"]["w"]
        return pose, name  # Restituisce una tupla (pose, name)

    goals = list(map(create_pose, yaml_content["waypoints"]))

    # Select goals in the desired order
    target_order = ["Goal_3", "Goal_4", "Goal_2", "Goal_1"]
    goal_poses = [pose for name in target_order for pose, n in goals if n == name]

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer="smoother_server")

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()