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

from geometry_msgs.msg import PoseStamped, TransformStamped, Transform
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler, quaternion_from_matrix
from tf2_ros import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import tf_transformations


goals_yaml_path=os.path.join(get_package_share_directory('rl_fra2mo_description'), "config", "nav_aruco.yaml")
with open(goals_yaml_path, 'r') as yaml_file:
        yaml_content = yaml.safe_load(yaml_file)

class ArucoNavigation(Node):
    def __init__(self):
        super().__init__('aruco_navigation_node')
        self.aruco_subscription = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',
            self.aruco_callback,
            10
        )
        self.get_logger().info("Subscribed to /aruco_single/pose")
        self.tf_broadcaster = StaticTransformBroadcaster(self)  # TF broadcaster

    def aruco_callback(self, msg):
        
        # Trasformazione Posa finale rispetto gazebo
        transform_to_map = Transform()
        transform_to_map.translation.x = -3.34
        transform_to_map.translation.y = 0.0
        transform_to_map.translation.z = 0.0
        quaternion_to_map = quaternion_from_euler(0.0, 0.0, -1.57)
        transform_to_map.rotation.x = quaternion_to_map[0]
        transform_to_map.rotation.y = quaternion_to_map[1]
        transform_to_map.rotation.z = quaternion_to_map[2]
        transform_to_map.rotation.w = quaternion_to_map[3]


        transform1_matrix = tf_transformations.translation_matrix([transform_to_map.translation.x,
                                                   transform_to_map.translation.y,
                                                   transform_to_map.translation.z])
        transform1_matrix = tf_transformations.concatenate_matrices(
            transform1_matrix,
            tf_transformations.quaternion_matrix([
            transform_to_map.rotation.x,
            transform_to_map.rotation.y,
            transform_to_map.rotation.z,
            transform_to_map.rotation.w,
        ])
        )
        
        #Trasformazione base_footprint - camera_link_optical
        transform_to_base = Transform()
        transform_to_base.translation.x = 0.108
        transform_to_base.translation.y = 0.0
        transform_to_base.translation.z = 0.065
     
    
     
        transform_to_base.rotation.x = -0.5
        transform_to_base.rotation.y = 0.5
        transform_to_base.rotation.z = -0.5
        transform_to_base.rotation.w = 0.5


        transform2_matrix = tf_transformations.translation_matrix([transform_to_base.translation.x,
                                                   transform_to_base.translation.y,
                                                   transform_to_base.translation.z])
        transform2_matrix = tf_transformations.concatenate_matrices(
            transform2_matrix,
            tf_transformations.quaternion_matrix([
            transform_to_base.rotation.x,
            transform_to_base.rotation.y,
            transform_to_base.rotation.z,
            transform_to_base.rotation.w,
        ])
        )
        
        #Trasformazione aruco_marker_frame - camera_link_optical
        transform_to_optical = Transform()
        transform_to_optical.translation.x = msg.pose.position.x
        transform_to_optical.translation.y = msg.pose.position.y
        transform_to_optical.translation.z = msg.pose.position.z
     
    
     
        transform_to_optical.rotation.x = msg.pose.orientation.x
        transform_to_optical.rotation.y = msg.pose.orientation.y
        transform_to_optical.rotation.z = msg.pose.orientation.z
        transform_to_optical.rotation.w = msg.pose.orientation.w


        transform3_matrix = tf_transformations.translation_matrix([transform_to_optical.translation.x,
                                                   transform_to_optical.translation.y,
                                                   transform_to_optical.translation.z])
        transform3_matrix = tf_transformations.concatenate_matrices(
            transform3_matrix,
            tf_transformations.quaternion_matrix([
            transform_to_optical.rotation.x,
            transform_to_optical.rotation.y,
            transform_to_optical.rotation.z,
            transform_to_optical.rotation.w,
        ])
        )


        #Applicazione trasformazioni 
        aruco_pose_matrix=Transform()
        aruco_pose_matrix = tf_transformations.concatenate_matrices(transform2_matrix, transform3_matrix)
        aruco_pose_matrix = tf_transformations.concatenate_matrices(transform1_matrix,aruco_pose_matrix)

        
        # self.get_logger().info(f"Detected ArUco marker position at: [x: {translation.x:.2f}, y: {aruco_pose_matrix.translation.y:.2f}, z: {aruco_pose_matrix.translation.z:.2f}]"
        #                        f"Detected ArUco marker orientation at: [x: {aruco_pose_matrix.rotation.x:.2f}, y: {aruco_pose_matrix.rotation.y:.2f}, z: {aruco_pose_matrix.rotation.z:.2f}, w: {aruco_pose_matrix.rotation.w:.2f}]")
        

        # Estrai la posizione (traslazione) dal risultato finale
        aruco_position = tf_transformations.translation_from_matrix(aruco_pose_matrix)

        # Estrai l'orientamento (quaternione) dal risultato finale
        aruco_orientation = tf_transformations.quaternion_from_matrix(aruco_pose_matrix)

        # Stampa la posizione
        print("Posizione del marker rispetto a map:")
        print(f"x: {aruco_position[0]:.3f}, y: {aruco_position[1]:.3f}, z: {aruco_position[2]:.3f}")

        # Stampa l'orientamento
        print("Orientamento del marker (quaternione):")
        print(f"x: {aruco_orientation[0]:.3f}, y: {aruco_orientation[1]:.3f}, z: {aruco_orientation[2]:.3f}, w: {aruco_orientation[3]:.3f}")

        # Pubblica la trasformazione come TF
        tf_transform = TransformStamped()
        tf_transform.header.stamp = self.get_clock().now().to_msg()  # Timestamp attuale
        tf_transform.header.frame_id = "map"  # Frame di riferimento 
        tf_transform.child_frame_id = "aruco_marker_frame"  # Frame dell'ArUco marker

        # Posizione
        tf_transform.transform.translation.x = aruco_position[0]
        tf_transform.transform.translation.y = aruco_position[1]
        tf_transform.transform.translation.z = aruco_position[2]

        # Orientamento
        tf_transform.transform.rotation.x = aruco_orientation[0]
        tf_transform.transform.rotation.y = aruco_orientation[1]
        tf_transform.transform.rotation.z = aruco_orientation[2]
        tf_transform.transform.rotation.w = aruco_orientation[3]

        # Invia la trasformazione
        self.tf_broadcaster.sendTransform(tf_transform)

        self.get_logger().info(f"TF Trasformazione inviata - Frame padre: {tf_transform.header.frame_id}, "
                                f"Frame figlio: {tf_transform.child_frame_id}, "
                                f"Posizione: x={tf_transform.transform.translation.x:.2f}, "
                                f"y={tf_transform.transform.translation.y:.2f}, "
                                f"z={tf_transform.transform.translation.z:.2f}, "
                                f"Orientamento: x={tf_transform.transform.rotation.x:.2f}, "
                                f"y={tf_transform.transform.rotation.y:.2f}, "
                                f"z={tf_transform.transform.rotation.z:.2f}, "
                                f"w={tf_transform.transform.rotation.w:.2f}")

        
        
        
   
def main():
    rclpy.init()

    # Crea il nodo e il subscriber per l'ArUco
    node = ArucoNavigation()

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
    target_order = ["Goal_1", "Goal_2"]
    goal_poses = [pose for pose, name in goals if name in target_order]

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer="smoother_server")

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    
    while not navigator.isTaskComplete():

        # Do something with the feedback
        i = i + 1

        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            if (feedback.current_waypoint + 1) == 2:
                # Aspetta la rilevazione dell'ArUco marker
                print("Looking for ArUco marker...")
                rclpy.spin_once(node)

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=1000):
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
    node.destroy_node()
    exit(0)


if __name__ == '__main__':
    main()