#!/usr/bin/env python3
#
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
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
#
# This source code is derived from gazebo_ros_pkgs
#   (https://github.com/ros-simulation/gazebo_ros_pkgs)
# * Copyright 2013 Open Source Robotics Foundation
# licensed under the Apache-2.0 license,
# cf. 3rd-party-licenses.txt file in the root directory of this source tree.
#
# The original code was modified to:
# - Allow overriding the initial position and orientation programmatically
#   from within another ROS node by setting parameters xref, rollref, etc.
import rclpy
from rclpy.node import Node
import sys
import os
import time
import warnings
import re

from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose, Quaternion
from std_srvs.srv import Empty
import tf_transformations as tft

try:
    import pymap3d

    use_pymap = True
except Exception as ex:
    print(
        "Package pymap3d is not available, WGS84 coordinates cannot be used\n"
        "Download pymap3d for Python 3.x as\n"
        ">> pip install pymap3d"
    )
    use_pymap = False

model_database_template = """<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://MODEL_NAME</uri>
    </include>
  </world>
</sdf>"""


def usage():
    """Print usage instructions for this script."""
    print(
        """Commands:
    -[urdf|sdf|trimesh|gazebo] - specify incoming xml is urdf, sdf or trimesh format.
    -[file|param|database] [<file_name>|<param_name>|<model_name>] - source of the model xml or the trimesh file
    -model <model_name> - name of the model to be spawned.
    -reference_frame <entity_name> - optional: name of the model/body where initial pose is defined.
                                     If left empty or specified as "world", Gazebo world frame is used.
    -gazebo_namespace <gazebo ros_namespace> - optional: ROS namespace of Gazebo offered ROS interfaces. Defaults to /gazebo/ (e.g. /gazebo/spawn_entity).
    -robot_namespace <robot ros_namespace> - optional: change ROS namespace of Gazebo plugins.
    -unpause - optional: !!!Experimental!!! unpause physics after spawning model
    -wait - optional: !!!Experimental!!! wait for model to exist
    -x <x in meters> - optional: initial pose, use 0 if left out
    -y <y in meters> - optional: initial pose, use 0 if left out
    -z <z in meters> - optional: initial pose, use 0 if left out
    -R <roll in radians> - optional: initial pose, use 0 if left out
    -P <pitch in radians> - optional: initial pose, use 0 if left out
    -Y <yaw in radians> - optional: initial pose, use 0 if left out
    -latitude <latitude in degrees> - optional: initial pose, will be ignored if left out
    -longitude <longitude in degrees> - optional: initial pose, will be ignored if left out
    -altitude <altitude in meters> - optional: initial pose, will be ignored if left out
    -latitude_ref <latitude of the origin in degrees> - optional: latitude of the world's origin, will be ignored if left out
    -longitude_ref <longitude of the origin in degrees> - optional: longitude of the world's origin, will be ignored if left out
    -altitude_ref <altitude of the origin in meters> - optional: altitude of the world's origin, will be ignored if left out
    """
    )
    sys.exit(1)


class SpawnModel(Node):

    def __init__(self):
        super().__init__("spawn_model")
        self.initial_xyz = [0.0, 0.0, 0.0]
        self.initial_rpy = [0.0, 0.0, 0.0]
        self.initial_geo = [None, None, None]
        self.ref_geo = [None, None, None]
        self.file_name = ""
        self.param_name = ""
        self.database_name = ""
        self.model_name = ""
        self.robot_namespace = self.get_namespace()
        self.gazebo_namespace = "/gazebo"
        self.reference_frame = ""
        self.unpause_physics = False
        self.wait_for_model = ""
        self.urdf_format = False
        self.sdf_format = False
        self.package_to_model = False

    def parse_user_inputs(self):
        # Parse command-line arguments
        for i in range(1, len(sys.argv)):
            if sys.argv[i] == "-h" or sys.argv[i] == "--help":
                usage()
            elif sys.argv[i] == "-urdf":
                self.urdf_format = True
            elif sys.argv[i] == "-sdf":
                self.sdf_format = True
            elif sys.argv[i] == "-file" and i + 1 < len(sys.argv):
                self.file_name = sys.argv[i + 1]
            elif sys.argv[i] == "-model" and i + 1 < len(sys.argv):
                self.model_name = sys.argv[i + 1]
            elif sys.argv[i] == "-x" and i + 1 < len(sys.argv):
                self.initial_xyz[0] = float(sys.argv[i + 1])
            elif sys.argv[i] == "-y" and i + 1 < len(sys.argv):
                self.initial_xyz[1] = float(sys.argv[i + 1])
            elif sys.argv[i] == "-z" and i + 1 < len(sys.argv):
                self.initial_xyz[2] = float(sys.argv[i + 1])
            elif sys.argv[i] == "-R" and i + 1 < len(sys.argv):
                self.initial_rpy[0] = float(sys.argv[i + 1])
            elif sys.argv[i] == "-P" and i + 1 < len(sys.argv):
                self.initial_rpy[1] = float(sys.argv[i + 1])
            elif sys.argv[i] == "-Y" and i + 1 < len(sys.argv):
                self.initial_rpy[2] = float(sys.argv[i + 1])
            elif sys.argv[i] == "-latitude" and i + 1 < len(sys.argv):
                self.initial_geo[0] = float(sys.argv[i + 1])
            elif sys.argv[i] == "-longitude" and i + 1 < len(sys.argv):
                self.initial_geo[1] = float(sys.argv[i + 1])
            elif sys.argv[i] == "-altitude" and i + 1 < len(sys.argv):
                self.initial_geo[2] = float(sys.argv[i + 1])
            elif sys.argv[i] == "-latitude_ref" and i + 1 < len(sys.argv):
                self.ref_geo[0] = float(sys.argv[i + 1])
            elif sys.argv[i] == "-longitude_ref" and i + 1 < len(sys.argv):
                self.ref_geo[1] = float(sys.argv[i + 1])
            elif sys.argv[i] == "-altitude_ref" and i + 1 < len(sys.argv):
                self.ref_geo[2] = float(sys.argv[i + 1])
            elif sys.argv[i] == "-package_to_model":
                self.package_to_model = True

        if not self.sdf_format and not self.urdf_format:
            self.get_logger().error(
                "Error: you must specify incoming format as either urdf or sdf format xml"
            )
            sys.exit(0)
        if self.model_name == "":
            self.get_logger().error("Error: you must specify model name")
            sys.exit(0)

    def call_spawn_service(self):
        # Load model XML
        if self.file_name:
            with open(self.file_name, "r") as f:
                model_xml = f.read()
        elif self.param_name:
            model_xml = self.get_parameter(self.param_name).value
        elif self.database_name:
            model_xml = model_database_template.replace(
                "MODEL_NAME", self.database_name
            )
        else:
            self.get_logger().error("Error: no model source specified")
            sys.exit(0)

        if self.package_to_model:
            model_xml = re.sub(
                r'<\s*mesh\s+filename\s*=\s*([\'"])(package://)',
                r"<mesh filename=\1model://",
                model_xml,
            )

        # Set initial pose
        initial_pose = Pose()
        if None not in self.initial_geo and None not in self.ref_geo and use_pymap:
            enu_pos = pymap3d.geodetic2enu(
                self.initial_geo[0],
                self.initial_geo[1],
                self.initial_geo[2],
                self.ref_geo[0],
                self.ref_geo[1],
                self.ref_geo[2],
            )
            initial_pose.position.x = enu_pos[0]
            initial_pose.position.y = enu_pos[1]
            initial_pose.position.z = enu_pos[2]
        else:
            initial_pose.position.x = self.initial_xyz[0]
            initial_pose.position.y = self.initial_xyz[1]
            initial_pose.position.z = self.initial_xyz[2]

        # Convert RPY to quaternion
        q = tft.quaternion_from_euler(
            self.initial_rpy[0], self.initial_rpy[1], self.initial_rpy[2]
        )
        initial_pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Call Gazebo spawn service
        spawn_client = self.create_client(
            SpawnEntity, f"{self.gazebo_namespace}/spawn_entity"
        )
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for spawn_entity service...")

        req = SpawnEntity.Request()
        req.name = self.model_name
        req.xml = model_xml
        req.initial_pose = initial_pose
        req.reference_frame = self.reference_frame if self.reference_frame else "world"

        future = spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Model spawned successfully")
        else:
            self.get_logger().error("Failed to spawn model")


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        usage()
    else:
        spawn_model = SpawnModel()
        spawn_model.parse_user_inputs()
        spawn_model.call_spawn_service()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
