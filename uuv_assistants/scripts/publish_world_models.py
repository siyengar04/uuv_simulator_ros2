#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
from tf_quaternion.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from gazebo_msgs.srv import GetModelState

class WorldPublisher(Node):
    def __init__(self):
        super().__init__('publish_world_models')
        self._model_paths = dict()
        self._get_model_state = None

        self.declare_parameter('meshes', {})
        meshes = self.get_parameter('meshes').value
        if type(meshes) != dict:
            self.get_logger().error('A dictionary of mesh filenames is required')
            sys.exit(1)
        self.add_meshes(meshes)

        self._mesh_topic = self.create_publisher(MarkerArray, '/world_models', 1)
        self.create_timer(10.0, self.publish_meshes)

    def add_meshes(self, models):
        for model in models:
            if model in self._model_paths:
                self.get_logger().info(f'Model {model} already exists')
                continue
            new_model = dict()
            new_model['position'] = [0, 0, 0]
            new_model['orientation'] = [0, 0, 0, 1]
            new_model['scale'] = [1, 1, 1]

            if 'pose' in models[model]:
                if 'position' in models[model]['pose'] and len(models[model]['pose']['position']) == 3:
                    new_model['position'] = models[model]['pose']['position']
                if 'orientation' in models[model]['pose'] and len(models[model]['pose']['orientation']) == 3:
                    new_model['orientation'] = quaternion_from_euler(*models[model]['pose']['orientation'])

            if 'scale' in models[model] and len(models[model]['scale']) == 3:
                new_model['scale'] = models[model]['scale']

            if 'mesh' in models[model]:
                new_model['mesh'] = models[model]['mesh']
                if 'model' in models[model]:
                    model_name = models[model]['model']
                    if self._get_model_state is None:
                        self._get_model_state = self.create_client(GetModelState, '/gazebo/get_model_state')
                        if not self._get_model_state.wait_for_service(timeout_sec=5.0):
                            self.get_logger().error('/gazebo/get_model_state service is unavailable')
                    req = GetModelState.Request()
                    req.model_name = model_name
                    future = self._get_model_state.call_async(req)
                    rclpy.spin_until_future_complete(self, future)
                    if future.result() is not None and future.result().success:
                        prop = future.result()
                        new_model['position'] = [prop.pose.position.x,
                                                 prop.pose.position.y,
                                                 prop.pose.position.z]
                        new_model['orientation'] = [prop.pose.orientation.x,
                                                    prop.pose.orientation.y,
                                                    prop.pose.orientation.z,
                                                    prop.pose.orientation.w]
                    else:
                        self.get_logger().warn(f'Model {model} not found in the current Gazebo scenario')
                else:
                    self.get_logger().warn(f'Information about model {model} for mesh {models[model]["mesh"]} could not be retrieved')
            elif 'plane' in models[model]:
                new_model['plane'] = models[model]['plane'] if len(models[model]['plane']) == 3 else [1, 1, 1]
            else:
                continue

            self._model_paths[model] = new_model
            self.get_logger().info(f'New model being published: {model}')
            self.get_logger().info(f'\t Position: {new_model["position"]}')
            self.get_logger().info(f'\t Orientation: {new_model["orientation"]}')
            self.get_logger().info(f'\t Scale: {new_model["scale"]}')

    def publish_meshes(self):
        markers = MarkerArray()
        i = 0
        total_models = len(self._model_paths.keys())
        for model in self._model_paths:
            marker = Marker()
            if 'mesh' in self._model_paths[model]:
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = self._model_paths[model]['mesh']
                marker.scale.x = self._model_paths[model]['scale'][0]
                marker.scale.y = self._model_paths[model]['scale'][1]
                marker.scale.z = self._model_paths[model]['scale'][2]
            elif 'plane' in self._model_paths[model]:
                marker.type = Marker.CUBE
                marker.scale.x = self._model_paths[model]['plane'][0]
                marker.scale.y = self._model_paths[model]['plane'][1]
                marker.scale.z = self._model_paths[model]['plane'][2]
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ''
            marker.id = i
            marker.action = Marker.ADD
            marker.pose.position.x = self._model_paths[model]['position'][0]
            marker.pose.position.y = self._model_paths[model]['position'][1]
            marker.pose.position.z = self._model_paths[model]['position'][2]
            marker.pose.orientation.x = self._model_paths[model]['orientation'][0]
            marker.pose.orientation.y = self._model_paths[model]['orientation'][1]
            marker.pose.orientation.z = self._model_paths[model]['orientation'][2]
            marker.pose.orientation.w = self._model_paths[model]['orientation'][3]
            marker.color.a = 0.2
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0 - float(i) / total_models
            markers.markers.append(marker)
            i += 1
        self._mesh_topic.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = WorldPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
