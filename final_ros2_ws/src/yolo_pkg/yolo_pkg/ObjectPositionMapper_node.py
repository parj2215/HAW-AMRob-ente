import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point


class ObjectMarkerNode(Node):
    def __init__(self):
        super().__init__('object_marker_node')

        # Subscription to YOLO node's detected object positions
        self.subscription = self.create_subscription(
            PoseStamped,  # Assuming YOLO publishes PoseStamped for object positions
            '/detected_object_position',
            self.object_callback,
            10
        )

        # Marker publisher to RViz
        self.marker_publisher = self.create_publisher(Marker, '/object_marker', 10)

        # TF2 buffer and listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Object Marker Node Initialized')

    def object_callback(self, msg):
        try:
            # Lookup transform from camera frame to map frame
            transform = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                msg.header.frame_id,  # Source frame (e.g., camera_link)
                rclpy.time.Time()  # Use latest available transform
            )

            # Transform the detected object's position to the map frame
            transformed_point = do_transform_point(PointStamped(header=msg.header, point=msg.pose.position), transform)

            # Log the transformed position
            position = transformed_point.point
            self.get_logger().info(f"Object in map frame: x={position.x}, y={position.y}, z={position.z}")

            # Publish a marker for the object
            self.publish_marker(position)

        except Exception as e:
            self.get_logger().error(f"Failed to transform object position: {e}")

    def publish_marker(self, position):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'detected_objects'
        marker.id = 0  # You can assign unique IDs for multiple objects
        marker.type = Marker.SPHERE  # Shape of the marker
        marker.action = Marker.ADD

        # Set the position of the marker
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z

        # Set the orientation (no rotation)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set the scale (size) of the marker
        marker.scale.x = 0.2  # Diameter in x
        marker.scale.y = 0.2  # Diameter in y
        marker.scale.z = 0.2  # Diameter in z

        # Set the color of the marker (red, with full opacity)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Publish the marker
        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
