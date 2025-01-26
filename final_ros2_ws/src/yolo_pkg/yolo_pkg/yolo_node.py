from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
import cv2
import math
from geometry_msgs.msg import PoseStamped


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # Subscriptions
        self.subscription = self.create_subscription(
            Image,
            '/image',  # Topic name from your camera
            self.image_callback,
            10
        )
        
        # Publishers
        self.image_publisher = self.create_publisher(Image, '/yolo/output_image', 10)
        self.object_position_publisher = self.create_publisher(PoseStamped, '/detected_object_position', 10)
        self.bool_publisher = self.create_publisher(Bool, '/object_detected', 10)  # Bool publisher

        # YOLO Model and configuration
        self.bridge = CvBridge()
        self.model = YOLO('/ros2_ws/src/yolo_pkg/models/bakedetectionmodel.pt')  # Path to your YOLO model
        self.iou_threshold = 0.5  # Set the IoU threshold for filtering
        
        # Camera parameters
        self.camera_height = 0.08  # 8 cm = 0.08 meters
        self.beacon_height = 0.32  # 32 cm = 0.32 meters
        self.focal_length = 272  # Focal length in pixels (example value)
        self.image_height = 240  # Image height in pixels (example value)
        self.camera_frame = 'camera_link'  # The frame ID of the camera
        
        self.get_logger().info('YOLO Node Initialized')

    def image_callback(self, msg):
        self.get_logger().info('Received image')
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run YOLO inference
            results = self.model(cv_image)
            detections = results[0].boxes.xyxy.cpu().numpy()  # Extract bounding boxes
            confidences = results[0].boxes.conf.cpu().numpy()  # Extract confidence scores

            # Filter redundant boxes using IoU
            filtered_indices = self.filter_boxes(detections, confidences)
            filtered_detections = detections[filtered_indices]
            filtered_confidences = confidences[filtered_indices]

            # Initialize object detected flag
            object_detected = len(filtered_detections) > 0

            # Publish Bool message indicating object detection
            bool_msg = Bool()
            bool_msg.data = object_detected
            self.bool_publisher.publish(bool_msg)

            if object_detected:
                self.get_logger().info("Object detected!")

            # Annotate the filtered boxes on the image and calculate distances
            for box, confidence in zip(filtered_detections, filtered_confidences):
                x1, y1, x2, y2 = map(int, box[:4])  # Bounding box coordinates
                label = f"Bake: {confidence:.2f}"  # Confidence score label

                # Calculate the bounding box height in pixels
                bounding_box_height = y2 - y1  # Height of the bounding box

                # Calculate the distance using focal length method
                distance = self.calculate_distance(bounding_box_height)

                # Add the distance to the label
                label += f" Dist: {distance:.2f}m"

                # Draw bounding box and label
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw bounding box
                cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.5, (0, 255, 0), 2)  # Display confidence and distance above the box

                # Calculate the object's position relative to the robot
                object_position = self.calculate_object_position(distance, x1, x2, cv_image.shape[1])

                # Publish the object's position as a PoseStamped message
                self.publish_object_position(object_position)

            # Publish the annotated image
            output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.image_publisher.publish(output_msg)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def filter_boxes(self, boxes, confidences):
        """Filters bounding boxes using IoU to remove duplicates."""
        filtered_indices = []
        sorted_indices = confidences.argsort()[::-1]  # Sort boxes by confidence

        while len(sorted_indices) > 0:
            current = sorted_indices[0]
            filtered_indices.append(current)
            sorted_indices = sorted_indices[1:]  # Remove the current box

            # Compute IoU for the remaining boxes
            sorted_indices = [
                i for i in sorted_indices
                if self.iou(boxes[current], boxes[i]) < self.iou_threshold
            ]

        return filtered_indices

    def iou(self, box1, box2):
        """Calculates Intersection over Union (IoU) between two boxes."""
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])

        inter_area = max(0, x2 - x1) * max(0, y2 - y1)
        box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
        box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union_area = box1_area + box2_area - inter_area

        return inter_area / union_area if union_area > 0 else 0

    def calculate_distance(self, bounding_box_height_in_image):
        """Calculate the distance to the object using focal length and bounding box height."""
        return (self.beacon_height * self.focal_length) / bounding_box_height_in_image

    def calculate_object_position(self, distance, x1, x2, image_width):
        """Calculate the object's position in the robot's coordinate frame."""
        bounding_box_center = (x1 + x2) / 2.0
        angle_offset = math.atan((bounding_box_center - image_width / 2.0) / self.focal_length)

        object_x = distance * math.cos(angle_offset)
        object_y = distance * math.sin(angle_offset)
        object_z = self.camera_height

        return (object_x, object_y, object_z)

    def publish_object_position(self, position):
        """Publish the object's position as a PoseStamped message."""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.camera_frame

        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.w = 1.0

        self.object_position_publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()