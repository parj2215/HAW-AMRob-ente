import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import time
from std_msgs.msg import Bool

"""
TODO: ADD RETURN TO START STATE
TODO: ADD OBECT DETECTION SUBSCRIBER
TEST: TEST THE FSM
TODO: MEASURE THE ROOM
"""

# FSM states
class States:
    START = "START"
    ALIGN_TO_WALL = "ALIGN_TO_WALL"         # Align the robot to the wall
    MOVE_TOWARD_WALL = "MOVE_TOWARD_WALL"
    TURN_LEFT_OR_RIGHT = "TURN_LEFT_OR_RIGHT"
    MOVE_FORWARD = "MOVE_FORWARD"
    RETURN_TO_START = "RETURN_TO_START"
    AVOID_COLLISION = "AVOID_COLLISION"      # Avoid obstacles
    STOP = "STOP"

class SelfNavigationFSM(Node):
    def __init__(self):
        super().__init__('self_navigation_fsm')

        # Parameters
        self.distance_to_wall_threshold = 1.0  # meters
        self.forward_distance = 2.5           # meters
        self.collision_threshold = 1.0        # meters
        self.object_detected = False
        self.exploration_speed = 1530
        self.stop_speed = 1500
        self.scan_topic = '/scan'
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 9600

        # Serial connection to Arduino
        self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        time.sleep(2)  # Wait for the serial connection to initialize

        # State variables
        self.current_state = States.START
        self.wall_side = None  # 'left' or 'right'
        self.turns_completed = 0

        # Subscribers
        self.lidar_subscriber = self.create_subscription(
            LaserScan, self.scan_topic, self.lidar_callback, 10
        )

        self.yolo_sub = self.create_subscription(Bool, '/object_detected', self.yolo_callback, 10)


        # Movement commands
        self.stop_robot()

    def send_command(self, pwm_dcmotor, angle_servo):
        command = f"{pwm_dcmotor},{angle_servo}\n"
        self.arduino.write(command.encode('utf-8'))

    def stop_robot(self):
        self.send_command(self.stop_speed, 90)  # Stop motor and center servo

    def yolo_callback(self, msg):
        """
        Callback for processing YOLO object detection messages.
        """
        self.object_detected = msg.data  # Assuming msg.data is a boolean
        if self.object_detected:
            self.get_logger().info("Object detected! Returning to start.")
            self.current_state = States.RETURN_TO_START

    def lidar_callback(self, msg):
        # Process Lidar data
        ranges = msg.ranges
        front_distance = min(ranges[0:10] + ranges[-10:])  # Front section of Lidar
        left_distance = min(ranges[80:100])
        right_distance = min(ranges[260:280])
        front_left = min(ranges[20:40])  # Front-left sector
        front_right = min(ranges[320:340])  # Front-right sector

        # Check for obstacles
        if front_distance < self.collision_threshold:
            self.current_state = States.AVOID_COLLISION

        # FSM transitions
        if self.current_state == States.START:
            self.current_state = States.ALIGN_TO_WALL

        elif self.current_state == States.ALIGN_TO_WALL:
            # Determine angle correction
            # if abs(front_left - front_right) > 0.1:  # Threshold to decide if correction is needed
            #     if front_left > front_right:
            #         self.send_command(self.exploration_speed, 120)  # Turn right
            #     else:
            #         self.send_command(self.exploration_speed, 60)  # Turn left
            #     time.sleep(0.5)  # Adjust time based on angular correction speed
            #     self.send_command(self.exploration_speed, 90)  # Center the servo
            # else:
            #     self.current_state = States.MOVE_TOWARD_WALL

            self.send_command(self.exploration_speed, 120)  # Turn left
            time.sleep(0.5)  # Adjust time based on angular correction speed
            self.send_command(self.exploration_speed, 90)  # Center the servo
            self.current_state = States.MOVE_TOWARD_WALL

        elif self.current_state == States.MOVE_TOWARD_WALL:
            if front_distance < self.distance_to_wall_threshold:
                self.current_state = States.TURN_LEFT_OR_RIGHT
                self.wall_side = 'left' if left_distance < right_distance else 'right'

        elif self.current_state == States.TURN_LEFT_OR_RIGHT:
            self.turns_completed += 1
            if self.turns_completed >= 4:
                self.current_state = States.MOVE_FORWARD

        elif self.current_state == States.MOVE_FORWARD:
            if front_distance > self.forward_distance:
                self.current_state = States.RETURN_TO_START
            elif self.object_detected:
                self.current_state = States.RETURN_TO_START

        elif self.current_state == States.RETURN_TO_START:
            self.current_state = States.STOP

        self.execute_state()

    def execute_state(self):
        if self.current_state == States.MOVE_TOWARD_WALL:
            self.send_command(self.exploration_speed, 90)  # Move forward

        elif self.current_state == States.TURN_LEFT_OR_RIGHT:
            angle = 60 if self.wall_side == 'left' else 120
            self.send_command(self.exploration_speed, angle)  # Turn left or right
            time.sleep(1.5)  # Wait for the turn to complete
            self.send_command(self.exploration_speed, 90)  # Center the servo
            self.current_state = States.MOVE_TOWARD_WALL

        elif self.current_state == States.MOVE_FORWARD:
            self.send_command(self.exploration_speed, 90)  # Move forward
            time.sleep(5)  # Simulate moving a specific distance
            self.stop_robot()

        elif self.current_state == States.RETURN_TO_START:
            angle = 60 if self.wall_side == 'left' else 120
            self.send_command(self.exploration_speed, angle)  # Turn left or right
            time.sleep(1.5)
            self.send_command(self.exploration_speed, 90)  # Move forward to return
            time.sleep(5)  # Simulate returning
            self.stop_robot()

        elif self.current_state == States.AVOID_COLLISION:
            self.stop_robot()
            self.get_logger().info("Collision detected! Stopping robot.")
            time.sleep(1)  # Pause to assess the situation
            angle = 120 if self.wall_side == 'left' else 60  # Turn away from the obstacle
            self.send_command(self.exploration_speed, angle)
            time.sleep(1.5)  # Wait for the turn to complete
            self.send_command(self.exploration_speed, 90)  # Move forward slightly
            time.sleep(2)
            self.stop_robot()
            self.current_state = States.MOVE_TOWARD_WALL

        elif self.current_state == States.STOP:
            self.stop_robot()

            self.get_logger().info("Navigation complete. Robot returned to start.")


def main(args=None):
    rclpy.init(args=args)
    fsm = SelfNavigationFSM()
    rclpy.spin(fsm)
    fsm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
