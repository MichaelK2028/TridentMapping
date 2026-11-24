#!/usr/bin/env python3
import math
import time
from importlib import import_module
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String

try:
    _create_actions = import_module('irobot_create_msgs.action')
    DriveDistance = _create_actions.DriveDistance
    RotateAngle = _create_actions.RotateAngle
except (ImportError, AttributeError) as exc:
    raise ImportError(
        'irobot_create_msgs actions are unavailable. Install the package and '
        'source the ROS 2 workspace before running this node.'
    ) from exc


class TridentNavigator(Node):
    def __init__(self):
        super().__init__('trident_navigator')

        # --- COURSE DIMENSIONS (in meters & radians) ---
        self.stem_dist = 1.0          # Start to 4-way intersection
        self.straight_branch = 1.0    # Intersection straight to Red
        self.side_out_dist = 1.0      # Intersection left/right out to side lane
        self.side_up_dist = 1.0       # Side lane forward up to Blue/Yellow
        self.turn_90 = math.pi / 2.0  # 1.5708 rad (90 degrees)

        # Speeds (safe indoor testing speeds)
        self.drive_speed = 0.20       # meters/sec
        self.turn_speed = 0.50        # radians/sec
        self.unload_wait_sec = 3.0    # Pause at drop-off zone

        self.is_busy = False

        # Action clients for Create 3 built-in movement
        self.drive_client = ActionClient(self, DriveDistance, '/drive_distance')
        self.rotate_client = ActionClient(self, RotateAngle, '/rotate_angle')

        # Subscribe to your teammate's color detection topic
        self.color_sub = self.create_subscription(
            String,
            '/target_color',
            self.color_callback,
            10
        )

        # Publish status when arrived at drop-off or returned to start
        self.status_pub = self.create_publisher(String, '/nav_status', 10)

        self.get_logger().info('Waiting for Create 3 movement action servers...')
        self.drive_client.wait_for_server()
        self.rotate_client.wait_for_server()
        self.get_logger().info('Ready! Publish "blue", "red", or "yellow" to /target_color.')

    def color_callback(self, msg: String):
        color = msg.data.strip().lower()
        if self.is_busy:
            self.get_logger().warn(f'Currently running a path. Ignoring "{color}".')
            return

        if color not in ['blue', 'red', 'yellow']:
            self.get_logger().warn(f'Unknown color "{color}". Expected: blue, red, or yellow.')
            return

        self.is_busy = True
        self.get_logger().info(f'Starting mission for {color.upper()} block...')
        self.execute_mission(color)
        self.is_busy = False

    def drive(self, distance_m: float):
        """Drive straight forward (+) or backward (-) in meters."""
        goal = DriveDistance.Goal()
        goal.distance = float(distance_m)
        goal.max_translation_speed = self.drive_speed

        send_future = self.drive_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Drive goal rejected!')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

    def rotate(self, angle_rad: float):
        """Rotate in place: positive (+) = counter-clockwise (left), negative (-) = clockwise (right)."""
        goal = RotateAngle.Goal()
        goal.angle = float(angle_rad)
        goal.max_rotation_speed = self.turn_speed

        send_future = self.rotate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Rotate goal rejected!')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def execute_mission(self, color: str):
        # 1. Drive stem from Start to intersection
        self.get_logger().info('Driving 1.0m to intersection...')
        self.drive(self.stem_dist)

        if color == 'red':
            # Go straight ahead 1.0m
            self.get_logger().info('Heading straight 1.0m to RED...')
            self.drive(self.straight_branch)

            self.publish_status('arrived_red')
            time.sleep(self.unload_wait_sec)

            # Return to Start: turn 180, drive 2.0m back, turn 180 to face forward
            self.get_logger().info('Returning to Start from RED...')
            self.rotate(math.pi)
            self.drive(self.straight_branch + self.stem_dist)
            self.rotate(math.pi)

        elif color == 'blue':
            # Left 90 -> 1.0m out -> Right 90 -> 1.0m up
            self.get_logger().info('Turning left and driving to BLUE...')
            self.rotate(self.turn_90)
            self.drive(self.side_out_dist)
            self.rotate(-self.turn_90)
            self.drive(self.side_up_dist)

            self.publish_status('arrived_blue')
            time.sleep(self.unload_wait_sec)

            # Return to Start: reverse path
            self.get_logger().info('Returning to Start from BLUE...')
            self.rotate(math.pi)
            self.drive(self.side_up_dist)
            self.rotate(-self.turn_90)
            self.drive(self.side_out_dist)
            self.rotate(self.turn_90)
            self.drive(self.stem_dist)
            self.rotate(math.pi)

        elif color == 'yellow':
            # Right 90 -> 1.0m out -> Left 90 -> 1.0m up
            self.get_logger().info('Turning right and driving to YELLOW...')
            self.rotate(-self.turn_90)
            self.drive(self.side_out_dist)
            self.rotate(self.turn_90)
            self.drive(self.side_up_dist)

            self.publish_status('arrived_yellow')
            time.sleep(self.unload_wait_sec)

            # Return to Start: reverse path
            self.get_logger().info('Returning to Start from YELLOW...')
            self.rotate(math.pi)
            self.drive(self.side_up_dist)
            self.rotate(self.turn_90)
            self.drive(self.side_out_dist)
            self.rotate(-self.turn_90)
            self.drive(self.stem_dist)
            self.rotate(math.pi)

        self.publish_status('at_start')
        self.get_logger().info('Mission complete! Ready for next block.')


def main(args=None):
    rclpy.init(args=args)
    node = TridentNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
