#!/usr/bin/env python3
import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from irobot_create_msgs.action import DriveDistance, RotateAngle


class TridentNavigator(Node):
    def __init__(self):
        super().__init__('trident_navigator')

        # --- COURSE DIMENSIONS (in meters & radians) ---
        self.stem_dist = 1.0          # Start to 4-way intersection
        self.straight_branch = 1.0    # Intersection straight to Red
        self.side_out_dist = 1.0      # Intersection left/right out to side lane
        self.side_up_dist = 1.0       # Side lane forward up to Blue/Yellow
        self.turn_90 = math.pi / 2.0  # 1.5708 rad (90 degrees)

        # Speeds
        self.drive_speed = 0.20       # meters/sec
        self.turn_speed = 0.50        # radians/sec
        self.unload_wait_sec = 3.0    # Pause at drop-off zone

        self.is_busy = False

        # Action clients
        self.drive_client = ActionClient(self, DriveDistance, '/drive_distance')
        self.rotate_client = ActionClient(self, RotateAngle, '/rotate_angle')

        # Subscribe to color topic & publish status
        self.color_sub = self.create_subscription(String, '/target_color', self.color_callback, 10)
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
        threading.Thread(target=self.run_mission_thread, args=(color,), daemon=True).start()

    def run_mission_thread(self, color: str):
        try:
            self.get_logger().info(f'Starting mission for {color.upper()} block...')
            self.execute_mission(color)
        finally:
            self.is_busy = False

    def drive(self, distance_m: float):
        goal = DriveDistance.Goal()
        goal.distance = float(distance_m)
        goal.max_translation_speed = self.drive_speed
        done_event = threading.Event()
        send_future = self.drive_client.send_goal_async(goal)

        def on_goal_response(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().error('Drive goal rejected!')
                done_event.set()
                return
            goal_handle.get_result_async().add_done_callback(lambda _: done_event.set())

        send_future.add_done_callback(on_goal_response)
        done_event.wait()

    def rotate(self, angle_rad: float):
        goal = RotateAngle.Goal()
        goal.angle = float(angle_rad)
        goal.max_rotation_speed = self.turn_speed
        done_event = threading.Event()
        send_future = self.rotate_client.send_goal_async(goal)

        def on_goal_response(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().error('Rotate goal rejected!')
                done_event.set()
                return
            goal_handle.get_result_async().add_done_callback(lambda _: done_event.set())

        send_future.add_done_callback(on_goal_response)
        done_event.wait()

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def execute_mission(self, color: str):
        if color == 'red':
            # Continuous 2.0m straight shot from Start to Red
            self.get_logger().info('Driving straight 2.0m to RED...')
            self.drive(self.stem_dist + self.straight_branch)

            self.publish_status('arrived_red')
            self.get_logger().info('Arrived at RED! Waiting 3s...')
            time.sleep(self.unload_wait_sec)

            # Return to center intersection facing forward
            self.get_logger().info('Returning to center intersection from RED...')
            self.rotate(math.pi)                  # Face south
            self.drive(self.straight_branch)      # Drive 1.0m back to intersection
            self.rotate(math.pi)                  # Turn 180 to face forward (north)

        elif color == 'blue':
            self.get_logger().info('Driving 1.0m to intersection...')
            self.drive(self.stem_dist)
            self.get_logger().info('Turning left and driving to BLUE...')
            self.rotate(self.turn_90)             # Face west (left)
            self.drive(self.side_out_dist)        # Drive 1.0m out
            self.rotate(-self.turn_90)            # Face north (forward)
            self.drive(self.side_up_dist)         # Drive 1.0m up to Blue

            self.publish_status('arrived_blue')
            self.get_logger().info('Arrived at BLUE! Waiting 3s...')
            time.sleep(self.unload_wait_sec)

            # Return to center intersection facing forward
            self.get_logger().info('Returning to center intersection from BLUE...')
            self.rotate(math.pi)                  # Face south
            self.drive(self.side_up_dist)         # Drive 1.0m down to cross-lane
            self.rotate(self.turn_90)             # Turn left (+90) to face east (toward center)
            self.drive(self.side_out_dist)        # Drive 1.0m back to center intersection
            self.rotate(self.turn_90)             # Turn left (+90) to face forward (north)

        elif color == 'yellow':
            self.get_logger().info('Driving 1.0m to intersection...')
            self.drive(self.stem_dist)
            self.get_logger().info('Turning right and driving to YELLOW...')
            self.rotate(-self.turn_90)            # Face east (right)
            self.drive(self.side_out_dist)        # Drive 1.0m out
            self.rotate(self.turn_90)             # Face north (forward)
            self.drive(self.side_up_dist)         # Drive 1.0m up to Yellow

            self.publish_status('arrived_yellow')
            self.get_logger().info('Arrived at YELLOW! Waiting 3s...')
            time.sleep(self.unload_wait_sec)

            # Return to center intersection facing forward
            self.get_logger().info('Returning to center intersection from YELLOW...')
            self.rotate(math.pi)                  # Face south
            self.drive(self.side_up_dist)         # Drive 1.0m down to cross-lane
            self.rotate(-self.turn_90)            # Turn right (-90) to face west (toward center)
            self.drive(self.side_out_dist)        # Drive 1.0m back to center intersection
            self.rotate(-self.turn_90)            # Turn right (-90) to face forward (north)

        self.publish_status('at_intersection')
        self.get_logger().info('At center intersection facing forward! Ready for next block.')


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
