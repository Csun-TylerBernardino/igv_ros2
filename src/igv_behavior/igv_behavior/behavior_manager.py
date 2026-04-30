import rclpy
from rclpy.node import Node

from igv_interfaces.msg import BehaviorState, LaneEstimate, ObstacleSummary


def clamp(value, low, high):
    return max(low, min(high, value))


def apply_deadband(value, deadband):
    if abs(value) < deadband:
        return 0.0
    return value


class BehaviorManager(Node):
    def __init__(self):
        super().__init__('behavior_manager')

        self.publisher_ = self.create_publisher(BehaviorState, '/behavior_state', 10)

        self.lane_sub = self.create_subscription(
            LaneEstimate,
            '/lane_estimate',
            self.lane_callback,
            10
        )

        self.obstacle_sub = self.create_subscription(
            ObstacleSummary,
            '/obstacle_summary',
            self.obstacle_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.publish_state)

        self.mode = 'AUTONOMOUS'
        self.state = 'IDLE'
        self.reason = 'waiting_for_lane_and_obstacle_inputs'
        self.target_speed_mps = 0.0
        self.target_steering_rad = 0.0

        self.latest_lane_valid = False
        self.latest_lane_confidence = 0.0
        self.latest_lateral_error = 0.0
        self.latest_heading_error = 0.0
        self.latest_lane_time_ns = 0

        self.filtered_lateral_error = 0.0
        self.filtered_heading_error = 0.0

        self.obstacle_ahead = False
        self.obstacle_distance = 999.0
        self.latest_obstacle_time_ns = 0

        # Controller tuning
        self.filter_alpha = 0.15
        self.k_heading = 0.45
        self.k_lateral = 0.55

        self.heading_deadband = 0.03
        self.lateral_deadband = 0.03

        self.max_steering = 0.45
        self.fast_speed = 0.50
        self.medium_speed = 0.30
        self.slow_speed = 0.18

        # State timing
        self.lane_timeout_sec = 1.0
        self.clear_hold_sec = 0.75

        # Search behavior
        self.search_turn_rate = 0.10

        self.get_logger().info('Behavior manager started')

    def lane_callback(self, msg: LaneEstimate):
        self.latest_lane_valid = msg.valid
        self.latest_lane_confidence = msg.confidence
        self.latest_lateral_error = msg.lateral_error_m
        self.latest_heading_error = msg.heading_error_rad
        self.latest_lane_time_ns = self.get_clock().now().nanoseconds

        self.filtered_lateral_error = (
            (1.0 - self.filter_alpha) * self.filtered_lateral_error
            + self.filter_alpha * self.latest_lateral_error
        )
        self.filtered_heading_error = (
            (1.0 - self.filter_alpha) * self.filtered_heading_error
            + self.filter_alpha * self.latest_heading_error
        )

        self.update_behavior()

    def obstacle_callback(self, msg: ObstacleSummary):
        self.obstacle_ahead = msg.obstacle_ahead
        self.obstacle_distance = msg.nearest_obstacle_distance_m

        if self.obstacle_ahead:
            self.latest_obstacle_time_ns = self.get_clock().now().nanoseconds

        self.update_behavior()

    def lane_is_fresh(self):
        if self.latest_lane_time_ns == 0:
            return False
        age_sec = (self.get_clock().now().nanoseconds - self.latest_lane_time_ns) / 1e9
        return age_sec <= self.lane_timeout_sec

    def obstacle_recently_seen(self):
        if self.latest_obstacle_time_ns == 0:
            return False
        age_sec = (self.get_clock().now().nanoseconds - self.latest_obstacle_time_ns) / 1e9
        return age_sec <= self.clear_hold_sec

    def update_behavior(self):
        # 1) Hard obstacle stop
        if self.obstacle_ahead and self.obstacle_distance < 3.0:
            self.state = 'STOP'
            self.reason = 'obstacle_ahead'
            self.target_speed_mps = 0.0
            self.target_steering_rad = 0.0
            return

        # 2) Brief hold after obstacle clears to avoid chatter
        if self.obstacle_recently_seen():
            self.state = 'WAIT_FOR_CLEAR'
            self.reason = 'obstacle_recently_cleared'
            self.target_speed_mps = 0.0
            self.target_steering_rad = 0.0
            return

        # 3) Normal lane following
        if self.lane_is_fresh() and self.latest_lane_valid:
            lat = apply_deadband(self.filtered_lateral_error, self.lateral_deadband)
            hdg = apply_deadband(self.filtered_heading_error, self.heading_deadband)

            steering = -(self.k_heading * hdg + self.k_lateral * lat)
            steering = clamp(steering, -self.max_steering, self.max_steering)

            abs_steer = abs(steering)
            if abs_steer < 0.15:
                speed = self.fast_speed
            elif abs_steer < 0.35:
                speed = self.medium_speed
            else:
                speed = self.slow_speed

            self.state = 'LANE_KEEP'
            self.reason = 'valid_lane_detected'
            self.target_speed_mps = speed
            self.target_steering_rad = steering
            return

        # 4) Lane missing/stale
        self.state = 'SEARCH_LANE'
        if not self.lane_is_fresh():
            self.reason = 'lane_timeout'
        else:
            self.reason = 'lane_invalid'

        # Safe placeholder search behavior
        self.target_speed_mps = 0.0
        self.target_steering_rad = self.search_turn_rate

    def publish_state(self):
        self.update_behavior()

        msg = BehaviorState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.mode = self.mode
        msg.state = self.state
        msg.reason = self.reason
        msg.target_speed_mps = self.target_speed_mps
        msg.target_steering_rad = self.target_steering_rad

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
