#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import time
import math


class SensorSimulator(Node):
    def __init__(self):
        super().__init__("sensor_simulator")

        # Create publishers for all sensors
        self._sensor_publishers = {
            "lidar": self.create_publisher(Range, "/sensor/lidar", 10),
            "izq_front": self.create_publisher(Range, "/sensor/izq_front", 10),
            "der_front": self.create_publisher(Range, "/sensor/der_front", 10),
            "izq_back": self.create_publisher(Range, "/sensor/izq_back", 10),
            "der_back": self.create_publisher(Range, "/sensor/der_back", 10),
        }

        # Timer for 50ms publishing rate (20Hz)
        self.timer = self.create_timer(0.05, self.publish_sensor_data)

        # Simulation parameters
        self.start_time = time.time()
        self.duration = 5.0  # 2 seconds
        self.message_count = 0

        self.get_logger().info(
            "Sensor simulator started - publishing for 2 seconds at 50ms intervals"
        )

    def publish_sensor_data(self):
        current_time = time.time()
        elapsed = current_time - self.start_time

        # Stop after 2 seconds
        if elapsed > self.duration:
            self.get_logger().info(
                f"Simulation complete - published {self.message_count} messages"
            )
            rclpy.shutdown()
            # self.timer.cancel()
            return

        # Create base Range message
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.1
        range_msg.min_range = 0.02
        range_msg.max_range = 4.0

        # Simulate different scenarios
        self.simulate_approaching_obstacle(range_msg, elapsed)

        self.message_count += 1

    def simulate_approaching_obstacle(self, base_msg, elapsed):
        """Simulate an obstacle approaching the front of the robot"""
        # Obstacle starts at 2.0m and approaches to 0.2m over 2 seconds
        front_distance = 0.15  # 2.0 - (elapsed / 2.0) * 1.8

        # Front sensors detect the approaching obstacle
        front_msg = Range()
        front_msg.header = base_msg.header
        front_msg.header.frame_id = "lidar_frame"
        front_msg.radiation_type = base_msg.radiation_type
        front_msg.field_of_view = base_msg.field_of_view
        front_msg.min_range = base_msg.min_range
        front_msg.max_range = base_msg.max_range
        front_msg.range = max(0.2, front_distance)  # Don't go below 0.2m

        # Side front sensors see obstacle at slightly different distances
        side_msg = Range()
        side_msg.header = base_msg.header
        side_msg.radiation_type = base_msg.radiation_type
        side_msg.field_of_view = base_msg.field_of_view
        side_msg.min_range = base_msg.min_range
        side_msg.max_range = base_msg.max_range

        # Back sensors see clear path
        back_msg = Range()
        back_msg.header = base_msg.header
        back_msg.radiation_type = base_msg.radiation_type
        back_msg.field_of_view = base_msg.field_of_view
        back_msg.min_range = base_msg.min_range
        back_msg.max_range = base_msg.max_range
        back_msg.range = 3.0  # Clear behind

        # Publish data
        self._sensor_publishers["lidar"].publish(front_msg)

        # Side sensors see obstacle at angle (slightly farther)
        side_msg.header.frame_id = "izq_front_frame"
        side_msg.range = max(0.3, front_distance + 0.2)
        self._sensor_publishers["izq_front"].publish(side_msg)

        side_msg.header.frame_id = "der_front_frame"
        side_msg.range = max(0.3, front_distance + 0.2)
        self._sensor_publishers["der_front"].publish(side_msg)

        # Back sensors
        back_msg.header.frame_id = "izq_back_frame"
        self._sensor_publishers["izq_back"].publish(back_msg)

        back_msg.header.frame_id = "der_back_frame"
        self._sensor_publishers["der_back"].publish(back_msg)

        if self.message_count % 10 == 0:  # Log every 500ms
            self.get_logger().info(f"Front distance: {front_msg.range:.2f}m")

    def simulate_static_obstacle(self, base_msg, elapsed):
        """Simulate a static obstacle at fixed distance"""
        # All sensors report fixed distances
        distances = {
            "lidar": 0.25,  # Obstacle close in front
            "izq_front": 1.0,  # Clear on front-left
            "der_front": 1.0,  # Clear on front-right
            "izq_back": 2.0,  # Clear behind
            "der_back": 2.0,  # Clear behind
        }

        for sensor_name, distance in distances.items():
            msg = Range()
            msg.header = base_msg.header
            msg.header.frame_id = f"{sensor_name}_frame"
            msg.radiation_type = base_msg.radiation_type
            msg.field_of_view = base_msg.field_of_view
            msg.min_range = base_msg.min_range
            msg.max_range = base_msg.max_range
            msg.range = distance

            self._sensor_publishers[sensor_name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    simulator = SensorSimulator()

    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        # rclpy.shutdown()


if __name__ == "__main__":
    main()
