import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from QubotDev.routines import define_routines


class QubotWalk(Node):
    def __init__(self):
        super().__init__("qubot_walk")

        # Parameters for basic movement (used for individual commands)
        self.publish_interval = 0.05  # seconds (50ms)
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.movement_duration = 2.0  # seconds

        # Create publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        # self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Create subscriber for control_cmd
        self.control_cmd_sub = self.create_subscription(
            String, "/control_cmd", self.control_cmd_callback, 1
        )

        # Timer for continuous publishing
        self.movement_timer = None
        self.current_twist = Twist()
        self.movement_start_time = None

        # Routine execution state
        self.current_routine = None
        self.current_step_index = 0
        self.routine_paused = False
        self.step_start_time = None

        # Define movement routines
        self.routines = define_routines()

        self.get_logger().info("QubotWalk node with routines started")
        self.get_logger().info(f"Available routines: {list(self.routines.keys())}")
        self.get_logger().info(
            f"Basic movement settings: {self.publish_interval*1000:.0f}ms interval, {self.movement_duration}s duration"
        )

    def control_cmd_callback(self, msg):
        """
        Handle control commands: individual movements, routines, pause/resume/stop
        """
        command = msg.data.strip().lower()
        self.get_logger().info(f"Received control command: {command}")

        # Handle pause/resume/stop commands for routines
        if command == "pause" or command == "safety_pause":
            if self.current_routine is not None:
                self.routine_paused = True

                if command == "pause":
                    self.get_logger().info("Routine paused")
                    # Send stop command while paused
                    stop_twist = Twist()
                    self.cmd_vel_pub.publish(stop_twist)
                elif command == "safety_pause":
                    self.get_logger().info("Routine paused due safety")
            else:
                self.get_logger().warn("No routine running to pause")
            return

        elif command == "resume":
            if self.current_routine is not None and self.routine_paused:
                self.routine_paused = False
                self.step_start_time = self.get_clock().now()  # Reset step timer
                self.get_logger().info("Routine resumed")
            else:
                self.get_logger().warn("No paused routine to resume")
            return

        elif command == "stop":
            self.get_logger().info("Stopping all movement")
            self.stop_all_movement()
            return

        # Check if command is a routine
        if command in self.routines:
            self.start_routine(command)
            return

        # # Handle individual movement commands (original functionality)
        # self.handle_individual_command(command)

    def start_routine(self, routine_name):
        """
        Start executing a movement routine
        """
        self.get_logger().info(f"Starting routine: {routine_name}")

        # Stop any current movement
        self.stop_all_movement()

        # Initialize routine state
        self.current_routine = self.routines[routine_name]
        self.current_step_index = 0
        self.routine_paused = False

        # Start the first step
        self.execute_routine_step()

    def execute_routine_step(self):
        """
        Execute the current step of the routine
        """
        if self.current_routine is None or self.current_step_index >= len(
            self.current_routine
        ):
            self.get_logger().info("Routine completed")
            self.current_routine = None
            self.stop_all_movement()
            return

        # Get current step parameters
        linear_speed, angular_speed, duration = self.current_routine[
            self.current_step_index
        ]

        self.get_logger().info(
            f"Executing routine step {self.current_step_index + 1}/{len(self.current_routine)}: "
            f"linear={linear_speed}, angular={angular_speed}, duration={duration}s"
        )

        # Create twist message for this step
        self.current_twist = Twist()
        self.current_twist.linear.x = linear_speed
        self.current_twist.angular.z = angular_speed

        # Set timing
        self.step_start_time = self.get_clock().now()
        self.movement_duration = duration

        # Start publishing timer
        if self.movement_timer is not None:
            self.movement_timer.cancel()

        self.movement_timer = self.create_timer(
            self.publish_interval, self.routine_publish_callback
        )

        # Publish first message immediately
        self.cmd_vel_pub.publish(self.current_twist)

    def routine_publish_callback(self):
        """
        Timer callback for routine execution
        """
        if self.routine_paused:
            return  # Don't publish while paused

        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.step_start_time).nanoseconds / 1e9

        if elapsed_time >= self.movement_duration:
            # Current step completed, move to next step
            self.current_step_index += 1

            if self.movement_timer is not None:
                self.movement_timer.cancel()
                self.movement_timer = None

            # Execute next step or complete routine
            self.execute_routine_step()
        else:
            # Continue current step
            self.cmd_vel_pub.publish(self.current_twist)
            self.get_logger().debug(
                f"Routine step progress: {elapsed_time:.2f}/{self.movement_duration:.2f}s"
            )

    def stop_all_movement(self):
        """
        Stop all movement and clean up timers
        """
        # Cancel any active timer
        if self.movement_timer is not None:
            self.movement_timer.cancel()
            self.movement_timer = None

        # Reset routine state
        self.current_routine = None
        self.routine_paused = False

        # Send stop command
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_twist)

    def destroy_node(self):
        """
        Clean up when node is destroyed
        """
        self.stop_all_movement()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        qubot_walk = QubotWalk()
        rclpy.spin(qubot_walk)
    except KeyboardInterrupt:
        print("Shutting down QubotWalk node...")
    except Exception as e:
        print(f"Error in QubotWalk: {e}")
    finally:
        if "qubot_walk" in locals():
            qubot_walk.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
