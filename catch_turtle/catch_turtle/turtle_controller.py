#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import sqrt, sin, cos, atan2, pi
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_turtle_interfaces.msg import Turtle, TurtleArray
from my_turtle_interfaces.srv import CatchTurtle


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        # Declare parameters
        self.declare_parameter('linear_velocity_factor', 1.5)  # Default: 1.5
        self.declare_parameter('angular_velocity_factor', 4.0)  # Default: 4.0
        self.declare_parameter('timer_frequency', 0.02)  # Default: 0.02

        # Get parameters
        self.linear_velocity_factor_ = self.get_parameter('linear_velocity_factor').get_parameter_value().double_value
        self.angular_velocity_factor_ = self.get_parameter('angular_velocity_factor').get_parameter_value().double_value
        self.timer_frequency_ = self.get_parameter('timer_frequency').get_parameter_value().double_value

        # State variables
        self.current_pose_ = None  # Current pose of the turtle (initialized to None)
        self.target_turtle_ = None  # Target turtle (initialized to None)

        # ROS 2 Interfaces
        self.turtle_pose_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose, 10)
        self.alive_turtles_sub_ = self.create_subscription(TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.catch_turtle_client_ = self.create_client(CatchTurtle, "catch_turtle")

        # Timer (control loop with adjustable frequency)
        self.timer_ = self.create_timer(self.timer_frequency_, self.control_loop)

        self.get_logger().info("Turtle controller node started...")

    # --- Callback Functions ---
    
    def callback_pose(self, pose: Pose):
        """Stores the current position and orientation of the turtle."""
        self.current_pose_ = pose

    def callback_alive_turtles(self, turtle_list: TurtleArray):
        """Selects the closest turtle from the list of alive turtles as target."""
        if self.current_pose_ is None or not turtle_list.turtles:
            return

        # Calculate distances to each turtle
        turtle_distances = [
            self.calculate_distance(turtle.x, turtle.y, self.current_pose_.x, self.current_pose_.y)
            for turtle in turtle_list.turtles
        ]

        # Choose the closest turtle
        min_index = turtle_distances.index(min(turtle_distances))
        self.target_turtle_ = turtle_list.turtles[min_index]

    # --- Control Loop ---

    def control_loop(self):
        """Main movement logic that guides the turtle to the target."""
        if self.current_pose_ is None or self.target_turtle_ is None:
            return

        # Compute distance and angle to target
        dx = self.target_turtle_.x - self.current_pose_.x
        dy = self.target_turtle_.y - self.current_pose_.y
        distance = sqrt(dx ** 2 + dy ** 2)

        cmd = Twist()

        if distance > 0.5:
            # Compute minimal angular difference
            angle_to_target = atan2(dy, dx)
            angle_diff = atan2(
                sin(angle_to_target - self.current_pose_.theta),
                cos(angle_to_target - self.current_pose_.theta)
            )

            # Proportional control
            cmd.linear.x = self.linear_velocity_factor_ * distance
            cmd.angular.z = self.angular_velocity_factor_ * angle_diff
            self.cmd_vel_pub_.publish(cmd)

            self.get_logger().info(f"Distance to target: {distance:.2f}")
        else:
            # Stop and trigger catching service
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub_.publish(cmd)

            self.catch_target_turtle()
            self.get_logger().info("Target reached")

    # --- Service Client ---

    def catch_target_turtle(self):
        """Sends a service request to catch the current target turtle."""
        request = CatchTurtle.Request()
        request.turtle_to_catch = self.target_turtle_

        future = self.catch_turtle_client_.call_async(request=request)
        future.add_done_callback(self.callback_catch_target_turtle)

    def callback_catch_target_turtle(self, future):
        """Handles the response from the catch_turtle service."""
        response: CatchTurtle.Response = future.result()
        self.target_turtle_ = None  # Reset after catching
        self.get_logger().info(response.message)

    # --- Utility Function ---

    @staticmethod
    def calculate_distance(target_x, target_y, current_x, current_y):
        """Returns Euclidean distance between current and target positions."""
        dx = target_x - current_x
        dy = target_y - current_y
        return sqrt(dx ** 2 + dy ** 2)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
