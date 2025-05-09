#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
from functools import partial

from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from my_turtle_interfaces.msg import Turtle, TurtleArray
from my_turtle_interfaces.srv import CatchTurtle


class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")

        # Declare parameters
        self.declare_parameter('spawn_frequency', 1.5)  # Default spawn frequency

        # Get parameter
        self.spawn_frequency_ = self.get_parameter('spawn_frequency').get_parameter_value().double_value

        # State variables
        self.alive_turtles_ = []

        # ROS 2 Interfaces
        self.alive_turtles_publisher_ = self.create_publisher(TurtleArray, "alive_turtles", 10)
        self.spawn_client_ = self.create_client(Spawn, "/spawn")
        self.catch_turtle_service_ = self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle)
        self.kill_turtle_client_ = self.create_client(Kill, "/kill")

        # Timer for spawning turtles based on the spawn frequency parameter
        self.create_timer(self.spawn_frequency_, self.call_spawn_turtle)

    # --- Service Handlers ---

    def callback_catch_turtle(self, request: CatchTurtle.Request, response: CatchTurtle.Response):
        """Handles the request to catch a turtle and kill it."""
        self.call_kill_turtle_service(request.turtle_to_catch)
        response.message = f"Killing turtle: {request.turtle_to_catch.name}"
        return response

    def publish_alive_turtles(self):
        """Publishes the list of alive turtles."""
        msg = TurtleArray()
        msg._turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)

    # --- Turtle Spawning ---

    def call_spawn_turtle(self):
        """Calls the service to spawn a new turtle with random position."""
        # Wait for the service to be available
        while not self.spawn_client_.wait_for_service(1):
            self.get_logger().warn("Waiting for service...")

        # Generate random pose for the new turtle
        random_pose = self.generate_random_target_pose()

        request = Spawn.Request()
        request.x = random_pose.x
        request.y = random_pose.y
        request.theta = random_pose.theta

        # Call the spawn service asynchronously
        future = self.spawn_client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawn_turtle, request=request))

    def callback_call_spawn_turtle(self, future, request: Spawn.Request):
        """Handles the callback after the turtle is spawned."""
        response: Spawn.Response = future.result()

        if response.name != "":
            self.get_logger().info("New alive turtle name: " + response.name)
            new_turtle = Turtle()
            new_turtle.x = request.x
            new_turtle.y = request.y
            new_turtle.theta = request.theta
            new_turtle.name = response.name
            self.alive_turtles_.append(new_turtle)
            self.publish_alive_turtles()
        else:
            self.get_logger().info("New turtle not spawned!")

    # --- Kill Turtle ---

    def call_kill_turtle_service(self, turtle: Turtle):
        """Kills the specified turtle by calling the /kill service."""
        while not self.kill_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for kill service...")

        request = Kill.Request()
        request.name = turtle.name

        future = self.kill_turtle_client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill_turtle_service, request=request))

    def callback_call_kill_turtle_service(self, future, request: Kill.Request):
        """Handles the callback after a turtle is killed."""
        for i, turtle in enumerate(self.alive_turtles_):
            if turtle.name == request.name:
                del self.alive_turtles_[i]
                break

        self.publish_alive_turtles()

    # --- Random Pose Generator ---

    def generate_random_target_pose(self):
        """Generates a random pose for spawning the turtle."""
        pose = Pose()
        pose.x = random.uniform(1.0, 10.0)
        pose.y = random.uniform(1.0, 10.0)
        pose.theta = random.uniform(0.0, 2 * 3.14159)
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
