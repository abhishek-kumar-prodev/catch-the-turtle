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

        self.current_pose_: Pose = None
        self.target_turtle_:Turtle = None

        self.turtle_pose_sub_ = self.create_subscription(
            Pose, "/turtle1/pose", self.callback_pose, 10)
        self.alive_turtles_sub_ = self.create_subscription(
            TurtleArray, "alive_turtles",self.callback_alive_turtles, 10)
        self.cmd_vel_pub_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        self.catch_turtle_client_ = self.create_client(CatchTurtle, "catch_turtle")
        self.timer_ = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("Turtle controller node started...")

    def callback_alive_turtles(self, turtle_list:TurtleArray):
        if turtle_list.turtles:  
            turtle_distances = []
            for turtle in turtle_list.turtles:
                dist = self.calculate_distance(turtle.x, turtle.y, self.current_pose_.x, self.current_pose_.y)
                turtle_distances.append(dist)

            min_index = turtle_distances.index(min(turtle_distances))
            self.target_turtle_ = turtle_list.turtles[min_index]


    @staticmethod
    def calculate_distance(target_x,target_y,current_x,current_y):
        dx = target_x - current_x
        dy = target_y - current_y
        distance = sqrt(dx ** 2 + dy ** 2)
        return distance

    def callback_pose(self, pose:Pose):
        self.current_pose_ = pose

    def control_loop(self):
        if self.current_pose_ == None or self.target_turtle_ == None:
            return

        dx = self.target_turtle_.x - self.current_pose_.x
        dy = self.target_turtle_.y - self.current_pose_.y
        distance = sqrt(dx ** 2 + dy ** 2)

        cmd = Twist()

        if distance > 0.5:
            angle_to_target = atan2(dy, dx)
            angle_diff = atan2(sin(angle_to_target - self.current_pose_.theta), 
                   cos(angle_to_target - self.current_pose_.theta))

            cmd.linear.x = 1.5 * distance
            cmd.angular.z = 4.0 * angle_diff
            self.cmd_vel_pub_.publish(cmd)
            self.get_logger().info(f"Distance to target: {distance:.2f}")
        else:
            cmd.linear.x = 0
            cmd.angular.z = 0
            self.cmd_vel_pub_.publish(cmd)
            self.catch_target_turtle()
            self.get_logger().info("Target reached")

    def catch_target_turtle(self):
        request = CatchTurtle.Request()
        request.turtle_to_catch =  self.target_turtle_ 

        future = self.catch_turtle_client_.call_async(request=request)
        future.add_done_callback(self.callback_catch_target_turtle)

    def callback_catch_target_turtle(self, future):
        response:CatchTurtle.Response = future.result()
        self.target_turtle_ = None  
        self.get_logger().info(response.message)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
