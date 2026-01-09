#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from functools import partial
import random
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.srv import CatchTurtle

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner_node")
        self.alive_turtles_pub_ = self.create_publisher(TurtleArray, "alive_turtles", 10)
        #cliente para spawnear turtles
        self.client_ = self.create_client(Spawn, "spawn")
        #cliente para kill turtles
        self.client_kill_ = self.create_client(Kill, "kill")
        self.timer_ = self.create_timer(3.0, self.spawn_turtle)
        #crear array
        self.alive_turtles = TurtleArray()
        #server para atrapar las turtles y borrarlas matarlas
        self.server_ = self.create_service(CatchTurtle, "catched_turtle", self.callback_catched_turtle)
        self.get_logger().info("Turtle Spawner Node has been started ...")

    def publish_alive_turtles(self):
        msg = self.alive_turtles
        self.get_logger().info("Publish array turtles.. ")
        self.alive_turtles_pub_.publish(msg)

    def spawn_turtle(self):
        while not self.client_.wait_for_service(1.0):
           self.get_logger().warn("waiting for turtleSim Node server....")
        
        request = Spawn.Request()
        request.x = self.get_random_value()
        request.y = self.get_random_value()
        request.theta = 2.0        
        future = self.client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_spanw, request=request))
    
    def callback_call_spanw(self, future, request):
        response = future.result()
        self.get_logger().info("Get Response: "+ str(response.name))
        #update array con el nombre dado por la respuesta
        self.add_turtle(request.x, request.y, request.theta, response.name)
        self.publish_alive_turtles()

    def get_random_value(self):
        return random.uniform(0.0, 11.0)
    
    def add_turtle(self, x, y , theta, name):
        turtle = Turtle()
        turtle.name = name
        turtle.x = x
        turtle.y = y
        turtle.theta = theta
        self.alive_turtles.turtles.append(turtle)

    def delete_turtle(self, name):
        self.get_logger().info("delete_turtle")
        self.alive_turtles.turtles = [turtle for turtle in self.alive_turtles.turtles if turtle.name != name]

    def kill_turtle(self, name):
        while not self.client_kill_.wait_for_service(1.0):
           self.get_logger().warn("waiting for turtleSim Node server....")
        request = Kill.Request()
        request.name = name      
        future = self.client_kill_.call_async(request)
        future.add_done_callback(partial(self.callback_killed_turtle, request=request))
        self.get_logger().info("Turtle " + str(name) +" killed")

    def callback_catched_turtle(self, request: CatchTurtle.Request, response: CatchTurtle.Response):
        #request.name
        self.delete_turtle(request.name)
        self.kill_turtle(request.name)
        self.get_logger().info("Deleted turtle: "+ str(request.name))
        response.success = True
        return response
    
    def callback_killed_turtle(self, future, request):
        response = future.result()
        self.get_logger().info("callback_killed_turtle" + str(response))

    



def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()