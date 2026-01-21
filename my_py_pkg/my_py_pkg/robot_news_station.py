#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")
        #declarar parametros
        self.declare_parameter("robot_name","C3P0")
        #obtener valor y usarlo
        self.robot_name_ = self.get_parameter("robot_name").value
        self.publisher_ = self.create_publisher(String, "robot_news", 10) #datatype  , name , qeue size
        self.timer_ = self.create_timer(0.5, self.publish_news) #llama al metodo cada 1/2s seg
        self.get_logger().info("Robot news Station has been started..")

    def publish_news(self):
        #1 crear message
        msg = String()
        msg.data = "Hi, this is "+ self.robot_name_ +"  from the robot news station"
        #2 publicar
        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()