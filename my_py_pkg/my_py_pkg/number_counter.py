#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter") 
        #contador variable global
        self.counter_ = 0
        #iniciar la suscripcion al topic
        self.suscriber_ = self.create_subscription(Int64, "number", self.callback_number_publisher,10)
        #iniciarl el publisher
        self.publisher_ = self.create_publisher(Int64, "number_count", 10) #datatype  , name , qeue size
        #self.timer_ = self.create_timer(0.5, self.publish_number_count) #llama al metodo cada 1/2s seg
        #logs
        self.get_logger().info("Number Counter node has been started")
        #creamos el server
        self.server_ = self.create_service(SetBool, "reset_counter", self.callback_reset_counter )

    def callback_number_publisher(self, msg: Int64):
        self.get_logger().info("Received: " + str(msg.data))
        self.counter_+=1
        #self.get_logger().info("Counter: " + str(self.counter_))
        #al recibir el dato del topic publicar
        new_msg = Int64()
        new_msg.data = self.counter_
        self.publisher_.publish(new_msg)

    def publish_number_count(self):
        #1 crear message
        msg = Int64()
        msg.data = self.counter_
        #2 publicar
        self.publisher_.publish(msg)

    def callback_reset_counter(self, request: SetBool.Request, response: SetBool.Response):
        request_ = request.data
        response.message = "received"
        response.success = False
        if(request_):
            self.counter_ = 0
            self.get_logger().info("received request TRUE")
            response.success = True
            response.message = "received True msg"
        else:
            self.get_logger().info("received request FALSE")
            response.success = True
            response.message = "received False msg"
        return response

 
def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()  
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()