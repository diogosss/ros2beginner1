#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from example_interfaces.msg import Int64

class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("number_publisher")
        self.get_logger().info("In Constructor")
        self.number_ = 1
        self.publish_frequency_ = 1.0
        self.number_publisher_ = None
        self.number_timer_ = None


    #Aqui se puede: ros2 communication, suscribers , connect to hardware etc
    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_configure.")
        self.number_publisher_ = self.create_lifecycle_publisher(Int64, "number", 10) # no se activa, no publica hasta estar en estado Activo el lifecycle node
        self.number_timer_ = self.create_timer(
            1.0 / self.publish_frequency_, self.publish_number)        
        return TransitionCallbackReturn.SUCCESS  # or FAILURE
    
    # Aqui se limpia para regresar al estado Unconfigured, destroy ROS comunicatiosn from hardware
    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_cleanup.")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS  # or FAILURE
    
    # Activar el estado del nodo example Hardwarea
    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_activate.")
        return super().on_activate(previous_state) # activa todo el nodo
    
    # Desactivar el estado del nodo 
    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_deactivate.")
        return super().on_deactivate(previous_state) # desactiva todo el nodo
    

    # Shutdown 
    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_shutdown.")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS  # or FAILURE
    

    

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)
        self.number_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
