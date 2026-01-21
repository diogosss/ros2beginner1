#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from functools import partial

class BatteryNode(Node): 
    def __init__(self):
        super().__init__("led_battery_node_client")
        self.client_ = self.create_client(SetLed, "set_led")
        self.timer_ = self.create_timer(7, self.call_led_panel_node)
        self.current_number = 0
        self.current_state = False
        self.call_count = 0
        self.get_logger().info("NumberPublisher has been started")


    def call_led_panel_node(self):
        while not self.client_.wait_for_service(1.0):
           self.get_logger().warn("waiting for Led Panel Node server....")

        self.callPanel()
        
        request = SetLed.Request()
        request.led_number = self.current_number
        request.state = self.current_state
        self.get_logger().info("request led_number->"+ str(self.current_number)+" state " +str(self.current_state))

        future = self.client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_led_panel_node, request=request))

    def callback_call_led_panel_node(self, future, request):
        response = future.result()
        self.get_logger().info("Get Response: "+ str(response.success))

    def callPanel(self):
        if(self.call_count == 0):
            self.current_number = 0
            self.current_state = False
        elif(self.call_count == 1):
            self.current_number = 1
            self.current_state = True
        elif(self.call_count == 2):
            self.current_number = 2
            self.current_state = True
        elif(self.call_count == 3):
            self.current_number = 3
            self.current_state = True
        self.call_count = (self.call_count + 1)%5


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()