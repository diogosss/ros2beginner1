#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from functools import partial

class BatteryNode(Node): 
    def __init__(self):
        super().__init__("led_battery_node_client")
        self.client_ = self.create_client(SetLed, "set_led")
        self.battery_state = "full"
        self.last_time_battery_state_changed = self.get_current_time_seconds()
        self.battery_timer_ = self.create_timer(0.1, self.check_battery_state)
        self.get_logger().info("BatteryNode has been started")


    def call_set_led(self, led_number, state):
        while not self.client_.wait_for_service(1.0):
           self.get_logger().warn("waiting for Led Panel Node server....")
       
        request = SetLed.Request()
        request.led_number = led_number
        request.state = state
        future = self.client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_led_panel_node, request=request))

    def callback_call_led_panel_node(self, future, request):
        response: SetLed.Response = future.result()
        self.get_logger().info("Get Response: "+ str(response.success))
        if response.success:
            self.get_logger().info("Led Turned On")
        else:
            self.get_logger().info("Led Not Changed")

    
    def get_current_time_seconds(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds / 1000000000.0
    
    def check_battery_state(self):
        time_now = self.get_current_time_seconds()
        if self.battery_state == "full" :
            if time_now - self.last_time_battery_state_changed > 4.0:
                self.battery_state = "empty"
                self.call_set_led(1,True)
                self.get_logger().info("Battery is empty charging ..")
                self.last_time_battery_state_changed = time_now
        elif self.battery_state == "empty":
            if time_now - self.last_time_battery_state_changed > 6.0:
                self.battery_state = "full"
                self.call_set_led(3,True)
                self.get_logger().info("Battery now is Full ..")
                self.last_time_battery_state_changed = time_now


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()