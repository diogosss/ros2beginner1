#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from my_robot_interfaces.msg import LedPanelState

class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel_node_server")
        self.server_ = self.create_service(SetLed, "set_led", self.callback_led_panel)
        self.publisher_ = self.create_publisher(LedPanelState, "led_panel_state", 10)
        self.get_logger().info("Set Led Servaer has been started...")

    def callback_led_panel(self, request: SetLed.Request, response: SetLed.Response):
        led_number = request.led_number
        state = request.state
        #Verificar datos recibidos sean correctos
        # if led_number >= len(led_number) or led_number < 0:
        #     response.success = False
        #     return response
        # if state not in [0,1]:
        #     response.success = False
        #     return response

        self.get_logger().info("Led number: " + str(request.led_number)+ " -> state: " + str(request.state))
        self.publish_panel_state(request.led_number, request.state)
        return response
    
    def publish_panel_state(self, ledNumber, stateLed):
        #self.get_logger().info("publish_panel_state Metho call")
        msg = LedPanelState()
        msg.message = self.obtener_mensaje(self.obtener_option(ledNumber,stateLed))
        self.publisher_.publish(msg)

    def obtener_mensaje(self, option):
        panel_led = {
            0:"(X)(X)(X)",
            1:"(0)(X)(X)",
            2:"(0)(0)(X)",
            3:"(0)(0)(0)",
        }
        return panel_led.get(option, panel_led)
    
    def obtener_option(self, ledNumber, stateLed):
        if(ledNumber==0 and not stateLed): return 0
        if(ledNumber==1 and stateLed): return 1
        if(ledNumber==2 and stateLed): return 2
        if(ledNumber==3 and stateLed): return 3
         


def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()