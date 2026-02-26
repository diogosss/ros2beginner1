#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup #
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup #

class Node1(Node):
    def __init__(self):
        super().__init__("node1")
        
        #1 ¿Timer 1 paralelo a Timer 2?->SI
        # 	¿Timer 1 paralelo a sí mismo?->Sí (Potencialmente peligroso)
        # 	¿Uso de CPU/Hilos?->Muy alto
        # self.cb_group_ = ReentrantCallbackGroup()
        # self.timer1_ = self.create_timer(1.0, self.callback_timer1, callback_group=self.cb_group_)
        # self.timer2_ = self.create_timer(1.0, self.callback_timer2, callback_group=self.cb_group_)
        # self.timer3_ = self.create_timer(1.0, self.callback_timer3, callback_group=self.cb_group_)

        #3 ¿Timer 1 paralelo a Timer 2?->SI	
        # ¿Timer 1 paralelo a sí mismo?-> Timer 1: Sí / Timer 2 y 3: No	
        # ¿Uso de CPU/Hilos?->Medio
        # self.cb_group1_ = ReentrantCallbackGroup()
        # self.cb_group2_ = MutuallyExclusiveCallbackGroup()
        # self.timer1_ = self.create_timer(1.0, self.callback_timer1, callback_group=self.cb_group1_)
        # self.timer2_ = self.create_timer(1.0, self.callback_timer2, callback_group=self.cb_group2_)
        # self.timer3_ = self.create_timer(1.0, self.callback_timer3, callback_group=self.cb_group2_)

        #3 ¿Timer 1 paralelo a Timer 2?->SI	
        # ¿Timer 1 paralelo a sí mismo?->No (Seguro)	
        # ¿Uso de CPU/Hilos?->Equilibrado
        self.cb_group1_ = MutuallyExclusiveCallbackGroup()
        self.cb_group2_ = MutuallyExclusiveCallbackGroup()        
        #self.cb_group3_ = MutuallyExclusiveCallbackGroup()
        self.timer1_ = self.create_timer(1.0, self.callback_timer1, callback_group=self.cb_group1_)
        self.timer2_ = self.create_timer(1.0, self.callback_timer2, callback_group=self.cb_group2_)
        self.timer3_ = self.create_timer(1.0, self.callback_timer3, callback_group=self.cb_group2_)

    def callback_timer1(self):
        time.sleep(2.0)
        self.get_logger().info("cb 1")

    def callback_timer2(self):
        time.sleep(2.0)
        self.get_logger().info("cb 2")

    def callback_timer3(self):
        time.sleep(2.0)
        self.get_logger().info("cb 3")

# NODO 2
class Node2(Node):
    def __init__(self):
        super().__init__("node2")
        self.cb_group4_ = ReentrantCallbackGroup()
        self.timer4_ = self.create_timer(1.0, self.callback_timer4, callback_group=self.cb_group4_ )
        self.timer5_ = self.create_timer(1.0, self.callback_timer5, callback_group=self.cb_group4_ )

    def callback_timer4(self):
        time.sleep(2.0)
        self.get_logger().info("cb 4")

    def callback_timer5(self):
        time.sleep(2.0)
        self.get_logger().info("cb 5")


def main(args=None):
    rclpy.init(args=args)
    
    node1 = Node1()
    node2 = Node2()
    #1
    #rclpy.spin(node1)
    
    #2

    #3 
    executor = MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)
    executor.spin()   
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# 1
# En este caso, no hay restricciones. Un grupo reentrante permite que sus callbacks se ejecuten en paralelo entre sí,
# e incluso que múltiples instancias del mismo callback se ejecuten simultáneamente si el anterior no ha terminado.

#2
# Aquí divides las reglas de juego. timer1 es libre, pero timer2 y timer3 comparten una restricción.

#3
# Este es el diseño más común cuando quieres paralelismo pero quieres proteger cada función de sí misma (que una función no se pise a sí misma).