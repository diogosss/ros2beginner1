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
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller_node")

        #iniciar la suscripcion al topic recibir lista de turtles
        self.suscriber_ = self.create_subscription(TurtleArray, "alive_turtles", self.callback_alive_turtles,10)
        #publicar al topic de controll de la tortuga master
        self.velocity_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        #suscribir al topic de posici[on de la tortuga master
        self.pose_sub_ = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        self.pose = Pose()

        #cliente para nodo spawn para matar tortugas
        self.client_kill_ = self.create_client(CatchTurtle, "catched_turtle")

        #tutrle goal
        self.goal_x = 8.0
        self.goal_y = 8.0

        #array de alive turtles
        self.waypoints = []
        self.current_goal = None

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Turtle Controller Node has been started ...")


    def callback_alive_turtles(self, msg: TurtleArray):
        #self.get_logger().info("Received: " + str(msg.turtles))
        self.get_logger().info("[0]: " + str(msg.turtles[0]))
        #self.publish_catch_turtle(msg.turtles[0])
        self.waypoints = msg.turtles
        self.get_logger().info("Se recibe nueva lista con {len(self.waypoints)} tortugas")

        #si no nos movemos empezamos con el primero
        # if self.current_goal is None and len(self.waypoints)>0:
        #     self.current_goal = self.waypoints[0]
        if len(self.waypoints)>0:
            self.select_nearest_waypoint()


    def control_loop(self):
        if self.current_goal is None:
            return
        
        distancia = self.get_distance(self.current_goal.x, self.current_goal.y)
        msg = Twist()
        self.get_logger().info("distancia: " + str(distancia))
        if distancia > 0.1:
            msg.linear.x = 1.5 * distancia #velocidad lineal proporcional a la distancia
            #Velocidad angular orientarse hacia el objetivo
            angle_to_goal = math.atan2(self.current_goal.y - self.pose.y, self.current_goal.x - self.pose.x)
            error_angular = angle_to_goal -self.pose.theta
            while error_angular>math.pi:
                error_angular -= 2.0 * math.pi
            while error_angular < -math.pi:
                error_angular += 2.0 * math.pi
            
            msg.angular.z = 4.0 * error_angular
            self.publish_velocity_turtle(msg)

        else:
            msg.linear.x=0.0
            msg.angular.z=0.0
            self.publish_velocity_turtle(msg)
            self.kill_turtle(self.current_goal.name)
            self.current_goal = None


    def publish_velocity_turtle(self, msg:Twist):     

        self.get_logger().info("Publish velocity control turtle.. ")
        self.velocity_pub_.publish(msg)
        

    
    #actualizar posicion tortuga master
    def update_pose(self, data):
        self.pose = data

    def get_distance(self, goal_x, goal_y):
        return math.sqrt(math.pow((goal_x - self.pose.x),2) + math.pow((goal_y -self.pose.y),2))
    

    def kill_turtle(self, name):
        while not self.client_kill_.wait_for_service(1.0):
           self.get_logger().warn("waiting for turtle Spawn Node server....")
        request = CatchTurtle.Request()
        request.name = name      
        future = self.client_kill_.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill_response, request=request))

    def callback_call_kill_response(self, future, request):
        response = future.result()
        self.get_logger().info("Get Response: "+ str(response))
    

    def select_nearest_waypoint(self):
        best_distance = float('inf')
        best_waypoint = None
        index_to_remove = -1

        for i, wp in enumerate(self.waypoints):
            dist = math.sqrt(
                math.pow((wp.x - self.pose.x),2) +
                math.pow((wp.y - self.pose.y),2)
            )
            if dist < best_distance:
                best_distance = dist
                best_waypoint = wp
                index_to_remove = i
        if best_waypoint:
            self.current_goal = best_waypoint
            self.get_logger().info("Nuevo objeto cercano: ({self.current_goal.x},){self.current_goal.y}")

    




def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()