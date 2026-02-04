#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.goal_handle_:ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.count_until_server_ = ActionServer(
            self,
            CountUntil,
            "count_until",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback= self.execute_callback,
            callback_group=ReentrantCallbackGroup() )
        
        self.get_logger().info("Action server has been started")

    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("Received the goal ")
        #TODO: Buscar info threading
        with self.goal_lock_:
            #Policy: eRefuse new goal if current goal is still active
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().warn("A goal is already active, rejecting new goal..")
                return GoalResponse.REJECT

        #Validate the goal request
        if goal_request.target_number <= 0:
            self.get_logger().warn("the goal was Rejected... ")
            return GoalResponse.REJECT
        self.get_logger().info("Acepting the goal... ")
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        #obtener en el hilo qye se necesita consultar TODO: Buscar info threading
        with self.goal_lock_:
            self.goal_handle_ = goal_handle
        # Get request from goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period
        #Execute the action
        self.get_logger().info("Executing the goal")
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        counter = 0
        for i in range(target_number):
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Canceling the goal...")
                goal_handle.canceled()
                result.reached_number = counter
                return result
            counter += 1
            self.get_logger().info(str(counter))
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

        # Once done, set goal final state
        goal_handle.succeed()
        #goal_handle.abort()

        # and send the result        
        result.reached_number = counter
        return result
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().warn("RECEIVED CANCEL REQUEST")
        return CancelResponse.ACCEPT # or REJECT


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == "__main__":
    main()