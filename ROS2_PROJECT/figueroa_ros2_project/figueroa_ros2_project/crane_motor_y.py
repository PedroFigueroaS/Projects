
import arcade
import rclpy 
import numpy as np
from enum import Enum
from threading import Thread
from rclpy.node import Node
import time
#from .lib.crane_sim import CraneSimulation

from std_msgs.msg import Float64, Int64, Bool, Empty
from geometry_msgs.msg import Point
from ros2_project_interface.srv import EndEffectorPosition

class motor_y_control(Node):
    def __init__(self):
        super().__init__("control_motorx_node")
        self.yinit=0.0
        self.threshold=1
        self.ts=0.01
        self.my_pose = self.create_publisher(Float64, "/motor_y", 10)
        self.my_state = self.create_publisher(Bool, "/ack_y", 10)
        self.pose_sub = self.create_subscription(Point, "/controller_setpoint", self.goal_received, 10)
        self.kp=1.3
        #CLIENT
        self.cli = self.create_client(EndEffectorPosition, 'end_effector_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = EndEffectorPosition.Request()
        self.y_new_position()
    
    def goal_received(self, msg: Point):
        #self.get_logger().info("init point X: ({0})".format(self.xnew))
        self.goal = msg
        #self.get_logger().info("ENTRE AQUI")
        self.control_loop = self.create_timer(self.ts, self.on_control_loop)

    def send_request(self):
        self.get_logger().info("asking init point")
        self.req.empty=Empty()
        self.future = self.cli.call_async(self.req)
        #self.get_logger().info("RESPONDI AL SERVICIO")
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def y_new_position(self):
        self.response=EndEffectorPosition.Response()
        self.response = self.send_request()
        self.ynew=self.response.end_effector_position.y
        self.get_logger().info("init point Y: ({0})".format(self.ynew))
        return self.yinit
       
    
    def on_control_loop(self):
        # compute distance error
        ypub=Float64()
        state=Bool()
        state.data=False
        e_y=self.goal.y-self.ynew
        vely=self.kp*e_y
        if vely >= 100:
            vely=100
        elif vely<= -100:
            vely=-100
        self.ynew=self.ynew+vely*self.ts
        self.get_logger().info("posy: {0}".format(self.ynew))
        ypub.data=self.ynew
        self.my_pose.publish(ypub)
        if abs(e_y) <= self.threshold:
            # cancel timer
            state.data=True
            self.my_state.publish(state)
            self.control_loop.cancel() 
            self.get_logger().info("Goal Y Reached!")
            return
        self.my_state.publish(state)

def main():
    rclpy.init(args = None)

    motor_y_c = motor_y_control()
    rclpy.spin(motor_y_c)

    motor_y_c.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
