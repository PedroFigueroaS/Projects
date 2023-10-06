
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

class motor_x_control(Node):
    def __init__(self):
        super().__init__("control_motorx_node")
        self.xinit=0.0
        self.threshold=1
        self.ts=0.01
        self.mx_pose = self.create_publisher(Float64, "/motor_x", 10)
        self.mx_state = self.create_publisher(Bool, "/ack_x", 10)
        self.pose_sub = self.create_subscription(Point, "/controller_setpoint", self.goal_received, 10)
        self.kp=1.3
        #CLIENT
        self.cli = self.create_client(EndEffectorPosition, "end_effector_position")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = EndEffectorPosition.Request()
        #time.sleep(1)
        self.x_new_position()
        
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
    
    def x_new_position(self):
        #self.get_logger().info("Setpoint X: ({0})".format(msg.x))
        #self.get_logger().info("ENTRE AL SERVICIO")
        self.response=EndEffectorPosition.Response()
        self.response = self.send_request()
        self.xnew=self.response.end_effector_position.x
        self.get_logger().info("init point X: ({0})".format(self.xnew))
        #self.get_logger().info("ME RESPONDIO EL SERVICIO")


    def on_control_loop(self):
        # compute distance error
        #self.get_logger().info('ENTRASTE AL LOOP')
        state=Bool()
        #vx=Num()
        state.data=False
        xpub=Float64()
        e_x=self.goal.x-self.xnew
        velx=self.kp*e_x
        #vx.vx=velx
        #self.velx_pub.publish(vx)
        if velx >= 100:
            velx=100
        elif velx<= -100:
            velx=-100
        self.xnew=self.xnew+velx*self.ts
        #self.get_logger().info('NUEVA POSICION')
        #self.get_logger().info("posx: {0}".format(self.xnew))
        xpub.data=self.xnew
        self.mx_pose.publish(xpub)
        #self.get_logger().info('ESTAS AQUI')
        if abs(e_x) <= self.threshold:
            # cancel timer
            #self.get_logger().info('LLEGASTE A LA CONDICION')
            state.data=True
            self.mx_state.publish(state)
            self.control_loop.cancel() 
            #self.get_logger().info("Goal X Reached!")
            
            #self.get_logger().info('HACES RETURN')
            return
        self.mx_state.publish(state)

def main():
    rclpy.init(args = None)
    motor_x_c = motor_x_control()
    rclpy.spin(motor_x_c)

    motor_x_c.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
