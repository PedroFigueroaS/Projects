import arcade
import rclpy 
import numpy as np
from enum import Enum
from threading import Thread
from rclpy.node import Node
#from .lib.crane_sim import CraneSimulation
import time

from std_msgs.msg import Float64, Int64
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from ros2_project_interface.srv import ShapeService
class Stage(Enum):
    PICK = 1
    LIFT = 2
    MOVE = 3
    PLACE = 4
    DROP = 5

class logic(Node):
    def __init__(self):
        super().__init__("logic_node")
        
        self.motorxstate=True
        self.motorystate=True
        self.nstage=0
        self.logic_pub = self.create_publisher(Int64, "/next_stage", 10)
        self.xstate=self.create_subscription(Bool, "/ack_x", self.xstate_callback, 10)
        self.ystate=self.create_subscription(Bool, "/ack_y", self.ystate_callback, 10)
        self.ver_pub= self.create_publisher(Point, "/next_waypoint", 10) 
        self.draw= self.create_publisher(Bool, "/draw", 10) 
        #time.sleep(2)
        #self.x_array=[]
        #self.y_array=[]
        
        self.apos=0
        self.radious=100
        self.n=8
        #self.x=100.0
        #self.y=100.0
        #lst.append(x) 

        ##service cliente
        self.cli = self.create_client(ShapeService, 'shape_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ShapeService.Request()
        self.generate_vertices()
        
        self.timer = self.create_timer(1, self.logic_callback)
        #self.new_init_position()
        #self.vertice_timer = self.create_timer(1, self.send_vertices)


    def send_request(self, r, n):
        self.req.radius= r
        self.req.n = n
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def generate_vertices(self):
        response = self.send_request(self.radious,self.n)
        self.x_array=response.xver
        self.y_array=response.yver
        #print(self.x_array[0])
        #print(self.y_array[0])
    def xstate_callback(self, msgx:Bool):
        self.motorxstate=msgx.data
        
    def ystate_callback(self, msgy:Bool):
        self.motorystate=msgy.data

    #def pub_vertices(self):

    #    for x in range(self.n+1):
    #        self.vertice=Point()
    #        response = self.send_request(self.radious,self.n,x)
    #        self.get_logger().info('xpos %f: for radious %d and %d vertices, vertice number: %d' %(response.xver, self.radious, self.n,x))
    #        self.get_logger().info('ypos %f: for radious %d and %d vertices, vertice number: %d' %(response.yver, self.radious, self.n,x))
    #        self.vertice.x=response.xver
    #        self.vertice.y=response.yver
    #        self.ver_pub.publish(self.vertice)
    #        time.sleep(1)
        #print(self.x_array)
        #print(self.x_array,self.y_array) 

    #def new_init_position(self):
    #    self.vertice=Point()
    #    response = self.send_request(self.radious,self.n,self.apos)
    #    self.get_logger().info('xpos %f: for radious %d and %d vertices, vertice number: %d' %(response.xver, self.radious, self.n,self.apos))
    #    self.get_logger().info('ypos %f: for radious %d and %d vertices, vertice number: %d' %(response.yver, self.radious, self.n,self.apos))
    #    self.vertice.x=response.xver
    #    self.vertice.y=response.yver
    #    self.ver_pub.publish(self.vertice)
    #    self.apos=self.apos+1
            
    def logic_callback(self):
        sdraw=Bool()
        sdraw.data=True
        self.vertice=Point()
        
        #print(self.x_array[self.apos])
        #print(self.y_array[self.apos])
        self.vertice.x=self.x_array[self.apos]
        self.vertice.y=self.y_array[self.apos]
        if self.motorxstate==True and self.motorystate==True:
            self.motorxstate=False
            self.motorystate=False
            if self.apos==0:
                sdraw.data=False
                self.draw.publish(sdraw)
            self.ver_pub.publish(self.vertice)
            self.apos=self.apos+1
            self.draw.publish(sdraw)
        
        if self.apos >self.n:
            return
    #    self.ver_pub.publish(self.vertice)
    #    self.x=self.x+10
    #    self.y=self.y+10

        #if self.motorxstate==True and self.motorystate==True:
        #    self.motorxstate=False
        #    self.motorystate=False
        #    self.new_init_position()
        #state=Int64()
        #target=Point()
        #state.data=1
        #self.logic_pub.publish(state)
        #target.x=2.0
        #target.y=2.0
        
        #self.logic_pub.publish(state)
        #self.set_point.publish(target)
        #state.data=3
        #self.logic_pub.publish(state)
    #def logic_callback(self):
    #    state=Int64()
    #    state.data=1
    #    self.logic_pub.publish(state)


def main(args=None):
    rclpy.init(args = args)
    robot_logic = logic()
    time.sleep(1)
    rclpy.spin(robot_logic)

    robot_logic.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
