import arcade
import rclpy
import math
from threading import Thread
from rclpy.node import Node
from .lib.printer_sim import PrinterSimulation

from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Point

from ros2_project_interface.srv import ShapeService

class Shapeservice(Node):

    def __init__(self):
        super().__init__("Shape_service_node")
        
        self.srv = self.create_service(ShapeService, 'shape_service', self.calc_vertice)
        self.x_array=[]
        self.y_array=[]

    def calc_vertice(self, request, response):
        r=request.radius
        n=request.n

        for i in range(n+1):
            xi=r*math.cos((2*math.pi*i)/n)
            yi=r*math.sin((2*math.pi*i)/n)
            self.x_array.insert(i,xi)
            self.y_array.insert(i,yi)
        #self.get_logger().info('%f,%f' %(xi,yi))
        response.xver=self.x_array
        response.yver=self.y_array
        self.get_logger().info('Incoming request r: %d n: %d'  % (request.radius, request.n))

        return response

def main(args=None):
    rclpy.init(args=args)

    # Create node for simulation
    service_node = Shapeservice()
    
    # Spin indefinitely..
    rclpy.spin(service_node)

    # On shutdown...
    service_node.destroy_node()
    rclpy.shutdown()
    

# Script entry point
if __name__ == "__main__":
    main()
