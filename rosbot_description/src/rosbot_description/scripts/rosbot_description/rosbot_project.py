#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math
import time


class RosbotController:
    def __init__(self):

        self.active_ = False
        self.state_description= None
        self.pub_ = None
        self.regions_ = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
        self.state_ = 0
        self.state_dict_ = {
            0: 'find the wall',
            1: 'turn left',
            2: 'follow the wall',
            3: 'turn right',
        }
        self.kp1=1.5
        self.kp2=1.1
        self.vel_pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_sub=rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        self.vel_twist=Twist()
        self.rate = rospy.Rate(1000)
        
        self.timer=rospy.Timer(rospy.Duration(1),self.control_loop)

    def control_loop(self,timer):

        self.vel_twist.linear.x=0
        self.vel_twist.angular.z=-0.3
        self.vel_pub.publish(self.vel_twist)
        #if not self.active_:
        #    self.rate.sleep()
        #    #continue
        if self.state_ == 0:
            #print('find wall')
            msg = self.find_wall()
        elif self.state_ == 1:
            #print('turn left')
            msg = self.turn_left()
        elif self.state_ == 3:
                #print('turn left')
            msg = self.turn_right()
        elif self.state_ == 2:
            #print('follow wall')
            msg = self.follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        print(self.state_description)
        self.vel_pub.publish(msg)
        self.rate.sleep()
    
    
    def clbk_laser(self,msg):
        
        self.regions_ = {
            'right':   min(min(msg.ranges[540:610]), 12),
            'fright':  min(min(msg.ranges[611:682]), 12),
            'front':   min(min(min(msg.ranges[0:36]), min(msg.ranges[683:719])), 12),
            'fleft':   min(min(msg.ranges[37:108]), 12),
            'left':    min(min(msg.ranges[109:180]), 12),
    
        }
        self.take_action()


    def change_state(self,state):
        
        if state is not self.state_:
            #print ('Wall follower - [%s] - %s' % (state, self.state_dict_[state]))
            self.state_ = state
    #take_action()
        #print(regions_)

    def take_action(self):
        
        regions = self.regions_
        msg = Twist()
        linear_x = 0
        angular_z = 0
        self.state_description = ''

        d = 0.6#0.6 #REAL ROBOT 1
        d0= 0.5#0.5 # REAL ROBOT 1.5

        if regions['front'] > d0 and regions['fleft'] > d and regions['fright'] > d:
            self.state_description = 'case 1 - nothing'
            self.change_state(0)
            print('case 1')
        elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] > d: #fleft >d and fright > d
            self.state_description = 'case 2 - front'
            self.change_state(1)
            print('case 2')
        elif regions['front'] > d0 and regions['fleft'] > d and regions['fright'] < d:
            self.state_description = 'case 3 - fright'
            self.change_state(2)
            print('case 3')
        elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] > d:
            self.state_description = 'case 4 - fleft'
            self.change_state(0)
            print('case 4')
        elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] > d: #fleft >d and fright < d
            self.state_description = 'case 5 - front and fright'
            self.change_state(3)
            print('case 5')
        elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] < d: #fleft <d and fright > d
            self.state_description = 'case 6 - front and fleft'
            self.change_state(1)
            print('case 6')
        elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] < d: #fleft <d and fright < d
            self.state_description = 'case 7 - front and fleft and fright'
            self.change_state(1)
            print('case 7')
        elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] < d: 
            self.state_description = 'case 8 - fleft and fright'
            self.change_state(0)
            print('case 8')
        else:
            self.state_description = 'unknown case'
            rospy.loginfo(regions)


    def find_wall(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = -0.3
        return msg

    def turn_right(self):
        msg = Twist()
        msg.angular.z = -0.6
        msg.linear.x = 0.1
    #print(msg.angular.z)
        return msg
    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.6
        msg.linear.x = 0.075
        return msg

    def follow_the_wall(self):

        d_w=self.regions_['left']-self.regions_['right']
        w=self.kp1*d_w
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z=w
        #yd=0.45
        #y=regions_['right']
        return msg
def main():
    rospy.init_node("rosbot_controller")
    try:
        RosbotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("exception thrown")
        pass
main()