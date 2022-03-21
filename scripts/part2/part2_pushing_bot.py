#!/usr/bin/env python
import imutils as imutils

import nav_msgs
import rospy
import sys

# !/usr/bin/env python

import rospy
import dynamic_reconfigure.client
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.srv import GetMap

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base actioninst
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import cv2
from PIL import Image as im

from nav_msgs.srv import GetPlan

#!/usr/bin/env python
# Import ROS libraries and messages
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
# Import OpenCV libraries and tools
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
import argparse

#import imutils
import time
#import pandas as pd

import pickle
from geometry_msgs.msg import Twist, Pose , Point
from tf.transformations import euler_from_quaternion

from actionlib_msgs.msg import GoalStatus
# for cosstmap
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
# from tf import TransformListener

from std_msgs.msg import String
# define the lower and upper boundaries of the "blue"
# from tf.transformations import euler_from_quaternion

import navfn
#
# from part2_camera import Ball_Queue
BlueLower = (100, 150, 0)
BlueUpper = (140, 255, 255)

RedLower = (0, 50, 50)
RedUppder = (10, 255, 255)

PIXSELS_TO_BIG_BALL = 30
PIXSELS_TO_BIG_BALL_LOWER = 50
PIXSELS_TO_BIG_BALL_UPPER = 150

import math



class Ball_Queue:
    def __init__(self):
        self.ball_list = list()

    def pop(self):
        val = self.ball_list.pop(0)
        return val

    def insert(self, val):
        if not any(dist([x.ball.x, x.ball.y], [val.ball.x, val.ball.y]) < 0.2 for x in self.ball_list):
            self.ball_list.append(val)
        else:
            print("we didnt add to requests")

class PushBot:

    busy = False
    def __init__(self):

        self.ms = MapServicePart2()
        # inorder to move ( cant use movebase to hit balls)
        self.pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)
        self.yaw = 0

        self.order = rospy.Subscriber('assistance_request', String, self.perform_assistance)
        # transformer listen for position accurary
        #self.tf = TransformListener()
        # twist for cmd vel
        self.twist = Twist()
        self.rotation_time = None
        # for movebase actions (to use given movebase
        self.client = actionlib.SimpleActionClient('tb3_0/move_base', MoveBaseAction)
        self.goal = MoveBaseGoal()
        self.x = 0
        self.y = 0
        self.ballcounter = 0

        self.square_rotation_flag = False
        self.time_of_flag = None

        # RECTANGLES
        self.listOfFloorRects = []
        self.start_time = time.time()
        self.sub = rospy.Subscriber('/tb3_0/odom', Odometry, self.get_rotation) # now only gets locations
        # costmap to use for comparison with static map (check ball existence and if marked already)
        self.costmap_updater = CostmapUpdater()
        rospy.wait_for_service('/tb3_0/static_map')
        static_map = rospy.ServiceProxy('/tb3_0/static_map', GetMap)
        # static map arguments (we address the map_arr for info about pixel)
        self.map_data = static_map().map
        self.map_org = np.array([self.map_data.info.origin.position.x, self.map_data.info.origin.position.y])
        shape = self.map_data.info.height, self.map_data.info.width
        self.map_arr = np.array(self.map_data.data, dtype='float32').reshape(shape)
        self.resolution = self.map_data.info.resolution
        # camera to get image and distance variables from
        self.destination = None
        self.ball_location = None

        rospy.Subscriber('tb3_0/initialpose', PoseWithCovarianceStamped, self.initial_pose)
        self.initialpose = None
        # self.ms = MapService()

        self.pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)
        self.ballx = None
        self.bally = None
        self.goalx = None
        self.goaly = None
        # self.sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, self.trick)
        self.yaw = 0
        self.sub_scan = rospy.Subscriber('/tb3_0/scan', LaserScan, self.scan_callback)
        self.distance = 999
        self.pixel_counter = None
        self.front_dist = None
        self.request_sub = rospy.Subscriber('communication_channel',Point, self.request_callback)
        # we reached a point where we need to stop pushing so the ball wont pass the target goal //
        # OR push real slowly?
        self.queue = Ball_Queue()
    def request_callback(self,data):
        self.queue.insert(data)
        print("request recieved")
        while PushBot.busy:
            rospy.sleep(1)
            # and not PushBot.busy
        if self.queue.ball_list :
            PushBot.busy = True
            print("request handling")
            ball = self.queue.ball_list[0]
            self.ballx , self.bally = ball.x , ball.y
            self.goalx , self.goaly = self.get_push_direction(data.z)
            # print("GOOAAALLLLLL")
            # print(self.goalx,self.goaly)
            self.calculate_sphere_approach_direction()
            self.push_ball()
            self.queue.ball_list.pop(0)
            print("request finished")
            PushBot.busy=False
    # self.ballx = data.x
        # self.bally = data.y
        # self.goalx = 0
        # self.goaly = 0
        # self.calculate_sphere_approach_direction()
        # self.push_ball()

    def get_push_direction(self,val):
        zero_val = self.ms.map_to_position(np.array([0, 0]))
        if val == 1 :
            return zero_val[0] , zero_val[1]
        size_ver = self.map_arr.shape[0]
        size_hor = self.map_arr.shape[1]
        [size_hor , size_ver] = self.ms.map_to_position(np.array([size_ver, size_ver]))

        if val == 2 :
            return zero_val[0] , size_ver
        if val == 3 :
            return size_hor , zero_val[1]
        return size_hor,size_ver
    def scan_callback(self, msg):

        self.ranges = msg.ranges

    def get_rotation(self, data):
        # print("x valuye = "+str(data.pose.pose.position.x) + "  y value = " + str(data.pose.pose.position.y))
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        z = data.pose.pose.position.z
        y = self.y
        x = self.x
        orientation = data.pose.pose.orientation
        orientationList = [orientation.x, orientation.y, orientation.z, orientation.w]
        # print(" the value of yaw is :")
        # print(math.atan2(2.0*(orientation.y*orientation.z + orientation.w*orientation.x), orientation.w*orientation.w - orientation.x*orientation.x - orientation.y*orientation.y + orientation.z*orientation.z))
        (_, _, self.yaw) = euler_from_quaternion(orientationList)
        # doesnt work :(

    def quanterntion_from_euler(self,x,y,z):
        yawOver2 = x * 0.5
        cosYawOver2 = math.cos(yawOver2)
        sinYawOver2 = math.sin(yawOver2)
        pitchOver2 = y * 0.5
        cosPitchOver2 = math.cos(pitchOver2)
        sinPitchOver2 = math.sin(pitchOver2)
        rollOver2 = z * 0.5
        cosRollOver2 = math.cos(rollOver2)
        sinRollOver2 = math.sin(rollOver2)

        w = cosYawOver2 * cosPitchOver2 * cosRollOver2 + sinYawOver2 * sinPitchOver2 * sinRollOver2
        x = sinYawOver2 * cosPitchOver2 * cosRollOver2 + cosYawOver2 * sinPitchOver2 * sinRollOver2
        y = cosYawOver2 * sinPitchOver2 * cosRollOver2 - sinYawOver2 * cosPitchOver2 * sinRollOver2
        z = cosYawOver2 * cosPitchOver2 * sinRollOver2 - sinYawOver2 * sinPitchOver2 * cosRollOver2

        return w, x, y, z

    def initial_pose(self, msg):
        self.initialpose = msg.pose.pose

    #def trick(self, data):
    # if self.camera.see_circle and abs(self.camera.front_dist - self.camera.distance) < 0.15:
    #     self.to_circle()
    def perform_assistance(self,data):
        # data contains ball location and ball destination
        self.destination= data["destination"]
        self.ball_location = data["ball location"] # still needs to be implemented

    def MoveLoop(self):
        # if self.camera.see_circle and abs(self.camera.front_dist - self.camera.distance) < 0.15:
        while(True):
            if(self.destination is not None):   #communication order here

                pass

    # we work with map values
    def calculate_sphere_approach_direction(self):
        pass
        vector_x = self.goalx - self.ballx
        vector_y = self.goaly - self.bally
        required_orientation = math.atan2(vector_y, vector_x)
        vec_length = np.sqrt(pow(vector_x,2)+pow(vector_y,2))
        vector_x = vector_x/vec_length
        vector_y = vector_y/vec_length
        wanted_location_x = self.ballx - 0.5 * vector_x
        wanted_location_y = self.bally - 0.5 * vector_y
        # robot_x_offset = 0.04290
        robot_x_offset = 0
        arr = [float(""'%.5f'"" % round(wanted_location_x, 5))-robot_x_offset, float(""'%.5f'"" % round(wanted_location_y, 5))]
        required_orientation = float(""'%.5f'"" % round(required_orientation, 5))

        print( arr[0]-0.04290, arr[1], required_orientation)
        w ,x ,y ,z = self.quanterntion_from_euler(0,0,required_orientation)
        self.ms.movebase_client(0, arr[0], arr[1], w ,x ,y ,z )
        # print("end")



    def adjust_to_middle(self):
        pass


    def check_if_path_linear(self):
        pass
        # needs to be implemented if we want to handle a non linear path
        return True
        # or we can move responsibility to the camera bot, that will give the push-bot only linear push orders?

        # receieves the inut as point on the map? or on the arraymap?

    def check_ball_next_to_wall(self):
        rospy.sleep(1)
        center_x = self.x + np.cos(self.yaw) * (0.6)
        center_y = self.y + np.sin(self.yaw) * (0.6)
        index = self.ms.position_to_map(np.array([center_y, center_x]))
        x = index[0]
        y = index[1]
        x = int(x)
        y = int(y)
        if  self.map_arr[x][y] == -1 or self.map_arr[x][y] == 100:
            return True
        print(x,y)
        return False
    def push_ball(self):
        pass
        # boarder_x = self.x + 0.5 * math.cos(self.yaw)
        # boarder_y = self.y + 0.5 * math.sin(self.yaw)
        # check = self.costmap_updater.cost_map[boarder_x][boarder_y] > 65
        rospy.sleep(0.1)
        while(dist([self.x,self.y],[self.goalx,self.goaly]) > 0.5 and not self.check_ball_next_to_wall() ):
            # sanity_check = dist([self.x, self.y], [self.goalx, self.goaly])
            self.twist.linear.x = 0.2
            self.pub.publish(self.twist)
            print("we pushing")
            rospy.sleep(0.1)

            # if self.ranges[10] == self.ranges[0]/math.cos(np.pi/18) or self.ranges[350] == self.ranges[0]/math.cos(np.pi/18):
            #     print("WE BROKE")
            #     break
            # rospy.sleep(0.1)
            # if sanity_check == dist([self.x,self.y],[self.goalx,self.goaly]):
            #     break

        self.twist.linear.x = 0
        self.pub.publish(self.twist)


    ###
    ### FROM HERE ITS THE CAMERA
    ###

    def shrink(self, data,rows, cols):

        self.x_shrinkage = rows
        self.y_shrinkage = cols
        self.cols = rows = data.shape[0] / rows
        self.rows = cols = data.shape[1] / cols
        self.square_arr = data.reshape(rows, data.shape[0] / rows, cols, data.shape[1] / cols).sum(axis=1).sum(axis=2)

    def unshirnk(self, shrunk_x,shrunk_y):
        return shrunk_x*self.x_shrinkage+1 , shrunk_y*self.y_shrinkage+1

    def get_shrinked_pos(self, x,y):
        return int(x/self.x_shrinkage), int(y/self.y_shrinkage)
    # or already been traversed
    def is_available(self, x, y):
        if self.square_arr[x][y] != 0:
            return False
        real_x,real_y = self.unshirnk(x,y)
        if self.costmap_updater.cost_map[real_x][real_y] > 65:
            print("WE SAW BALL")
            self.square_arr[x][y]=333
            return False
        return True

    # will start with given location
    def traverse_loop(self, x , y):

        real_x, real_y = self.unshirnk(x,y)

        arr = self.ms.map_to_position(np.array([real_y,real_x]))
        # print(real_x,real_y,arr)
        self.ms.movebase_client(0, arr[0], arr[1])
        self.square_arr[x][y] = 666     # marked that it has already been visited
        if self.is_available(x+1,y):
            self.traverse_loop(x+1,y)
        if self.is_available(x-1,y):
            self.traverse_loop(x-1,y)
        if self.is_available(x,y+1):
            self.traverse_loop(x,y+1)
        if self.is_available(x,y-1):
            self.traverse_loop(x,y-1)

        # if we got here we are stuck (all neighboring positions already been traversed or blocked
        # we assume its already been traversed and not blocked.
    def preprocess(self, val):
        return 0 if val == 0 else 100

    def find_closest(self):
        for x in range(self.rows):
            for y in range(self.cols):
                if self.square_arr[x][y] < 100:
                    return x,y
        return 0,0

    def move_wrapper(self):
        self.square_arr = np.abs(np.sign(self.square_arr) * 100 )
        start_x , start_y = self.ms.position_to_map(np.array([self.x,self.y]))
        start_x , start_y = self.get_shrinked_pos(start_x,start_y)
        self.traverse_loop(start_x,start_y)
        while np.any(self.square_arr < 100 ) :
            x,y = self.find_closest()
            self.traverse_loop(x,y)

class MapServicePart2(object):

    def __init__(self):
        """
        Class constructor
        """
        rospy.wait_for_service('/tb3_0/static_map')
        static_map = rospy.ServiceProxy('/tb3_0/static_map', GetMap)
        self.map_data = static_map().map
        self.map_org = np.array([self.map_data.info.origin.position.x, self.map_data.info.origin.position.y])
        shape = self.map_data.info.height, self.map_data.info.width
        self.map_arr = np.array(self.map_data.data, dtype='float32').reshape(shape)

        self.sub_odom = rospy.Subscriber("odom", Odometry, self.callback)
        self.resolution = self.map_data.info.resolution
        self.first_run = True

        self.first_x = 0
        self.first_y = 0

    def callback(self, data):
        if (self.first_run):
            self.first_x = data.pose.pose.position.x
            self.first_y = data.pose.pose.position.y
            self.first_run = False

    def show_map(self, point=None):
        plt.imshow(self.map_arr)
        if point is not None:
            plt.scatter([point[0]], [point[1]])
        plt.show()

    def position_to_map(self, pos):
        return (pos - self.map_org) // self.resolution

    def map_to_position(self, indices):
        return indices * self.resolution + self.map_org

    def movebase_client(self,agent_id=0, x=1.0, y=0.5, w=1, orx=0, ory=0 , orz=0):

        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('tb3_%d/move_base' % agent_id, MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # Move 0.5 meters forward along the x axis of the "map" coordinate frame
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        # No rotation of the mobile base frame w.r.t. map frame
        goal.target_pose.pose.orientation.w = w
        goal.target_pose.pose.orientation.x = orx
        goal.target_pose.pose.orientation.y = ory
        goal.target_pose.pose.orientation.z = orz
        # Sends the goal to the action server.
        client.send_goal(goal)
        # Waits for the server to finish performing the action.
        wait = client.wait_for_result()

        # GetPlan - example of functionality

        start = PoseStamped()
        start.header.seq = 0
        start.header.frame_id = "map"
        start.header.stamp = rospy.Time(0)
        start.pose.position.x = goal.target_pose.pose.position.x
        start.pose.position.y = goal.target_pose.pose.position.y

        Goal = PoseStamped()
        Goal.header.seq = 0
        Goal.header.frame_id = "map"
        Goal.header.stamp = rospy.Time(0)
        Goal.pose.position.x = 1.0
        Goal.pose.position.y = 1.0

        get_plan = rospy.ServiceProxy('/tb3_%d/move_base/make_plan' % agent_id, nav_msgs.srv.GetPlan)
        req = nav_msgs.srv.GetPlan()
        req.start = start
        req.goal = Goal
        req.tolerance = .5
        resp = get_plan(req.start, req.goal, req.tolerance)
        rospy.loginfo(len(resp.plan.poses))
        # rospy.loginfo()

        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            return client.get_result()

    def move(self,agent_id=0, x=1.0, y=0.5, w=1.0):

        try:
            result = self.movebase_client(agent_id, x, y, w)
            if result:
                rospy.loginfo("Goal execution done!")
            else:
                rospy.loginfo("Goal unachievable!")
            return result
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")
class CostmapUpdater:
    def __init__(self):
        self.cost_map = None
        self.shape = None
        rospy.Subscriber('tb3_0/move_base/global_costmap/costmap', OccupancyGrid, self.init_costmap_callback)
        rospy.Subscriber('tb3_0/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.costmap_callback_update)

    def init_costmap_callback(self, msg):
        # print('only once')  # For the student to understand
        self.shape = msg.info.height, msg.info.width
        self.cost_map = np.array(msg.data).reshape(self.shape)

    def costmap_callback_update(self, msg):
        # print('periodically')  # For the student to understand
        shape = msg.height, msg.width
        data = np.array(msg.data).reshape(shape)
        self.cost_map[msg.y:msg.y + shape[0], msg.x: msg.x + shape[1]] = data

    # self.show_map()  # For the student to see that it works

    def show_map(self):
        if not self.cost_map is None:
            plt.imshow(self.cost_map)
            plt.show()







# calc the distance between two points
def dist(l1, l2):
    return pow((pow((l1[0] - l2[0]), 2) + pow((l1[1] - l2[1]), 2)), 0.5)






from ctypes import cdll

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':


    rospy.init_node('movebase_client_py')

    # agent = PushBot()
    # agent.shrink(agent.map_arr,4,4)
    # agent.move_wrapper()

    # agent.ballx = 2.500000
    # agent.bally = -0.900000
    # agent.goalx = 2.900000
    # agent.goaly = -2
    # agent.calculate_sphere_approach_direction()
    # agent.push_ball()
