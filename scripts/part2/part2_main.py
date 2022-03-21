#!/usr/bin/env python2.7

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
import cv2
import imutils
import time
import numpy as np
import math
from map import CostmapUpdater
from agent_move import move

# itay added imports
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Twist, Pose , Point
import matplotlib.pyplot as plt
epsilon_circle = 2
BlueLower = (100, 150, 0)
BlueUpper = (140, 255, 255)

RedLower = (0, 50, 50)
RedUppder = (10, 255, 255)

center_x =  960.5
center_y = 540.5

K_matrix = np.array([[1206.8897719532354, 0.0, 960.5],[0.0, 1206.8897719532354, 540.5],
                    [0.0, 0.0, 1.0]])
f = 1206.8897719532354
print K_matrix

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from part2_pushing_bot import PushBot , MapServicePart2
class Camera:

    def __init__(self):
        # get the subscribe from the cemrea
        rospy.init_node('camera', anonymous=True)
        self.ms = MapServicePart2()
        # get my location
        self.robot_camera_location_gazebo = dict()
        self.robot_camera_location_gazebo_odom = dict()
        _ = rospy.Subscriber("/tb3_1/odom", Odometry, self.update_robot_camera_my_location)

        # get image from camera
        self.image_index = 0
        _ = rospy.Subscriber("/tb3_1/camera/rgb/image_raw", Image, self.update_image_robot_camera)

        # get leaser scan from leaser
        self.ranges = list()
        self.ranges_leaser = list()
        _ = rospy.Subscriber('/tb3_1/scan', LaserScan, self.update_leaser_robot_camera)

        # get the location of the balls
        self.balls = list()

        ## itay added variables # Ball_Queue
        self.request_pub = rospy.Publisher('communication_channel', Point, queue_size=999)
        rospy.wait_for_service('/tb3_1/static_map')
        static_map = rospy.ServiceProxy('/tb3_1/static_map', GetMap)
        # static map arguments (we address the map_arr for info about pixel)
        self.map_data = static_map().map
        self.map_org = np.array([self.map_data.info.origin.position.x, self.map_data.info.origin.position.y])
        shape = self.map_data.info.height, self.map_data.info.width
        self.map_arr = np.array(self.map_data.data, dtype='float32').reshape(shape)
        self.robot_camera_location_gazebo_odom['x'] = 0
        self.robot_camera_location_gazebo_odom['y'] = 0
        self.costmap_updater = CostmapUpdater(1)
        # self.x_shrinkage =
        # self.y_shrinkage = cols
        # self.cols = rows = data.shape[0] / rows
        # self.rows = cols = 1
        # self.square_arr = 1
        self.client = actionlib.SimpleActionClient('tb3_1/move_base', MoveBaseAction)
        self.shrink(self.map_arr,8,8)
        self.queue = Ball_Queue()


    def check_ball_next_to_wall(self, ballx,bally):
        rospy.sleep(1)
        index_up = self.ms.position_to_map(np.array([bally, ballx+0.5]))
        x = index_up[0]
        y = index_up[1]
        x = int(x)
        y = int(y)
        if  self.map_arr[x][y] == -1 or self.map_arr[x][y] == 100:
            return True
        index_down = self.ms.position_to_map(np.array([bally, ballx-0.5]))
        x = index_down[0]
        y = index_down[1]
        x = int(x)
        y = int(y)
        if self.map_arr[x][y] == -1 or self.map_arr[x][y] == 100:
            return True
        index_right = self.ms.position_to_map(np.array([bally+0.5, ballx]))
        x = index_right[0]
        y = index_right[1]
        x = int(x)
        y = int(y)
        if self.map_arr[x][y] == -1 or self.map_arr[x][y] == 100:
            return True
        index_left = self.ms.position_to_map(np.array([bally-0.5, ballx]))
        x = index_left[0]
        y = index_left[1]
        x = int(x)
        y = int(y)
        if self.map_arr[x][y] == -1 or self.map_arr[x][y] == 100:
            return True
        return False
    def shrink(self, data, rows, cols):

        self.x_shrinkage = rows
        self.y_shrinkage = cols
        self.cols = rows = data.shape[0] / rows
        self.rows = cols = data.shape[1] / cols
        self.square_arr = data.reshape(rows, data.shape[0] / rows, cols, data.shape[1] / cols).sum(axis=1).sum(
            axis=2)

    def unshirnk(self, shrunk_x, shrunk_y):
        return shrunk_x * self.x_shrinkage + 1, shrunk_y * self.y_shrinkage + 1

    def get_shrinked_pos(self, x, y):
        return int(x / self.x_shrinkage), int(y / self.y_shrinkage)

    # or already been traversed
    def is_available(self, x, y):
        if self.square_arr[x][y] != 0:
            return False
        real_x, real_y = self.unshirnk(x, y)
        if self.costmap_updater.cost_map[real_x][real_y] > 65:
            print("WE SAW BALL")
            self.square_arr[x][y] = 333
            return False
        return True

    # will start with given location
    def traverse_loop(self, x, y):
        # print(666)
        real_x, real_y = self.unshirnk(x, y)

        arr = self.ms.map_to_position(np.array([real_y, real_x]))
        # print(real_x,real_y,arr)
        self.ms.movebase_client(1, arr[0], arr[1])
        self.square_arr[x][y] = 666  # marked that it has already been visited
        rospy.sleep(0.1)
        if self.is_available(x + 1, y):
            self.traverse_loop(x + 1, y)
        if self.is_available(x - 1, y):
            self.traverse_loop(x - 1, y)
        if self.is_available(x, y + 1):
            self.traverse_loop(x, y + 1)
        if self.is_available(x, y - 1):
            self.traverse_loop(x, y - 1)

        # if we got here we are stuck (all neighboring positions already been traversed or blocked
        # we assume its already been traversed and not blocked.

    def preprocess(self, val):
        return 0 if val == 0 else 100

    def find_closest(self):
        for x in range(self.rows):
            for y in range(self.cols):
                if self.square_arr[x][y] < 100:
                    return x, y
        return 0, 0

    def move_wrapper(self):
        self.square_arr = np.abs(np.sign(self.square_arr) * 100)
        rospy.sleep(0.1)
        og_y = self.robot_camera_location_gazebo_odom['x']
        og_x = self.robot_camera_location_gazebo_odom['y']
        start_x, start_y = self.ms.position_to_map(np.array([og_x, og_y]))
        start_x, start_y = self.get_shrinked_pos(start_x, start_y)
        self.traverse_loop(start_x, start_y)
        while np.any(self.square_arr < 100):
            x, y = self.find_closest()
            self.traverse_loop(x, y)

    def update_robot_camera_my_location(self, msg):
        # print(1)
        pose = msg.pose.pose
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # print yaw
        self.robot_camera_location_gazebo_odom['x'] = pose.position.x
        self.robot_camera_location_gazebo_odom['y'] = pose.position.y
        self.robot_camera_location_gazebo_odom['yaw'] = yaw
        # print self.robot_camera_location_gazebo

    def update_image_robot_camera(self, msg):
        # print(2)
        # load the image
        bridge = CvBridge()
        self.ranges = self.ranges_leaser
        self.robot_camera_location_gazebo = self.robot_camera_location_gazebo_odom
        try:
            image = bridge.imgmsg_to_cv2(msg, "passthrough")
        except Exception as e:
            # print "no!!"
            return


        # resize the image
        image = imutils.resize(image, width=1920)

        # Converts an image from one color space to another
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # cv2.imwrite("image{}.jpg".format(self.image_index), image)
        self.image_index += 1

        # print 'image save'
        # find all the circles in the image
        # for blue
        image, _ = self.find_circle(image,'blue')
        # for red
        image, _ = self.find_circle(image, 'red')
        # show image
        if image is not None:
            cv2.imshow("Image Window", image)
            cv2.waitKey(3)
        time.sleep(1)

    def update_leaser_robot_camera(self, msg):
        # print(3)
        # print msg.ranges
        self.ranges_leaser = msg.ranges
        # print 'right'
        # look left
        # print self.ranges[0:10:1]
        # look right
        # print 'left'
        # print self.ranges[0:-10:-1]
        # pass

    def find_circle(self,image, ball_color):
        # set Gaussian
        blurred = cv2.GaussianBlur(image, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        if ball_color == 'blue':
            colorUpper = BlueUpper
            colorLower = BlueLower
        else:
            colorUpper = RedUppder
            colorLower = RedLower

        msk = cv2.inRange(hsv, colorLower, colorUpper)
        msk = cv2.erode(msk, None, iterations=2)
        msk = cv2.dilate(msk, None, iterations=2)

        # find circles by the image
        cnts = cv2.findContours(msk.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)

        cnts = imutils.grab_contours(cnts)

        circles_pixels = list()

        Ki = np.linalg.inv(K_matrix)
        # find red or blue ball base on the color
        for c in cnts:
            # c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))  # find the center of the shape

            if (center[0] - int(x)) ** 2 + (center[1] - int(y)) ** 2 > epsilon_circle:
                continue

            # calculate angular
            r_1 = Ki.dot([x, y, 1.0])
            r_center = Ki.dot([center_x, y, 1.0])
            cos_angle = r_1.dot(r_center) / (np.linalg.norm(r_1) * np.linalg.norm(r_center))
            angle_degrees = math.degrees(math.acos(cos_angle))
            if x > center_x:
                angle_degrees *= -1
            # print angle_degrees

            range_currect = 'inf'
            degree = None
            len_of_ball = None
            # check the ranges:
            if self.ranges is not None and len(self.ranges)>0:
                # print  len(self.ranges)
                degree = int(math.floor(angle_degrees))

                range_currect = min(self.ranges[degree],self.ranges[degree-1],self.ranges[degree+1])
                len_of_ball = (radius *  range_currect / cos_angle)/f
                classified_size = None
                if len_of_ball == np.inf:
                    continue
                if len_of_ball > 0.15:
                    classified_size = 'big'
                    rad_size = 0.2
                else:
                    classified_size = 'small'
                    rad_size = 0.1
                # avoid distortions
                if not(degree and degree>= -10 and degree<=10):
                    continue
                # aviod prblems with range
                # print range_currect
                if not(range_currect and range_currect>= 1 and range_currect<=3):
                    continue

                # get the ball location

                yaw_g = self.robot_camera_location_gazebo['yaw']
                x_g = self.robot_camera_location_gazebo['x']
                y_g = self.robot_camera_location_gazebo['y']
                # print (x_g,y_g,yaw_g)
                # print np.cos(yaw_g)
                # print np.sin(yaw_g)
                a_mul = np.sign(np.cos(yaw_g))
                b_mul = np.sign(np.sin(yaw_g))
                # (x_x,y_y) = (x_g + (a_mul*rad_size) +
                #              ( range_currect * np.cos(yaw_g)),
                #              y_g + (b_mul*rad_size) + (range_currect * np.sin(yaw_g)))
                (x_x, y_y) = (x_g  +
                              ((rad_size+range_currect) * np.cos(yaw_g)),
                              y_g + ((range_currect+rad_size) * np.sin(yaw_g)))

                # draw
                cv2.circle(image, (int(x), int(y)), int(radius),(0, 255, 255), 2)

                cv2.circle(image, center, 5, (255, 255, 255), -1)

                cv2.circle(image, (int(center_x), int(y)), 5, (255, 255, 255), -1)

                # write the degree
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 1
                color = (255, 255, 0)
                thickness = 1
                org = (int(x), int(y))
                # request = Ball_and_Goal(x_x,y_y,0,0)
                request = Point()
                request.x = x_x
                request.y = y_y

                # print(request)
                # print("range current is :" , str(range_currect))

                # won't publish if ball is already next to a wall
                # need to add difference between ball according to size and color
                if not self.check_ball_next_to_wall(x_x,y_y):
                    request.z = self.decide_goal_location(ball_color,classified_size)
                    self.request_pub.publish(request)
                    # self.queue.insert(request)
                    # self.request_pub.publish(self.queue)

                image = cv2.putText(image, str(x_x) +'::'+str(y_y)+'::'+classified_size+":::"+ball_color, org, font,
                                    fontScale, color, thickness, cv2.LINE_AA)
                # image = cv2.putText(image,  int(org[0])+5,int(org[1])+5, font,
                #                     fontScale, color, thickness, cv2.LINE_AA)
                # get the ball location

                circles_pixels.append(center)

        return (image, circles_pixels)

        ###
        ### FROM HERE ITS THE CAMERA
        ###

    def decide_goal_location(self,color,size):
        if color == "red" and size == "big":
            return 1
        if color == "red" and size == "small":
            return 2
        if color == "blue" and size == "big":
            return 3
        return 4

class move_robot_with_camera:
    def __init__(self):
        # move to 15 random points in the map

        # init
        self.move_new_location = move
        self.map_service = CostmapUpdater(1)
        # wait to map
        while not self.map_service.is_map:
            pass

        self.static_map = self.map_service.cost_map.copy()
        self.static_map = self.map_service.change_map(self.static_map)
        locations = np.where(self.static_map == 0)
        self.locations_map = np.concatenate(([locations[0]], [locations[1]]), axis=0)
        self.locations_map = self.locations_map.transpose()
        self.locations_map_gazebo = list()

        for val in self.locations_map:
            self.locations_map_gazebo.append(CostmapUpdater.map_to_position(val))

        self.locations_map_gazebo_array = np.transpose(np.array(self.locations_map_gazebo))



    def run(self):
        for location_index in range(0,len(self.locations_map_gazebo),100):
            position = dict()
            position['agent_id'] = 1
            position['y'] = math.floor(self.locations_map_gazebo[location_index][0]*10000)/10000.0
            position['x'] = math.floor(self.locations_map_gazebo[location_index][1]*10000)/10000.0
            print('i wanna go to location x-{} y-{}'.format( position['x'], position['y']))
            position['w'] = 0.5
            self.move_new_location(**position)


def dist(l1, l2):
    return pow((pow((l1[0] - l2[0]), 2) + pow((l1[1] - l2[1]), 2)), 0.5)

class Ball_and_Goal:
    def __init__(self, ballx,bally,goalx,goaly):
        self.ball = [ballx, bally]
        self.goal = [goalx,goaly]

class Ball_Queue:
    def __init__(self):
        self.ball_list = list()

    def pop(self):
        val = self.ball_list.pop(0)
        return val
    def insert(self, val):
        if not any(dist(x.ball,val.ball) < 0.2 for x in self.ball_list ):
            self.ball_list.append(val)
        else:
            print("we didnt add to requests")

if __name__ == '__main__':
    global active_queue
    push = PushBot()
    active_queue = push.queue
    c = Camera()

    # plt.scatter(agent.locations_map_gazebo_array[1],agent.locations_map_gazebo_array[0])
    # plt.show()

    # agent.run()
    c.move_wrapper()
    while not rospy.is_shutdown():
        rospy.spin()

    # while 1:
    #     pass
