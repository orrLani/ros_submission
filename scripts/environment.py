#!/usr/bin/env python
import time

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from std_msgs.msg import String
import subprocess
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from mapPointDirty import map_point_dirty

def Diff(li1, li2):
    return list(set(li1) - set(li2))

class Env():
    def __init__(self,agent_id, action_size):
        self.goal_x = 0
        self.goal_y = 0
        self.which_dirty_is_goal = 0
        self.heading = 0
        # self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('/tb3_{}/cmd_vel'.format(agent_id), Twist, queue_size=5)

        # gazebo messages
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

        # input messages
        _ = rospy.Subscriber('/tb3_{}/odom'.format(agent_id), Odometry, self.get_my_odometry)
        _ = rospy.Subscriber('/tb3_{}/odom'.format(1- agent_id), Odometry, self.get_opp_odometry)

        # position
        # sub_current_pose = rospy.Subscriber('/tb3_{}'.format(agent_id), PoseWithCovarianceStamped, callback0)
        

        # get enmy goal
        # _ = rospy.Subscriber('/tb3_{}/goal'.format(1- agent_id), Odometry, self.get_opp_odometry)


        # dirty
        self.dirty_location_list = list()
        self.map_point_dirty_list = list()
        self.dirty_location_msg = ""
        self.dirty_location = list()
        _ = rospy.Subscriber('/dirt', String, self.get_dirty_update)
        self.flag = 1

        self.reset_dirt = rospy.Publisher('reset_command', String, queue_size=1)

        # process
        self.dirty_proc = None
        self.dummy_agent_proc = None

    # update the dirty
    def get_dirty_update(self, msg):

        # check from update from dirty
        if len(str(msg.data)) == len(str(self.dirty_location_msg)):
            # no update
            return

        rospy.loginfo("update dirty!")
        self.dirty_location_msg = msg.data
        rospy.loginfo(msg.data)
        # Convert dirty msg to list
        if ('(' or ')') in self.dirty_location_msg:
                dirty_string = self.dirty_location_msg.replace('(','')
                _dirty_string = dirty_string.replace(')', ',')
        else:
            dirty_string = self.dirty_location_msg.replace('[','')
            _dirty_string = dirty_string.replace(']', ',')

        _dirty_list = _dirty_string.split(',')

        dirty_list = []
        for i in range(0,len(_dirty_list[:-1]),2):
            dirty_list.append((float(_dirty_list[i]),float(_dirty_list[i+1])))

        # check if we need to create dirty or delete dirty
        dirty_collect = Diff(self.dirty_location, dirty_list)
        self.dirty_location = dirty_list

        if len(dirty_collect) > 0:
            # need to delete dirty
            rospy.loginfo("the dirty that was collect is"+ str(dirty_collect))
            rospy.loginfo("The dirty is collect is"+str(dirty_collect))
            self.map_point_dirty_list = map_point_dirty.find_square_and_remove( self.map_point_dirty_list,
                                                                                dirty_collect[0][0],dirty_collect[0][1])
        else:
            # add to dirty list
            self.map_point_dirty_list = []
            for i, dirty in enumerate(self.dirty_location):
                self.map_point_dirty_list.append(map_point_dirty("dirty_"+str(i), dirty[0], dirty[1]))
            # create dirty
            for dirty in self.map_point_dirty_list:
                dirty.create_square()

    def getMinDistace(self):
        temp_list = []
        for i, map_point_dirty in enumerate(self.map_point_dirty_list):
            min_distance = round(math.hypot(map_point_dirty.goal_position.position.x - self.position.x, map_point_dirty.goal_position.position.y - self.position.y), 2)
            temp_list.append((min_distance, i))

        return min(temp_list)

    def get_opp_odometry(self, odom):
        pass

    def get_my_odometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        yaw = math.atan2(orientation.w,orientation.z)
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 2)





    def getState(self, scan):
        # TODO he state depend only by the scan - need to be change that the state will be depend in more things.

        scan_range = []
        heading = self.heading
        min_range = 0.13
        done = False

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if current_distance < 0.2:
            self.get_goalbox = True

        # conditions for done
        # by min range
        if min_range > min(scan_range) > 0:
            done = True

        # robot finish collecting all the dirty
        if len(self.map_point_dirty_list) == 0:
            done = True

        # TODO ADD CONDITION - if itai Mechonot get more points from us.

        return scan_range + [heading, current_distance], done


    def setReward(self, state, done, action):
        # the reward function, here we need to change the function.
        return 1

    def step(self, action):
        # doing a step in the environment.

        # the max velocity that we can a chive (may be need to change)
        max_angular_vel = 1.5

        # doing a step in the env (in this case change the velocity)
        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.15
        vel_cmd.angular.z = ang_vel
        # pub the velocity ( in our case we need to think about some think else)
        self.pub_cmd_vel.publish(vel_cmd)

        # change the state - in this case the state is depended on only by the scan.
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/tb3_1/scan', LaserScan, timeout=5)
            except:
                pass

        # change the state by the data scan.
        state, done = self.getState(data)

        # set the reward
        reward = self.setReward(state, done, action)

        # TODO REMOVE DONE = FALSE
        done = False
        # check if is done by env
        return np.asarray(state), reward, done

    def reset(self):
        # reset the simulator
        rospy.loginfo("Prepare to reset the simulator!!")
        rospy.wait_for_service('gazebo/reset_simulation')

        try:
            self.pause_proxy()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        # reset the dirty publisher
        # gnome-terminal --title="Python dirty" -e "bash -c \"cd ~/my_ws ; source devel/setup.bash ; export TURTLEBOT3_MODEL=burger ; sleep 5 ; rosrun MRS_236609 dirt_publisher_ex3.py ; $SHELL\""

        # remove all dirty
        # run again the dirty

        rospy.loginfo("Prepare to reset dirty!!")
        for dirty in self.map_point_dirty_list:
            dirty.deleteModel()

        # reset dirty
        self.dirty_location_list = list()
        self.map_point_dirty_list = list()
        self.dirty_location_msg = ""
        self.dirty_location = list()
        command = 'rosrun MRS_236609 dirt_publisher_ex3.py'
        self.common_dirty = command


        # command = 'recordmydesktop --on-the-fly-encoding -o videos/episode'
        # self.dirty_proc = subprocess.Popen('rosrun MRS_236609 dirt_publisher_ex3.py', shell=True)
        # self.dirty_proc = subprocess.Popen('rosrun MRS_236609 dirt_publisher_ex3.py', shell=True)

        # dirty process
        if self.dirty_proc is not None:
            self.dirty_proc.kill()

        # agent process
        if self.dummy_agent_proc is not None:
            self.dummy_agent_proc.kill()

        self.dirty_proc = subprocess.Popen('rosrun MRS_236609 dirt_publisher_ex3.py', shell=True)
        self.dummy_agent_proc = subprocess.Popen('rosrun ros_submission dummy_agent.py', shell=True)

        time.sleep(5)
        subprocess.call('/home/orr/my_ws/src/ros_submission/scripts/ros_rviz_shell')
        # subprocess.call('/home/orr/my_ws/src/ros_submission/scripts/ros_dirty_shell')
        time.sleep(10)
        rospy.loginfo("load the rviz")
        # reset opp agent
        # subprocess.call('/home/orr/my_ws/src/ros_submission/scripts/ros_dummy_shell')
        # time.sleep(10)
        # resert rviz
        # subprocess.call('/home/orr/my_ws/src/ros_submission/scripts/ros_rviz_shell')
        # time.sleep(20)


        # gnome-terminal --title="Python dirty" -e "bash -c \"cd ~/my_ws ; source devel/setup.bash ; export TURTLEBOT3_MODEL=burger ; sleep 5 ; rosrun MRS_236609 dirt_publisher_ex3.py ; $SHELL\""



        # TODO REMOVE THATS (we need that?)
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/tb3_0/scan', LaserScan, timeout=5)
            except:
                pass
        # TODO create dirty function
        # for dirty in self.map_point_dirty_list:
        #    _,_ = dirty.create_square()

        state, _ = self.getState(data)

        return np.asarray(state)
