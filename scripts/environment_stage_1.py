#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn

class Env():
    def __init__(self, action_size, list_of_dirty_locations):
        #self.goal_x_list = [0] * len(list_of_dirty_locations)
        #self.goal_y_list = [0] * len(list_of_dirty_locations)
        self.goal_x= 0
        self.goal_y= 0
        self.which_dirty_is_goal= 0
        self.heading = 0
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('/tb3_0/odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.list_respawn_goal=[]
        for i, dirty in enumerate(list_of_dirty_locations):
            self.list_respawn_goal.append(Respawn(str(i)+"_dirty", dirty[0], dirty[1]))
        self.flag = 1

    def getGoalDistace(self):
        temp_list = []
        for i, respawn in enumerate(self.list_respawn_goal):
            goal_distance = round(math.hypot(respawn.goal_position.position.x - self.position.x, respawn.goal_position.position.y - self.position.y), 2)
            temp_list.append((goal_distance, i))

        return min(temp_list)

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        yaw = math.atan2(orientation.w,orientation.z)
        # _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 2)
    def getState(self, scan):
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

        if min_range > min(scan_range) > 0:
            done = True

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if current_distance < 0.2:
            self.get_goalbox = True

        return scan_range + [heading, current_distance], done

    def setReward(self, state, done, action):
        yaw_reward = []
        current_distance = state[-1]
        heading = state[-2]

        for i in range(5):
            angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
            tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            yaw_reward.append(tr)

        distance_rate = 2 ** (current_distance / self.goal_distance)
        reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate)

        if done:
            rospy.loginfo("Collision!!")
            reward = -200
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 200
            self.pub_cmd_vel.publish(Twist())
            #self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance, self.which_dirty_is_goal = self.getGoalDistace()
            self.goal_x, self.goal_y = self.list_respawn_goal[self.which_dirty_is_goal].create_square(True, delete=True)
            self.get_goalbox = False

        return reward

    def step(self, action):
        max_angular_vel = 1.5
        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.15
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)
        #8694

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, done = self.getState(data)
        reward = self.setReward(state, done, action)

        return np.asarray(state), reward, done

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/tb3_0/scan', LaserScan, timeout=5)
            except:
                pass

        # here we put the yoristic function
        self.goal_distance, self.which_dirty_is_goal = self.getGoalDistace()
        self.goal_x, self.goal_y = self.list_respawn_goal[self.which_dirty_is_goal].create_square()

        for i, respawn in enumerate(self.list_respawn_goal):
            if(i==self.which_dirty_is_goal):
                pass
            else:
                _, _ = respawn.create_square()

        state, done = self.getState(data)

        return np.asarray(state)

    def getRespawn(self, x, y):
        for respawn in self.list_respawn_goal:
            if(respawn.init_goal_x==x and respawn.init_goal_y==y):
                return respawn