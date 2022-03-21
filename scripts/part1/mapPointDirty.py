#!/usr/bin/env python

import rospy
import random
import time
from pathlib import Path
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

class map_point_dirty():
    def __init__(self , modelName ,dirty_location_x,dirty_location_y):
        self.modelPath = str(Path(os.path.dirname(os.path.realpath(__file__))).parent)
        self.modelPath = self.modelPath.replace('src/ros_submission/scripts',
                                                 'src/ros_submission/scripts/part1/turtlebot3_square/goal_box/model.sdf')
        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()
        self.goal_position = Pose()
        self.dirty_location_x = dirty_location_x
        self.dirty_location_y = dirty_location_y
        self.goal_position.position.x = self.dirty_location_x
        self.goal_position.position.y = self.dirty_location_y
        self.modelName = modelName
        self.obstacle_1 = 0.4, 0.4
        self.obstacle_2 = 0.4, -0.4
        self.obstacle_3 = -0.4, 0.4
        self.obstacle_4 = -0.4, -0.4


    def create_square(self):
            rospy.wait_for_service('gazebo/spawn_sdf_model')
            spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
            spawn_model_prox(self.modelName, self.model, 'robotos_name_space', self.goal_position, "world")
            rospy.loginfo("Dirty position : %.1f, %.1f", self.goal_position.position.x,
                              self.goal_position.position.y)

    def deleteModel(self):
            rospy.wait_for_service('gazebo/delete_model')
            del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
            del_model_prox(self.modelName)

    @staticmethod
    def find_square_and_remove(map_point_dirty_list, dirty_location_x, dirty_location_y):
        # try to delete dirty
        for point in map_point_dirty_list:
            if point.dirty_location_x == dirty_location_x and \
                    point.dirty_location_y == dirty_location_y:
                point.deleteModel()
                map_point_dirty_list.remove(point)
                break

        return map_point_dirty_list


