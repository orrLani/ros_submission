import matplotlib.pyplot as plt
import time

import rospy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import numpy as np
import cv2
import sys
import math


class CostmapUpdater:
    resolution = None
    orign = None
    height = None
    width = None
    map_org = None
    sys.setrecursionlimit(50000)

    def get_shape(self):
        # wait until get height and widht

        while CostmapUpdater.height is None or CostmapUpdater.width is None:
            pass

        return CostmapUpdater.height, CostmapUpdater.width



    def __init__(self,agent_id):
        self.cost_map = None
        self.shape = None
        self.is_map = False
        self.location_gazebo = None
        self.static_map = None
        self.locations_map = None
        self.locations_map_gazebo = list()

        rospy.Subscriber('/tb3_{}/move_base/global_costmap/costmap'.format(agent_id), OccupancyGrid, self.init_costmap_callback)
        rospy.Subscriber('/tb3_{}/move_base/global_costmap/costmap_updates'.format(agent_id), OccupancyGridUpdate, self.costmap_callback_update)


    def init_costmap_callback(self, msg):

        print('only once')  # For the student to understand
        self.shape = msg.info.height, msg.info.width
        self.cost_map = np.array(msg.data).reshape(self.shape)
        CostmapUpdater.resolution = msg.info.resolution
        CostmapUpdater.height = msg.info.height
        CostmapUpdater.width = msg.info.width
        CostmapUpdater.map_org = np.array([msg.info.origin.position.x, msg.info.origin.position.y])

        # create the map
        self.static_map = self.change_map(self.cost_map)
        locations = np.where(self.static_map == 0)
        self.locations_map = np.concatenate(([locations[0]], [locations[1]]), axis=0)

        self.locations_map = self.locations_map.transpose()

        # change the values

        for val in self.locations_map:
            self.locations_map_gazebo.append(CostmapUpdater.map_to_position(val))

        self.show_map()
        self.is_map = True
        time.sleep(5)

    def get_static_map(self):
        while self.static_map is None:
            pass

        return self.static_map

    def get_valid_locations_map(self):
        while self.locations_map is None:
            pass
        return self.locations_map


    def costmap_callback_update(self, msg):
        # print('periodically')  # For the student to understand
        shape = msg.height, msg.width
        data = np.array(msg.data).reshape(shape)
        self.cost_map[msg.y:msg.y + shape[0], msg.x: msg.x + shape[1]] = data
        # elf.show_map()  # For the student to see that it works

    def show_map(self):
        pass
        # if not self.cost_map is None:
        #     plt.imshow(self.cost_map)
        #     plt.show()

    @staticmethod
    def inside(map):
        pass
    @staticmethod
    def position_to_map(pos):

        # return ((pos - CostmapUpdater.map_org) //  CostmapUpdater.resolution)
        try:
            ret = np.floor(((pos - CostmapUpdater.map_org) //  CostmapUpdater.resolution) * 10000) / 10000.0
        except Exception as e:
            ret = (None,None)

        return ret

    @staticmethod
    def map_to_position(indices):

        # return indices * CostmapUpdater.resolution + CostmapUpdater.map_org
        return np.floor((indices * CostmapUpdater.resolution + CostmapUpdater.map_org)*10000)/10000.0

    @staticmethod
    def show_point(map, location):
        pass
        # plt.imshow(map)
        # # location
        # map_location = CostmapUpdater.position_to_map(np.array([location['x'], location['y']]))
        # plt.scatter([map_location[0]], [map_location[1]])
        # plt.show()


    @staticmethod
    def fill_array(array, x, y):
        print("x- {} y- {}".format(x,y))

        if array[x][y] == 1 or array[x][y] == 2:
            return
        if array[x][y] == 0:
            array[x][y] = 2

        CostmapUpdater.fill_array(array, x, y - 1)
        CostmapUpdater.fill_array(array, x + 1, y)
        CostmapUpdater.fill_array(array, x, y + 1)
        CostmapUpdater.fill_array(array, x - 1, y)


    @staticmethod
    def change_inside_out_side(map):
        # plt.imshow(map)
        # location
        map_location = CostmapUpdater.position_to_map(np.array([0, 0]))
        # plt.scatter([map_location[0]], [map_location[1]])
        CostmapUpdater.fill_array(map,int(map_location[0]),int(map_location[1]))

        # height = None
        # width = None
        return map

    def get_valid_location(self):
        pass


    @staticmethod
    def change_map(map):
        # <50 -> 0 you can move here
        #  >50 -> 1 you can't move here
        # <50 -> 2 you can move here
        map_copy = map.copy()

        map_copy = np.array(map_copy,dtype=np.uint8)
        map_copy = np.where(map_copy >= 20, 1, map_copy)
        map_copy = np.where((map_copy < 20) , np.where(map_copy!=1, 0, map_copy),map_copy)

        image = cv2.cvtColor(map_copy, cv2.COLOR_GRAY2BGR)
        kernel = np.ones((5, 5), np.uint8)
        image = cv2.erode(image, kernel)

        copy_arr = np.array(image)

        copy_arr = copy_arr[:, :, 0]

        # copy_arr = CostmapUpdater.change_inside_out_side(copy_arr)
        # plt.imshow(copy_arr)
        # plt.show()

        # get your location

        return copy_arr