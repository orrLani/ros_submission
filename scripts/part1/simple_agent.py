#!/usr/bin/env python
import matplotlib.pyplot as plt
import rospy
import numpy as np
import multiprocessing
from agent_move import move
from map import CostmapUpdater
import math
from std_msgs.msg import String

def Diff(li1, li2):
    return list(set(li1) - set(li2))

class dummy_agent:

    ## id == 1
    def __init__(self, agent_id, init_location):
        rospy.init_node('movebase_client_py')

        self.agent_id = agent_id
        self.states = None
        self.last_position = None
        self.move_new_location = move
        self.initloginfo_location = init_location
        self.map_service = CostmapUpdater(self.agent_id)
        # wait to map
        while not self.map_service.is_map:
            pass

        self.dirty_location_list = list()
        self.map_point_dirty_list = list()
        self.dirty_location_msg = []
        self.dirty_location = list()
        _ = rospy.Subscriber('/dirt', String, self.get_dirty_update)
        self.flag = 1

    print('wow!!')




    def get_dirty_update(self, msg):

        # check from update from dirty
        if len(str(msg.data)) == len(str(self.dirty_location_msg)):
            # no update
            return
        ## yyyy
        flag = False
        if (len(str(self.dirty_location_msg))) > 0:
            flag = True

        rospy.loginfo("update dirty!")
        self.dirty_location_msg = msg.data
        rospy.loginfo(msg.data)
        # Convert dirty msg to list
        if ('(' or ')') in self.dirty_location_msg:
            dirty_string = self.dirty_location_msg.replace('(', '')
            _dirty_string = dirty_string.replace(')', ',')
        else:
            dirty_string = self.dirty_location_msg.replace('[', '')
            _dirty_string = dirty_string.replace(']', ',')

        _dirty_list = _dirty_string.split(',')

        dirty_list = []

        for i in range(0, len(_dirty_list[:-1]), 2):
            dirty_list.append((float(_dirty_list[i]), float(_dirty_list[i + 1])))

        # check if we need to create dirty or delete dirty
        dirty_collect = Diff(self.dirty_location, dirty_list)
        self.dirty_location = dirty_list

        ## how to know who collected



    def run(self):

        plt.imshow(self.static_map)
        plt.show()
        # for location_index in range(0,len(self.locations_map_gazebo),100):
        #     self.locations_map

        for i in self.dirty_location:
            position = dict()
            position['agent_id'] = self.agent_id
            position['x'] = i[0]
            position['y'] = i[1]
            # print('i wanna go to location x-{} y-{}'.format( position['x'], position['y']))
            position['w'] = 1.0
            self.move_new_location(**position)
        # self.move_new_location(**self.init_location)
        print('finish!!')


if __name__ == '__main__':
    init_position_robot = dict()
    init_position_robot['x'], init_position_robot['y'], init_position_robot['w'] = 1.0, 0, 1

    dummy_agent = dummy_agent(agent_id=1,
                              init_location=init_position_robot
                              )

    dummy_agent.run()
    print 'wow!!!!'
    # map_service.show_map()