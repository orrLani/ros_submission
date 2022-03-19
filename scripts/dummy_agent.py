#!/usr/bin/env python
import matplotlib.pyplot as plt
import rospy
import numpy as np
import multiprocessing
from agent_move import move
from map import CostmapUpdater
import math

class dummy_agent:

    ## id == 1
    def __init__(self, agent_id, init_location):
        rospy.init_node('movebase_client_py')

        self.agent_id = agent_id
        self.states = None
        self.last_position = None
        self.move_new_location = move
        self.init_location = init_location
        self.map_service = CostmapUpdater(self.agent_id)
        # wait to map
        while not self.map_service.is_map:
            pass

        self.static_map = self.map_service.cost_map.copy()
        # self.map_service.show_point(self.static_map,init_location)
        self.static_map = self.map_service.change_map(self.static_map)

        locations = np.where(self.static_map == 0)
        self.locations_map = np.concatenate(([locations[0]], [locations[1]]), axis=0)
        self.locations_map = self.locations_map.transpose()

        # change the values
        self.locations_map_gazebo = list()


        for val in self.locations_map:
            self.locations_map_gazebo.append(CostmapUpdater.map_to_position(val))


        # CostmapUpdater.map_to_position(np.array([198,198]))



        # get all the good points



        print('wow!!')


    def run(self):

        plt.imshow(self.static_map)
        plt.show()
        # for location_index in range(0,len(self.locations_map_gazebo),100):
        #     self.locations_map

        for location_index in range(0,len(self.locations_map_gazebo),100):
            position = dict()
            position['agent_id'] = self.agent_id
            position['y'] = math.floor(self.locations_map_gazebo[location_index][0]*10000)/10000.0
            position['x'] = math.floor(self.locations_map_gazebo[location_index][1]*10000)/10000.0
            print('i wanna go to location x-{} y-{}'.format( position['x'], position['y']))
            position['w'] = 1.0
            self.move_new_location(**position)
        # self.move_new_location(**self.init_location)
        print('finish!!')

if __name__ == '__main__':

    init_position_robot = dict()
    init_position_robot['x'], init_position_robot['y'], init_position_robot['w']  = 1.0, 0, 1

    dummy_agent = dummy_agent(agent_id=1,
              init_location=init_position_robot
              )

    dummy_agent.run()
    print 'wow!!!!'
    # map_service.show_map()