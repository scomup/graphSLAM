#!/usr/bin/env python
# coding:utf-8

import numpy as np

class CostMap():
    def __init__(self, size =(800,800), original_point = (400,400), resolution = 0.1):
        self.size = size
        self.map_data = np.zeros(size)
        self.resolution = resolution
        self.original_point = np.array(original_point)
        self.ker = np.array([[1./16.,1./8.,1./16.],[1./8.,1./4.,1./8.],[1./16.,1./8.,1./16.]])

    def world_map(self, world_point):
        map_point = world_point/self.resolution + np.tile(self.original_point,(world_point.shape[0],1))
        return map_point

    def map_world(self, map_point):
        world_point = (map_point - np.tile(self.original_point,(map_point.shape[0],1)) ) * self.resolution
        return world_point

if __name__ == "__main__":
    costmap = CostMap(size =(10,10), original_point = (5,5), resolution = 1)
    pos = np.array([[1.,1],[1,1]])
    idx = costmap.world_map(pos).astype(int)
    costmap.map_data[idx[:,0],idx[:,1]] += 1
    print costmap.map_data[idx[:,0],idx[:,1]]
    print costmap.map_data


