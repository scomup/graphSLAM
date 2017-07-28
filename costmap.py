#!/usr/bin/env python
# coding:utf-8

import numpy as np

class CostMap():
    def __init__(self, size =(1600,1600), original_point = (800,800), resolution = 0.05, cost_max = 100):
        self.size = size
        self.cost_max = cost_max
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

    def updateCostMap(self, map_point, val, use_gaussian):
        old_val = self.map_data[map_point[1], map_point[0]]
        if old_val > self.cost_max or old_val < -self.cost_max:
            return
        if not use_gaussian:
            new_val = old_val + val
            self.map_data[map_point[1], map_point[0]] = new_val
        else:
            self.map_data[map_point[1]-1 : map_point[1]+2, map_point[0]-1 : map_point[0]+2] += self.ker * val



if __name__ == "__main__":
    costmap = CostMap(size =(10,10), original_point = (5,5), resolution = 1)
    pos = np.array([[1.,1],[1,1]])
    idx = costmap.world_map(pos).astype(int)
    costmap.map_data[idx[:,0],idx[:,1]] += 1
    print costmap.map_data[idx[:,0],idx[:,1]]
    print costmap.map_data


