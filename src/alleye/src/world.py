#!/usr/bin/env python3

import yaml
import rospkg
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
import numpy as np


"""
BSD 2-Clause License
Copyright (c) 2017, Andrew Dahdouh
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CON   TRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import matplotlib.pyplot as plt
from copy import deepcopy

COLOR_MAP = (0, 8)


class PathPlanner:

    def __init__(self, grid, visual=False):
        """
        Constructor of the PathPlanner Class.
        :param grid: List of lists that represents the
        occupancy map/grid. List should only contain 0's
        for open nodes and 1's for obstacles/walls.
        :param visual: Boolean to determine if Matplotlib
        animation plays while path is found.
        """
        self.grid = grid
        self.visual = visual
        self.heuristic = None
        self.goal_node = None

    def calc_heuristic(self):
        '''
        calculates the cost of each square in the grid
        '''

        row = len(self.grid)
        col = len(self.grid[0])

        self.heuristic = [[0 for x in range(col)] for y in range(row)]
        for i in range(row):
            for j in range(col):
                row_diff = abs(i - self.goal_node[0])
                col_diff = abs(j - self.goal_node[1])
                self.heuristic[i][j] = int(abs(row_diff - col_diff) + min(row_diff, col_diff) * 2)

        print("Heuristic:")
        for i in range(len(self.heuristic)):
            print(self.heuristic[i])

    def a_star(self, start_cart, goal_cart):
        """
        A* Planner method.
        """
        goal = [goal_cart[1], goal_cart[0]]
        self.goal_node = goal
        init = [start_cart[1], start_cart[0]]
        # Calculate the Heuristic for the map
        self.calc_heuristic()

        print(init, goal)

        if self.visual:
            viz_map = deepcopy(self.grid)
            fig = plt.figure(figsize=(12, 12))
            ax = fig.add_subplot(111)
            ax.set_title('Occupancy Grid')
            plt.xticks(visible=False)
            plt.yticks(visible=False)
            plt.imshow(viz_map, origin='upper', interpolation='none', clim=COLOR_MAP)
            ax.set_aspect('equal')
            plt.pause(2)
            viz_map[init[0]][init[1]] = 5  # Place Start Node
            viz_map[goal[0]][goal[1]] = 6
            plt.imshow(viz_map, origin='upper', interpolation='none', clim=COLOR_MAP)
            plt.pause(2)

        # Different move/search direction options:

        delta = [[-1, 0],  # go up
                 [0, -1],  # go left
                 [1, 0],  # go down
                 [0, 1]]  # go right
        delta_name = ['^ ', '< ', 'v ', '> ']


        #I havent tried these yet but they should work

        # If you wish to use diagonals:
        # delta = [[-1, 0],  # go up
        #          [0, -1],  # go left
        #          [1, 0],  # go down
        #          [0, 1],  # go right
        #          [-1, -1],  # upper left
        #          [1, -1],  # lower left
        #          [-1, 1],  # upper right
        #          [1, 1]]  # lower right
        # delta_name = ['^ ', '< ', 'v ', '> ', 'UL', 'LL', 'UR', 'LR']

        # Modified from A* Examples by Sebastian Thrun:

        closed = [[0 for col in range(len(self.grid[0]))] for row in range(len(self.grid))]
        shortest_path = [['  ' for _ in range(len(self.grid[0]))] for _ in range(len(self.grid))]
        closed[init[0]][init[1]] = 1

        expand = [[-1 for col in range(len(self.grid[0]))] for row in range(len(self.grid))]
        delta_tracker = [[-1 for _ in range(len(self.grid[0]))] for _ in range(len(self.grid))]

        cost = 1
        x = init[0]
        y = init[1]
        g = 0
        f = g + self.heuristic[x][y]
        open = [[f, g, x, y]]

        found = False  # flag that is set when search is complete
        resign = False  # flag set if we can't find expand
        count = 0
        while not found and not resign:
            if len(open) == 0:
                resign = True
                if self.visual:
                    plt.text(2, 10, s="No path found...", fontsize=18, style='oblique', ha='center', va='top')
                    plt.imshow(viz_map, origin='upper', interpolation='none', clim=COLOR_MAP)
                    plt.pause(.5)
                return -1
            else:
                open.sort()
                open.reverse()
                next = open.pop()
                x = next[2]
                y = next[3]
                g = next[1]
                expand[x][y] = count
                count += 1

                if x == goal[0] and y == goal[1]:
                    found = True
                    if self.visual:
                        viz_map[goal[0]][goal[1]] = 7
                        plt.text(2, 10, s="Goal found!", fontsize=18, style='oblique', ha='center', va='top')
                        plt.imshow(viz_map, origin='upper', interpolation='none', clim=COLOR_MAP)
                        plt.pause(2)
                else:
                    for i in range(len(delta)):
                        x2 = x + delta[i][0]
                        y2 = y + delta[i][1]
                        if len(self.grid) > x2 >= 0 <= y2 < len(self.grid[0]):
                            if closed[x2][y2] == 0 and self.grid[x2][y2] == 0:
                                g2 = g + cost
                                f = g2 + self.heuristic[x2][y2]
                                open.append([f, g2, x2, y2])
                                closed[x2][y2] = 1
                                delta_tracker[x2][y2] = i
                                if self.visual:
                                    viz_map[x2][y2] = 3
                                    plt.imshow(viz_map, origin='upper', interpolation='none', clim=COLOR_MAP)
                                    plt.pause(.5)

        current_x = goal[0]
        current_y = goal[1]
        shortest_path[current_x][current_y] = '* '
        full_path = []
        while current_x != init[0] or current_y != init[1]:
            previous_x = current_x - delta[delta_tracker[current_x][current_y]][0]
            previous_y = current_y - delta[delta_tracker[current_x][current_y]][1]
            shortest_path[previous_x][previous_y] = delta_name[delta_tracker[current_x][current_y]]
            full_path.append((current_x, current_y))
            current_x = previous_x
            current_y = previous_y
        full_path.reverse()
        print("Found the goal in {} iterations.".format(count))
        print("full_path: ", full_path[:-1])
        for i in range(len(shortest_path)):
            print(shortest_path[i])

        if self.visual:
            for node in full_path:
                viz_map[node[0]][node[1]] = 7
                plt.imshow(viz_map, origin='upper', interpolation='none', clim=COLOR_MAP)
                plt.pause(5)

            # Animate reaching goal:
            viz_map[goal[0]][goal[1]] = 8
            plt.imshow(viz_map, origin='upper', interpolation='none', clim=COLOR_MAP)
            plt.pause(5)

        return init, full_path[:-1]


class World():
    def __init__(self, width=20):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(1.0)
        self.rospack = rospkg.RosPack()
        
        self.width = width
        self.grid = np.zeros((self.width,self.width))
        self.obstacles = []
        self.obs_bb = []
        self.max_bound = np.zeros((1,2))
        self.box_size = 0

        self.goal_id = 0
        self.start_id = 0

        self.load_config()
        self.add_obstacles()
        
        self.planner = PathPlanner(self.grid, True)

        self.pub_plan = rospy.Publisher(f"/path", Path, queue_size=10)
        
    def load_config(self):
        path = self.rospack.get_path('alleye')
        with open(path + '/config/world.yaml') as f:
            world_dict = yaml.load(f, Loader=yaml.FullLoader)
        
        self.start_id = world_dict["start"]
        self.goal_id = world_dict["goal"]
        self.new_origin = np.array(world_dict["boundaries"]["top_left"])
        self.max_bound = np.array([world_dict["boundaries"]["bot_right"]])
        self.max_bound = self.world_to_grid(self.max_bound)
        self.box_size = self.max_bound[0][0]/self.width

        # Listen for tag transforms
        for obs in world_dict['obstacles']:
            try:
                tag_id = obs['id']
                trans = self.tfBuffer.lookup_transform('world', f'tag_{tag_id}', rospy.Time(0))
                self.obstacles.append(np.array([trans.transform.translation.x, trans.transform.translation.y]))
                self.obs_bb.append(np.array(obs['bounds']))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(e)
                continue

    def world_to_grid(self, points): 
        points = np.asfarray(points)
        points -= self.new_origin
        points = np.flip(points, axis=1)
        points *= -1
        return points
        
    def grid_to_world(self, points):
        points = np.asfarray(points)
        points *= -1
        points = np.flip(points, axis=1)
        points += self.new_origin
        return points

    def add_obstacles(self):
        obstacles = self.world_to_grid(self.obstacles)
        rospy.loginfo(f"Max: {self.max_bound}")
        rospy.loginfo(f"Before: {self.obstacles}\nAfter: {obstacles}")
        for i in range(len(obstacles)):
            bb_tl = obstacles[i] - self.obs_bb[i]
            bb_br = obstacles[i] + self.obs_bb[i]

            tl = np.floor(bb_tl/self.box_size).astype(int)
            br = np.floor(bb_br/self.box_size).astype(int) + 1
            rospy.loginfo(f"{tl}, {br}")

            i_min = tl[0]
            j_min = tl[1]
            i_max = br[0]
            j_max = br[1]

            if i_min < 0:
                i_min = 0
            elif i_min > self.width:
                i_min = self.width

            if i_max < 0:
                i_max = 0
            elif i_max > self.width:
                i_max = self.width

            if j_min < 0:
                j_min = 0
            elif j_min > self.width:
                j_min = self.width
            
            if j_max < 0:
                j_max = 0
            elif j_max > self.width:
                j_max = self.width

            self.grid[j_min:j_max,i_min:i_max] = 1

    def generate_path(self):
        # Listen for tag transforms
        try:
            start_trans = self.tfBuffer.lookup_transform('world', f'tag_{self.start_id}', rospy.Time(0))
            goal_trans = self.tfBuffer.lookup_transform('world', f'tag_{self.goal_id}', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(e)
            return

        start = self.world_to_grid(
            np.array([[start_trans.transform.translation.x, start_trans.transform.translation.y]]))
        goal = self.world_to_grid(
            np.array([[goal_trans.transform.translation.x, goal_trans.transform.translation.y]]))

        start_idx = np.floor(start[0]/self.box_size).astype(int)
        start_idx = np.flip(start_idx)
        goal_idx = np.floor(goal[0]/self.box_size).astype(int)
        goal_idx = np.flip(goal_idx)

        rospy.loginfo(f"{start_idx}, {goal_idx}")
        path = self.planner.a_star(start_idx, goal_idx)
        rospy.loginfo(path)
        
            

if __name__ == "__main__":
    rospy.init_node("world_node")

    world = World()
    print(world.grid)
    world.generate_path()
    # rospy.spin()
