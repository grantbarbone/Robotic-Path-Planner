import queue
import numpy as np
import typing 
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import seaborn as sns
import pandas as pd
from dataclasses import dataclass
import math
import time
import heapq
from collections import OrderedDict
import random
import heapq

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

class Graph_Algorithm(Node):

    def __init__(self):
        super().__init__('graph_algorithm')
        self.subscription = self.create_subscription(
            Float32,
            'topic_Main_to_Motor_Controls',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%f"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    graph_algorithm = Graph_Algorithm()

    rclpy.spin(graph_algorithm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    graph_algorithm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




class TrajectoryQueueTree():
    def __init__(self):
        self.MainBranch=[]

    def add_Branch(self, CurrentBranch, BranchNode):
        Branch=[]
        CurrentBranch[BranchNode]=Branch
    def add_Node(self,CurrentBranch, DataValue):
        if CurrentBranch==None:
            self.MainBranch.append(DataValue)
            return self.MainBranch
        CurrentBranch.append(DataValue) 
        return CurrentBranch 
    def get_Main(self):
        return self.MainBranch   
    def find_Node(self,Node):
        for index, Nodlet in enumerate(self.MainBranch,0):
            if (len(Nodlet)==2):
                if Nodlet[0]==Node[0] and Nodlet[1]==Node[1]:
                    return [index, Nodlet]
    def get_Node(self,CurrentBranch,index):
        if CurrentBranch == None:
            return self.MainBranch[index]
    
        #index = 
    def reappend_Nodes(self,Node,NewNode):
        index=self.find_Node(Node)[0]
        if (len(self.MainBranch[index])==2):
            self.MainBranch[index]=NewNode

    

class Graph_Algorithm():
    def __init__(self):
        self.GraphwidthY: int = 25
        self.GraphlengthX: int = 25
        self.ScaleCourse: int=25
        self.Scale: int =1
        self.RobotCoords: [int,int] = [5, 5]
        self.ChargingStationCoords: [int, int] = [self.GraphlengthX - 1, self.GraphwidthY - 1]
        self.Graph = np.zeros((self.GraphwidthY, self.GraphlengthX), dtype=float)
        self.AllObstacles=[]
        self.ObstaclesListLength=0
        self.PreviousLength=0
        plt.figure()
        self.heatmap = sns.heatmap(self.Graph, annot=True, cbar=False)

    def GetRobotCoords(self):
        return [self.RobotCoords[0], self.RobotCoords[1]]

    def GetChargingStationCoords(self):
        return [self.ChargingStationCoords[0], self.ChargingStationCoords[1]]

    def SetRobotCoords(self, x, y):
        self.Graph[self.RobotCoords[1], self.RobotCoords[0]] = 0
        self.RobotCoords[0]= x
        self.RobotCoords[1]= y
        self.Graph[y, x]: float = 10

    def SetChargingStation(self, x, y):
        self.ChargingStationx=x
        self.ChargingStationy=y
        self.Graph[self.ChargingStationCoords[1], self.ChargingStationCoords[0]] = 0
        self.ChargingStationCoords[0], self.ChargingStationCoords[1] = x, y
        self.Graph[y, x]: float = 9

    def SetTrajectory(self):
        self.Trajectory   =TrajectoryQueueTree()
        
        xChargingStation: float=15
        yChargingStation: float=24
        xRobot: float = 5
        yRobot: float = 5
        self.SetRobotCoords(xRobot, yRobot)
        self.update_heatmap()
        DistanceX: float= (xChargingStation+1-xRobot)
        DistanceY: float= (yChargingStation+1-yRobot)
        x: float = DistanceX/self.ScaleCourse
        y: float = DistanceY/self.ScaleCourse 
        countx: float=xRobot
        county: float=yRobot
        SectorX =0
        SectorY =0
        MainBranch: float=[]
        UpDown: bool
        RightLeft: bool

        if(yRobot>yChargingStation):
            UpDown=True 
        else:
            UpDown=False
        
        if(xRobot>xChargingStation):
            RightLeft=True
        
        else:
            RightLeft=False
        
        while (SectorY< self.ScaleCourse and SectorX < self.ScaleCourse):
            SectorY=int(county)
            SectorX=int(countx)
            
            if (self.Graph[SectorY,SectorX]!=10.0):
                self.Graph[SectorY,SectorX]=1
                DataValue=[SectorY,SectorX]
                self.Trajectory.add_Node(None, DataValue)
                
            if (self.ScaleCourse>=xChargingStation and self.ScaleCourse>=yChargingStation ):
                self.SetChargingStation( xChargingStation, yChargingStation)
                
            if (SectorX==self.GetChargingStationCoords()[0] and SectorY == self.GetChargingStationCoords()[1] ):
                break
            
            if (UpDown):
                countx-=x
            
            else:
                countx+=x
            
            if (RightLeft):
                county-=y
            else:
                county+=y

        self.update_heatmap()  

    def UpdateObstacles(self):
        objects=random.randint(1,24)
        Obstacles=[]
        for x in range(objects):
            for y in range(objects):
                Obstacles.append([random.randint(1,24),random.randint(1,24)])
        
        for Obstacle in Obstacles:
            if Obstacle not in self.AllObstacles:
                self.AllObstacles.append([Obstacle[0],Obstacle[1]]) 
                            
    def GraphObstacles(self):
        for Obstacle in self.AllObstacles:
            if self.Graph[Obstacle[0],Obstacle[1]] is not (10 or 9 or 5):
                self.Graph[Obstacle[0],Obstacle[1]]=5   
        self.update_heatmap() 

    

    def dijkstra(self, start, goal):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        rows, cols = self.Graph.shape
        distances = np.full((rows, cols), np.inf)
        distances[start[0], start[1]] = 0
        pq = [(0, start)]
        previous = {tuple(start): None}

        while pq:
            current_distance, current_node = heapq.heappop(pq)

            if current_node == goal:
                path = []
                while current_node:
                    path.append(list(current_node))
                    current_node = previous[current_node]
                path.reverse()
                return path

            if current_distance > distances[current_node[0], current_node[1]]:
                continue

            for direction in directions:
                neighbor = (current_node[0] + direction[0], current_node[1] + direction[1])
                if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                    if self.Graph[neighbor[0], neighbor[1]] == 5:  # Obstacle
                        continue
                    distance = current_distance + 1
                    if distance < distances[neighbor[0], neighbor[1]]:
                        distances[neighbor[0], neighbor[1]] = distance
                        previous[neighbor] = current_node
                        heapq.heappush(pq, (distance, neighbor))

        return None
    def a_star(self, start, goal):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        rows, cols = self.Graph.shape
        g_scores = {start: 0}
        f_scores = {start: self.heuristic(start, goal)}
        came_from = {}

        open_set = [(f_scores[start], start)]

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            for direction in directions:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                    if self.Graph[neighbor[0], neighbor[1]] == 5:  # Obstacle
                        continue
                    tentative_g_score = g_scores[current] + 1  # Assuming each step has a cost of 1

                    if tentative_g_score < g_scores.get(neighbor, math.inf):
                        came_from[neighbor] = current
                        g_scores[neighbor] = tentative_g_score
                        f_scores[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_scores[neighbor], neighbor))

        return None

    def d_star_lite(self, start, goal):
        self.s_start = start
        self.s_goal = goal
        self.k_m = 0

        self.rhs = {self.s_goal: 0}
        self.g = {self.s_goal: float('inf')}
        self.rhs[self.s_start] = float('inf')
        self.g[self.s_start] = float('inf')

        self.u = [(self.calculate_key(self.s_goal), self.s_goal)]

        self.compute_shortest_path()

        path = self.reconstruct_path()
        return path

    def calculate_key(self, s):
        return (min(self.g.get(s, float('inf')), self.rhs.get(s, float('inf'))) + self.heuristic(self.s_start, s) + self.k_m,
                min(self.g.get(s, float('inf')), self.rhs.get(s, float('inf'))))

    def update_vertex(self, s):
        if s != self.s_goal:
            self.rhs[s] = min(self.g.get((s[0] + dx, s[1] + dy), float('inf')) + 1
                              for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)])
        if s in self.u:
            self.u.remove(s)
        if self.g.get(s, float('inf')) != self.rhs.get(s, float('inf')):
            heapq.heappush(self.u, (self.calculate_key(s), s))

    def compute_shortest_path(self):
        print(self.u[0])
        print(self.calculate_key(self.s_start) )
        while self.u and (self.u[0] < self.calculate_key(self.s_start) or self.rhs[self.s_start] != self.g[self.s_start]):
            k_old = heapq.heappop(self.u)[1]
            k_new = self.calculate_key(k_old)

            if k_old < k_new:
                heapq.heappush(self.u, (k_new, k_old))
            elif self.g.get(k_old, float('inf')) > self.rhs.get(k_old, float('inf')):
                self.g[k_old] = self.rhs[k_old]
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    self.update_vertex((k_old[0] + dx, k_old[1] + dy))
            else:
                g_old = self.g[k_old]
                self.g[k_old] = float('inf')
                self.update_vertex(k_old)
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    self.update_vertex((k_old[0] + dx, k_old[1] + dy))
            if (self.calculate_key(self.s_start)==(float('inf'),float('inf'))):
                pass
    def reconstruct_path(self):
        s = self.s_start
        path = [s]
        while s != self.s_goal:
            min_rhs = float('inf')
            s_next = None
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (s[0] + dx, s[1] + dy)
                if self.g.get(neighbor, float('inf')) + 1 < min_rhs:
                    min_rhs = self.g.get(neighbor, float('inf')) + 1
                    s_next = neighbor
            if s_next is None:
                return None
            s = s_next
            path.append(s)
        return path

    def heuristic(self, current, goal):
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

    def Trajectory_Adjustment(self, algorithm="dijkstra"):
        for Obstacle in self.AllObstacles:
            if Obstacle in self.Trajectory.get_Main():
                index, TrajectorycCoords = self.Trajectory.find_Node(Obstacle)
                previousTrajectorycCoords = self.Trajectory.get_Node(None, index - 1)
                nextTrajectroyCoords = self.Trajectory.get_Node(None, index + 1)

                if algorithm == "dijkstra":
                    new_path = self.dijkstra(tuple(previousTrajectorycCoords), tuple(nextTrajectroyCoords))
                elif algorithm == "a_star":
                    new_path = self.a_star(tuple(previousTrajectorycCoords), tuple(nextTrajectroyCoords))
                elif algorithm == "d_star_lite":
                    new_path = self.d_star_lite(tuple(previousTrajectorycCoords), tuple(nextTrajectroyCoords))

                if new_path:
                    self.Trajectory.reappend_Nodes(Obstacle, new_path)
                    for Coords in new_path:
                        self.Graph[Coords[0], Coords[1]] = 1
                    self.update_heatmap()

    def SubGraph(self):
        SubLength =5
        SubWidth=5
        if (self.StereoGraph()==True and self.LidarGraph()==True):
            pass
        elif (self.StereoGraph()==True):
            SafetyDistance=2
            x,y=self.GetRobotCoords()
            direction=45
            SafetyDistanceY=round(math.sin(math.radians(direction))*SafetyDistance)
            SafetyDistanceX=round(math.cos(math.radians(direction))*SafetyDistance)
            if ((direction>45 and direction < 135) or (direction>225 and direction < 315) ):
               pass
                #SubGraph=self.Graph[x-5:x+5,y+SafetyDistanceY:y+SafetyDistanceY+10]
                #print(SubGraph,"SubGraph Y")
                #use checkbounds
            if (direction%45==0):
                self.SubGraph=self.Graph[y+SafetyDistanceY:(y+SafetyDistanceY+5),x+SafetyDistanceX:(x+SafetyDistanceX+5)]
            else:
                
                #use checkbounds
                """
                SubGraph=self.Graph[x+SafetyDistanceX:(x+SafetyDistanceX+10)- ,y-5:y+5]
                #print(SubGraph, "SubGraph X")
                print(self.Graph[x:x+5,y:+5], "Hi there")"""
            #Listen to Ros to Upload the Obstacles

        elif (self.LidarGraph()==False):
            pass
    def UpdateRobot(self):
        pass
        
    def checkBounds(self,nearX,farEndX,nearY, farEndY):
        if (nearX>self.GraphlengthX or nearY>self.GraphwidthY):
            return [None, None]
        elif (farEndX>self.GraphlengthX):
            farEndX=farEndX-self.GraphlengthX
        elif (farEndY>self.GraphwidthY):
            farEndY=farEndY-self.GraphwidthY
        return [farEndX,farEndY]

    def LidarGraph(self):
            return False
    def StereoGraph(self):
            return True

        
    def update_heatmap(self):
        # Clear the previous plot and redraw the heatmap
        plt.clf()
        self.heatmap = sns.heatmap(self.Graph, annot=True, cbar=False)
        plt.draw()
        plt.pause(0.1)  # Adjust the pause time as needed

# Create an instance of the Graph_Algorithm class
graph_instance = Graph_Algorithm()



if __name__ == '__main__':
    main()
    # Call the update_heatmap() method on the instance
    graph_instance.update_heatmap()
    #Call the SetTrajectroy to set the trajectory to x to y
    graph_instance.SetTrajectory()
    graph_instance.UpdateRobot()
    graph_instance.UpdateObstacles()
    graph_instance.GraphObstacles()
    graph_instance.Trajectory_Adjustment(algorithm="dijkstra")

plt.show()

"""while(True):
    graph_instance.UpdateRobot()
    graph_instance.UpdateObstacles()
    graph_instance.GraphObstacles()
    graph_instance.Trajectory_Adjustment()

    time.sleep(100)"""

##### TODO NEED TO FIX, distribution of random blocks must be lower to allow a path.
##### TODO NEED TO FIX, path must dispaper from old trajectory. 


