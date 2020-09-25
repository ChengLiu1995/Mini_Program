# -*- coding: utf-8 -*-
"""
Created on Wed Feb  5 18:46:06 2020

@author: Lenovo
"""

import numpy as np
class Node_Elem:
    Obstacle = False
    Visited = False
    parent = [-1,-1]
        
class A_STAR:
    moveStep = [[-1,0],[0,-1],[1,0],[0,1]]
    roadMap = []
    route = []
    def __init__(self):
        print("init")
    def loadMap(self, m):
        self.grid = np.asarray(m, dtype=int)
        for i in range(self.grid.shape[0]):
            temp = []
            for j in range(self.grid.shape[1]):
                n = Node_Elem()
                if self.grid[i][j] == 1:
                    n.Obstacle = True
                temp.append(n)
            self.roadMap.append(temp)
            
    def Heuristic(self, x, y, x1, y1):
        return abs(x - x1) + abs(y - y1)   
    
    def AddToOpen(self, x, y, g, h):
        self.openList.append([x, y, g, h])
        self.roadMap[x][y].Visited = True
        
    def CheckValidMove(self,xMove,yMove):
        #print("check")
        on_grid_x = (xMove >= 0 and xMove < self.grid.shape[0])
        on_grid_y = (yMove >= 0 and yMove < self.grid.shape[1])
        #print(on_grid_x,on_grid_y)
        if on_grid_x and on_grid_y:
            return self.roadMap[xMove][yMove].Obstacle == False and self.roadMap[xMove][yMove].Visited == False  
        return False
    
    def expendNode(self,currentNode, goal_x, goal_y):
        x = currentNode[0]
        y = currentNode[1]
        g = currentNode[2]
        for i in range(4):
            xMove = x + self.moveStep[i][0];
            yMove = y + self.moveStep[i][1];
            if self.CheckValidMove(xMove, yMove):
                g1 = g + 1
                h = self.Heuristic(x, y, goal_x, goal_y)
                self.AddToOpen(xMove, yMove, g1, h);
                self.roadMap[xMove][yMove].parent = [x, y]
                
    def cmp(self,a):
        return a[2]+a[3]
    def Search(self, start, goal):
        self.startPoint = start
        self.goalPoint = goal
        self.openList = []
        x = start[0]
        y = start[1]
        g = 0
        h = self.Heuristic(x, y, goal[0], goal[1])
        self.AddToOpen(x, y, g, h)
        while len(self.openList)>0:
            self.openList.sort(key = self.cmp, reverse = True)
            currentNode = self.openList.pop()
            if currentNode[0] == goal[0] and currentNode[1] == goal[1]:
                return
            self.expendNode(currentNode, goal[0], goal[1])
        return
    def generatePath(self):
        self.route = []
        current_x = self.goalPoint[0]
        current_y = self.goalPoint[1]
        while(self.roadMap[current_x][current_y].parent[0] != -1):
            self.route.append([current_x, current_y])
            p = self.roadMap[current_x][current_y].parent
            current_x = p[0]
            current_y = p[1]
    def drawPath(self):
        self.grid[self.startPoint[0]][self.startPoint[1]] = 3
        self.grid[self.goalPoint[0]][self.goalPoint[1]] = 4
        for i in range(len(self.route)):
            if self.grid[self.route[i][0]][self.route[i][1]] == 0:
               self.grid[self.route[i][0]][self.route[i][1]] = 2 
        for i in range(self.grid.shape[0]):
            for j in range(self.grid.shape[1]):               
                if self.grid[i][j] == 1:
                    print("#",end = " ")
                elif self.grid[i][j] == 0:
                    print("-",end = " ")
                elif self.grid[i][j] == 3:
                    print("S",end = " ")
                elif self.grid[i][j] == 4:
                    print("E",end = " ")
                elif self.grid[i][j] == 2:
                    print("x",end = " ")
            print("")

def main():
    m = [[0,1,0,0,0,0],
         [0,1,0,0,0,0],
         [0,1,0,0,0,0],
         [0,1,0,1,1,0],
         [0,0,0,0,1,0]]
    
    a = A_STAR()
    a.loadMap(m)
    start = [0,0]
    goal = [4,5]
    a.Search(start,goal)
    a.generatePath()
    a.drawPath()
#print(a.route)
if __name__ == '__main__' :
    main()

