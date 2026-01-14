import API
import sys
from collections import deque
from enum import Enum

ROW_NUM = 9
COLUMN_NUM = 9
ROW_INDEX = 0
COL_INDEX = 1

class Direction(Enum):
    NORTH = (-1, 0)
    SOUTH = (1, 0)
    EAST = (0, 1)
    WEST = (0, -1)

class Maze:
    def __init__(self, row_num, col_num):
        self._row_num = row_num
        self._col_num = col_num
        #Make lists for horizontal and vertical walls
        self.hor_walls = [[] for i in range(row_num+1)]
        self.vert_walls = [[] for i in range(row_num)]
        #Setup list for floodfill distances
        self.floodfill_distances = [[] for i in range(row_num)]
    
    def initialise_wall_variables(self, hor_walls, vert_walls):
        """Populating the wall variables with booleans of true and false.
        Note that we already know that the outer walls are filled up"""
        #Horizontal walls first
        for i in range(ROW_NUM+1):
            for j in range(ROW_NUM):
                if i==0 or i == ROW_NUM:
                    hor_walls[i].append(True)
                else:
                    hor_walls[i].append(False)
        #Vertical walls now
        for i in range(ROW_NUM):
            for j in range(ROW_NUM+1):
                if j == 0 or j == ROW_NUM:
                    vert_walls[i].append(True)
                else:
                    vert_walls[i].append(False)
        
        #Now we have walls on the boundary
    def initialise_floodfill_nums(self, floodfill_distances, row_num, col_num):
        """For the floodfill distance 2D array, only the centre goal cell is set to 0. The 
        other cells are set to false/blank for now to show that they are not updated yet"""
        for i in range(row_num):
            for j in range(col_num):
                #Showing uninitialised distances
                floodfill_distances[i].append(False)
        
        floodfill_distances[row_num//2][col_num//2] = 0


                



    


def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()

def main():
    log("Running...")
    API.setColor(0, 0, "G")
    API.setText(0, 0, "abc")
    while True:
        if not API.wallLeft():
            API.turnLeft()
        while API.wallFront():
            API.turnRight()
        API.moveForward()

if __name__ == "__main__":
    main()
