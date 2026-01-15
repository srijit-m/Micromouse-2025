import API
import sys
from collections import deque
from enum import Enum

ROW_NUM = 9
COLUMN_NUM = 9

ROW_INDEX = 0
COLUMN_INDEX = 1

#For checking floodfill points
#NORTH_INDEX = 0
#EAST_INDEX = 1
#SOUTH_INDEX = 2
#WEST_INDEX = 3

class Direction(Enum):
    NORTH = (-1, 0)
    SOUTH = (1, 0)
    EAST = (0, 1)
    WEST = (0, -1)

def get_neighbour_coords(row, col):
        """This function returns the north, east, south and west coordinates of the points relative
        to a point popped out of the queue. For neighbour points that are out of bounds, they will be set to none
        Returns: A list of 4 tuples of the form (row, col)"""
        neighbour_points = []
        neighbour_points.append((row+Direction.NORTH.value[0], col+Direction.NORTH.value[1]))
        neighbour_points.append((row+Direction.EAST.value[0], col+Direction.EAST.value[1]))
        neighbour_points.append((row+Direction.SOUTH.value[0], col+Direction.SOUTH.value[1]))
        neighbour_points.append((row+Direction.WEST.value[0], col+Direction.WEST.value[1]))
        for i in range(4):
            if neighbour_points[i][0] < 0 or neighbour_points[i][0] > ROW_NUM-1:
                neighbour_points[i] = None
            elif neighbour_points[i][1] < 0 or neighbour_points[i][1] > COLUMN_NUM-1:
                neighbour_points[i] = None
        return neighbour_points

def check_bounds(row, col, direction):
    """Checks if a neighbouring cell is within the bounds of the maze"""
    (new_row, new_col) = (row+direction.value[ROW_INDEX], col+direction.value[COLUMN_INDEX])
    if new_row < 0 or new_row > ROW_NUM-1:
        return False
    if new_col < 0 or new_col > COLUMN_NUM-1:
        return False  
    return (new_row, new_col)

def can_move(row, col, direction, hor_walls, vert_walls):
    """Checks if the neighbouring cell can be accessed (i.e. there are no walls)"""
    if (direction == Direction.NORTH):
        #For North, we are looking at a horizontal wall
        if (hor_walls[row][col] == False):
            #No walls in the way
            return True
        return False
    if (direction == Direction.EAST):
        #For east, we deal with vertical walls
        if (vert_walls[row][col+1] == False):
            return True
        return False
    if (direction == Direction.SOUTH):
        #For south we deal with a horizontal wall
        if (hor_walls[row+1][col] == False):
            return True
        return False
    if (direction == Direction.WEST):
        #For west we deal with a vertical wall
        if (vert_walls[row][col] == False):
            return True
        return False


class Maze:
    def __init__(self, row_num, col_num):
        self._row_num = row_num
        self._col_num = col_num
        #Make lists for horizontal and vertical walls
        self.hor_walls = [[] for i in range(row_num+1)]
        self.vert_walls = [[] for i in range(row_num)]
        #Setup list for floodfill distances
        self.floodfill_distances = [[] for i in range(row_num)]
        #Initialising an empty queue for the floodfill algorithm
        self.dq = deque()
    
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
                floodfill_distances[i].append(-1)
        
        floodfill_distances[row_num//2][col_num//2] = 0
    def reset_floodfill_distances(self, floodfill_distances, row_num, col_num):
        """Here we want to reset the floodfill algorithm every time we hit a wall. 
        For an already intialised array, we don't need to append elements so we can just
        set the array values."""
        for i in range(row_num):
            for j in range(col_num):
                #Showing uninitialised distances
                floodfill_distances[i][j] = -1    
        floodfill_distances[row_num//2][col_num//2] = 0
      
    def calculate_floodfill_distances(self, row_num, col_num, dq, floodfill_distances, hor_walls, vert_walls):
        #Add the goal cell to the queue
        centre_point = (row_num//2, col_num//2)
        #print(f"Row: {centre_point[0]}, Column: {centre_point[1]}")
        dq.append(centre_point)
        while dq:
            #Carry out the floodfill algorithm
            #Pop the zero cell and save its row and column
            (row, col) = dq.popleft()
            #Now check the boxes north, east, south and west of the point
            #Note that in this for loop False + 1 = 1 (cool thing in python)
            for direction in [Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST]:
                #First check if the cell is unvisited and valid
                bounds_result = check_bounds(row, col, direction)
                if bounds_result == False:
                    continue
                elif floodfill_distances[bounds_result[ROW_INDEX]][bounds_result[COLUMN_INDEX]] != -1:
                    #This cell has already been visited
                    continue
                else:
                    #Now this is a valid cell, we now need to check if the cell is accessible
                    #Here use a can_move function that takes the current cell and the direction
                    if can_move(row, col, direction, hor_walls, vert_walls):
                        #Here, if the function returns True, we can update floodfill distances
                        #n_row stands for neighbour row, n_col stands for neighbour col
                        (n_row, n_col) = (row+direction.value[ROW_INDEX], col+direction.value[COLUMN_INDEX])
                        floodfill_distances[n_row][n_col] = floodfill_distances[row][col]+1
                        dq.append((n_row, n_col))

                
        
                



def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()

def main():
    log("Running...")
    API.setColor(0, 0, "G")
    API.setText(0, 0, "abc")

    #Initialise details of the maze
    maze = Maze(ROW_NUM, COLUMN_NUM)
    maze.initialise_wall_variables(maze.hor_walls, maze.vert_walls)
    maze.initialise_floodfill_nums(maze.floodfill_distances, maze._row_num, maze._col_num)
    maze.calculate_floodfill_distances(maze._row_num, maze._col_num, maze.dq, maze.floodfill_distances, maze.hor_walls, maze.vert_walls)
    
    for r in range(ROW_NUM):
        for c in range(COLUMN_NUM):
            val = maze.floodfill_distances[r][c]
            # Convert -1 (unvisited) to blank
            if val == -1:
                display_val = ""
            else:
                display_val = str(val)
            API.setText(r, c, display_val)

    #Wall follower algorithm    
    while True:
        if not API.wallLeft():
            API.turnLeft()
        while API.wallFront():
            API.turnRight()
        API.moveForward()

if __name__ == "__main__":
    main()
