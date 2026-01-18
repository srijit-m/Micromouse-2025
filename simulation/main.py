import API
import sys
from collections import deque
from enum import Enum, IntEnum
import time

Position = tuple[int, int]


class WallState(IntEnum):
    EMPTY = 0
    WALL = 1
    UNKNOWN = 2


class Direction(Enum):
    # direction deltas (dx, dy)
    NORTH = (0, 1)
    EAST = (1, 0)
    SOUTH = (0, -1)
    WEST = (-1, 0)

    @property
    def dx(self) -> int:
        return self.value[0]

    @property
    def dy(self) -> int:
        return self.value[1]

    @property
    def left(self):
        return Direction((-self.dy, self.dx))

    @property
    def right(self):
        return Direction((self.dy, -self.dx))

    @property
    def behind(self):
        return Direction((-self.dx, -self.dy))


class Move(Enum):
    FORWARD = 0
    TURN_LEFT = 1
    TURN_RIGHT = 2
    TURN_AROUND = 3


class Mouse:

    def __init__(self, start_pos: Position, start_dir: Direction) -> None:
        self._start_pos = start_pos
        self._start_dir = start_dir
        self._position = start_pos
        self._direction = start_dir

    @property
    def start_position(self) -> Position:
        return self._start_pos

    @property
    def start_direction(self) -> Direction:
        return self._start_dir

    @property
    def position(self) -> Position:
        return self._position

    @property
    def direction(self) -> Direction:
        return self._direction

    def reset(self) -> None:
        self._position = self._start_pos
        self._direction = self._start_dir

    def turn_left(self) -> None:
        self._direction = self._direction.left

    def turn_right(self) -> None:
        self._direction = self._direction.right

    def move_forward(self) -> None:
        self._position = step(self._position, self._direction)

    def apply_move(self, move: Move) -> None:
        if move == Move.FORWARD:
            self.move_forward()
        elif move == Move.TURN_LEFT:
            self.turn_left()
        elif move == Move.TURN_RIGHT:
            self.turn_right()
        elif move == Move.TURN_AROUND:
            self.turn_right()
            self.turn_right()


class Maze:
    def __init__(self, width: int, height: int) -> None:
        self._width = width
        self._height = height
        self._goal = (width // 2, height // 2)  # assume odd maze size for simplicity

        # initialise empty walls and distances
        self._h_walls = [[WallState.UNKNOWN] * (height + 1) for _ in range(width)]
        self._v_walls = [[WallState.UNKNOWN] * height for _ in range(width + 1)]
        self._dists = [[-1] * height for _ in range(width)]

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    @property
    def goal(self) -> Position:
        return self._goal

    @goal.setter
    def goal(self, position: Position) -> None:
        self._goal = position

    @property
    def dists(self) -> list[list[int]]:
        return self._dists

    @dists.setter
    def dists(self, dists) -> None:
        self._dists = dists

    def get_dist(self, position: Position) -> int:
        return self._dists[position[0]][position[1]]

    def set_dist(self, position: Position, dist: int) -> None:
        self._dists[position[0]][position[1]] = dist

    def get_wall(
        self, position: Position, direction: Direction
    ) -> tuple[list[list[WallState]], int, int]:
        """Returns the appropriate wall matrix and the indices for the wall
        adjacent to the specified position

        Returns:
            A tuple (walls, x_idx, y_idx), where the wall is accessed by
            walls[x_idx][y_idx]
        """
        x, y = position
        if direction == Direction.NORTH:
            return self._h_walls, x, y + 1
        if direction == Direction.EAST:
            return self._v_walls, x + 1, y
        if direction == Direction.SOUTH:
            return self._h_walls, x, y
        if direction == Direction.WEST:
            return self._v_walls, x, y
        return None

    def within_bounds(self, position: Position) -> bool:
        """Return whether the position is within the bounds of the maze"""
        x, y = position
        return 0 <= x < self.width and 0 <= y < self.height

    def update_wall(
        self, position: Position, direction: Direction, state: WallState
    ) -> bool:
        """Update the state of a wall if it is unknown and return whether the wall state changed."""
        if not self.within_bounds(position):
            return False

        walls, x, y = self.get_wall(position, direction)

        if walls[x][y] != WallState.UNKNOWN:
            return False

        walls[x][y] = state
        return True

    def is_wall(self, position: Position, direction: Direction) -> bool:
        if self.within_bounds(position):
            walls, x, y = self.get_wall(position, direction)
            return walls[x][y] == WallState.WALL
        return False

    def floodfill(self) -> None:
        self._dists = [[-1] * self._height for _ in range(self._width)]
        self.set_dist(self._goal, 0)
        q = deque([self._goal])
        while q:
            position = q.popleft()
            for direction in Direction:
                neighbour = step(position, direction)
                if (
                    self.within_bounds(neighbour)
                    and not self.is_wall(position, direction)
                    and self.get_dist(neighbour) == -1
                ):
                    self.set_dist(neighbour, self.get_dist(position) + 1)
                    q.append(neighbour)

    def next_direction(self, position: Position, heading: Direction) -> Direction:
        """Returns the direction to move, based on the current position and direction.

        When multiple neighbouring cells have the same distance from the goal,
        the order of precedence is: forward, left, right, backward.
        """
        directions = (
            heading,
            heading.left,
            heading.right,
            heading.behind,
        )

        x, y = position
        current_dist = self.dists[x][y]

        for direction in directions:
            neighbour = step(position, direction)

            if not self.within_bounds(neighbour) or self.is_wall(position, direction):
                continue

            if self.get_dist(neighbour) == current_dist - 1:
                return direction

        return heading


def step(pos: Position, dir: Direction) -> Position:
    return pos[0] + dir.dx, pos[1] + dir.dy


def next_move(direction: Direction, target: Direction) -> Move:
    """Returns the required move to make based on the current direction and
    the target direction"""
    # for simplicity, take advantage of the fact that the Direction enum is ordered clockwise
    dirs = list(Direction)
    offset = (dirs.index(target) - dirs.index(direction)) % len(dirs)

    if offset == 0:
        return Move.FORWARD
    if offset == 1:
        return Move.TURN_RIGHT
    if offset == 2:
        return Move.TURN_AROUND
    return Move.TURN_LEFT  # offset == 3


def update_walls(maze: Maze, mouse: Mouse) -> bool:
    """Update the known walls in the maze at the mouse's current position.

    Returns:
        Whether any new walls were added.
    """
    position = mouse.position
    direction = mouse.direction
    updated = False
    if API.wallFront():
        updated |= maze.update_wall(position, direction, WallState.WALL)
    else:
        updated |= maze.update_wall(position, direction, WallState.EMPTY)
    if API.wallLeft():
        updated |= maze.update_wall(position, direction.left, WallState.WALL)
    else:
        updated |= maze.update_wall(position, direction.left, WallState.EMPTY)
    if API.wallRight():
        updated |= maze.update_wall(position, direction.right, WallState.WALL)
    else:
        updated |= maze.update_wall(position, direction.right, WallState.EMPTY)
    return updated


def extract_path(
    maze: Maze, start_pos: Position, start_dir: Direction, goal: Position
) -> list[Direction]:
    """Extract the shortest path as a list of directions from start to goal"""
    old_goal = maze.goal
    old_dists = maze.dists
    if old_goal != goal:
        maze.goal = goal
        maze.floodfill()

    path = []
    position = start_pos
    direction = start_dir

    while position != maze.goal:
        next_dir = maze.next_direction(position, direction)
        path.append(next_dir)

        position = (position[0] + next_dir.dx, position[1] + next_dir.dy)
        direction = next_dir

    # restore old goal and distances
    if old_goal != goal:
        maze.goal = old_goal
        maze.dists = old_dists

    return path


def compress_path(path: list[Direction]) -> list[tuple[Direction, int]]:
    """Compress repeated directions in a path into (direction, count) tuples"""
    compressed = []
    last_dir = path[0]
    count = 1

    for direction in path[1:]:
        if direction == last_dir:
            count += 1
        else:
            compressed.append((last_dir, count))
            last_dir = direction
            count = 1

    compressed.append((last_dir, count))
    return compressed


def path_to_moves(
    path: list[tuple[Direction, int]], start_dir: Direction
) -> list[tuple[Move, int]]:
    moves = []
    prev_dir = start_dir
    for direction, count in path:
        move = next_move(prev_dir, direction)
        if move != Move.FORWARD:
            moves.append((move, 1))
        moves.append((Move.FORWARD, count))
        prev_dir = direction
    return moves

def display_walls(maze: Maze, mouse: Mouse) -> None:
    """Display the walls at the mouse's current position"""
    for direction in Direction:
        if maze.is_wall(mouse.position, direction):
            x, y = mouse.position
            if direction == Direction.NORTH:
                API.setWall(x, y, "n")
            elif direction == Direction.EAST:
                API.setWall(x, y, "e")
            elif direction == Direction.SOUTH:
                API.setWall(x, y, "s")
            elif direction == Direction.WEST:
                API.setWall(x, y, "w")


def display_dists(maze: Maze) -> None:
    for x in range(maze.width):
        for y in range(maze.height):
            dist = maze.get_dist((x, y))
            if dist != -1:
                API.setText(x, y, dist)


def explore_maze(maze: Maze, mouse: Mouse) -> list[tuple[Move, int]]:
    """Explore the maze and return optimal list of moves to solve maze"""
    centre = maze.width // 2, maze.height // 2
    last_path = None
    while True:
        if mouse.position == maze.goal:
            if maze.goal == mouse.start_position:
                log("Start reached")
                new_goal = centre
            else:
                log("Centre reached")
                new_goal = mouse.start_position

            path = extract_path(
                maze, mouse.start_position, mouse.start_direction, centre
            )

            if path == last_path:  # path converged
                return path_to_moves(compress_path(path), mouse.start_direction)

            last_path = path
            maze.goal = new_goal
            maze.floodfill()

        # update walls and distances
        if update_walls(maze, mouse):
            maze.floodfill()  # only recalculate if walls changed

        # display walls and distances for debugging
        display_walls(maze, mouse)
        display_dists(maze)

        # determine next move
        target_dir = maze.next_direction(mouse.position, mouse.direction)
        move = next_move(mouse.direction, target_dir)

        # update internal mouse state
        mouse.apply_move(move)

        # move the actual mouse
        execute_move(move)


def execute_move(move: Move) -> None:
    if move == Move.FORWARD:
        API.moveForward()
    elif move == Move.TURN_RIGHT:
        API.turnRight()
    elif move == Move.TURN_AROUND:
        API.turnRight()
        API.turnRight()
    elif move == Move.TURN_LEFT:
        API.turnLeft()


def log(string) -> None:
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()


WIDTH = API.mazeWidth()
HEIGHT = API.mazeHeight()
START = (0, 0)
HEADING = Direction.NORTH

def main():
    maze = Maze(WIDTH, HEIGHT)
    mouse = Mouse(START, HEADING)

    moves = explore_maze(maze, mouse)

    # fast run
    API.ackReset()
    mouse.reset()

    time.sleep(1)
    log("starting fast run")
    for move, count in moves:
        for _ in range(count):
            API.setColor(*mouse.position, "G")
            mouse.apply_move(move)
            execute_move(move)


if __name__ == "__main__":
    main()
