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

    def turn_around(self) -> None:
        self._direction = self._direction.behind

    def move_forward(self) -> None:
        self._position = step(self._position, self._direction)


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

    def is_wall(
        self, position: Position, direction: Direction, assume_wall=False
    ) -> bool:
        if self.within_bounds(position):
            walls, x, y = self.get_wall(position, direction)
            if walls[x][y] == WallState.UNKNOWN:
                return assume_wall
            return walls[x][y] == WallState.WALL
        return False

    def floodfill(self, goal: Position, require_valid_path=False) -> None:
        self._dists = [[-1] * self._height for _ in range(self._width)]
        self.set_dist(goal, 0)
        q = deque([goal])
        while q:
            position = q.popleft()
            for direction in Direction:
                neighbour = step(position, direction)
                if (
                    self.within_bounds(neighbour)
                    and not self.is_wall(
                        position, direction, assume_wall=require_valid_path
                    )
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

    front_wall_state = WallState.WALL if API.wallFront() else WallState.EMPTY
    left_wall_state = WallState.WALL if API.wallLeft() else WallState.EMPTY
    right_wall_state = WallState.WALL if API.wallRight() else WallState.EMPTY

    updated |= maze.update_wall(position, direction, front_wall_state)
    updated |= maze.update_wall(position, direction.left, left_wall_state)
    updated |= maze.update_wall(position, direction.right, right_wall_state)
    return updated


def extract_path(maze: Maze, mouse: Mouse, require_valid_path=True) -> list[Direction]:
    """Extract the shortest path as a list of directions from start to goal"""
    maze.floodfill(maze.goal, require_valid_path)

    path = []
    position = mouse.start_position
    direction = mouse.start_direction

    while position != maze.goal:
        next_dir = maze.next_direction(position, direction)
        path.append(next_dir)

        position = (position[0] + next_dir.dx, position[1] + next_dir.dy)
        direction = next_dir

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


def move_forward(mouse: Mouse):
    mouse.move_forward()
    API.moveForward()


def turn_right(mouse: Mouse):
    mouse.turn_right()
    API.turnRight()


def turn_around(mouse: Mouse):
    mouse.turn_around()
    API.turnRight()
    API.turnRight()


def turn_left(mouse: Mouse):
    mouse.turn_left()
    API.turnLeft()


def turn_to_face(mouse: Mouse, direction: Direction) -> None:
    if direction == mouse.direction.right:
        mouse.turn_right()
        API.turnRight()
    elif direction == mouse.direction.behind:
        mouse.turn_around()
        API.turnRight()
        API.turnRight()
    elif direction == mouse.direction.left:
        mouse.turn_left()
        API.turnLeft()


def move_mouse(mouse: Mouse, move: Move) -> None:
    if move == Move.FORWARD:
        move_forward(mouse)
    elif move == Move.TURN_RIGHT:
        turn_right(mouse)
    elif move == Move.TURN_AROUND:
        turn_around(mouse)
    elif move == Move.TURN_LEFT:
        turn_left(mouse)


def search_to(maze: Maze, mouse: Mouse, goal: Position):
    """Move to the target position at a safe speed while mapping the maze."""

    while mouse.position != goal:
        update_walls(maze, mouse)
        maze.floodfill(goal)

        # display walls and distances for debugging
        display_walls(maze, mouse)
        display_dists(maze)

        # determine next move
        target_dir = maze.next_direction(mouse.position, mouse.direction)
        move = next_move(mouse.direction, target_dir)

        # move the mouse
        move_mouse(mouse, move)


def search_maze(maze: Maze, mouse: Mouse):
    """Search to the goal, then search back to the start.

    Assumes that the mouse is at the start position and orientation.
    """
    search_to(maze, mouse, maze.goal)
    search_to(maze, mouse, mouse.start_position)
    turn_to_face(mouse, mouse.start_direction)


def speed_run(moves: list[tuple[Move, int]]):
    pass


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

    while True:
        search_to(maze, mouse, maze.goal)

        path = extract_path(maze, mouse, require_valid_path=True)
        if extract_path(maze, mouse, require_valid_path=False) == path:
            log("optimal path found")
            break
        else:
            log("non-optimal path found")

        search_to(maze, mouse, mouse.start_position)
        turn_to_face(mouse, mouse.start_direction)

        path = extract_path(maze, mouse, require_valid_path=True)
        if extract_path(maze, mouse, require_valid_path=False) == path:
            log("optimal path found")
            break
        else:
            log("non-optimal path found")

    # fast run
    API.ackReset()
    mouse.reset()

    time.sleep(1)
    log("starting fast run")
    moves = path_to_moves(compress_path(path), HEADING)
    for move, count in moves:
        for _ in range(count):
            API.setColor(*mouse.position, "G")
            move_mouse(mouse, move)


if __name__ == "__main__":
    main()
