from collections.abc import Iterable
from heapq import heappop, heappush
import operator

Cell = tuple[int, int]


class NoPathError(RuntimeError):
    pass


def _as_cell(value: object, label: str) -> Cell:
    """Normalise a cell to a plain pair of ints, or refuse it.

    Anything that is not exactly two integers can never compare equal to a
    neighbour, so an unchecked one is silently ignored rather than rejected:
    an obstacle at `(1.5, 1)` or `(1, 1, 1)` is accepted as in bounds and
    then planned straight through. A planner that quietly drops an obstacle
    it was told about is worse than one that refuses the input.
    """
    try:
        x, y = value  # type: ignore[misc]
        return (operator.index(x), operator.index(y))
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} must be a pair of integers: {value!r}") from error


class GridPlanner:
    """Small deterministic planner for simulator and integration tests."""

    def __init__(self, width: int, height: int, obstacles: Iterable[Cell] = ()):
        try:
            width = operator.index(width)
            height = operator.index(height)
        except TypeError as error:
            raise ValueError("grid dimensions must be integers") from error
        if width <= 0 or height <= 0:
            raise ValueError("grid dimensions must be positive")
        self.width = width
        self.height = height
        self._obstacles: set[Cell] = set()
        self.set_obstacles(obstacles)

    def set_obstacles(self, obstacles: Iterable[Cell]) -> None:
        candidate = {_as_cell(cell, "obstacle") for cell in obstacles}
        # Validated before it is stored, so a bad update leaves the previous
        # obstacles in place rather than half-applying.
        if any(not self._in_bounds(cell) for cell in candidate):
            raise ValueError("obstacles must be inside the grid")
        self._obstacles = candidate

    def plan(self, start: Cell, goal: Cell) -> list[Cell]:
        start = _as_cell(start, "start")
        goal = _as_cell(goal, "goal")
        self._validate_free_cell(start, "start")
        self._validate_free_cell(goal, "goal")
        frontier: list[tuple[int, Cell]] = [(0, start)]
        came_from: dict[Cell, Cell | None] = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            _, current = heappop(frontier)
            if current == goal:
                return self._reconstruct_path(came_from, current)

            for neighbor in self._neighbors(current):
                new_cost = cost_so_far[current] + 1
                if new_cost < cost_so_far.get(neighbor, float("inf")):
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self._heuristic(neighbor, goal)
                    heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current

        raise NoPathError(f"no path from {start} to {goal}")

    def _neighbors(self, cell: Cell) -> Iterable[Cell]:
        x, y = cell
        for neighbor in ((x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1)):
            if self._in_bounds(neighbor) and neighbor not in self._obstacles:
                yield neighbor

    def _validate_free_cell(self, cell: Cell, label: str) -> None:
        if not self._in_bounds(cell):
            raise ValueError(f"{label} is outside the grid")
        if cell in self._obstacles:
            raise ValueError(f"{label} is occupied")

    def _in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    @staticmethod
    def _heuristic(first: Cell, second: Cell) -> int:
        return abs(first[0] - second[0]) + abs(first[1] - second[1])

    @staticmethod
    def _reconstruct_path(
        came_from: dict[Cell, Cell | None], current: Cell
    ) -> list[Cell]:
        path = [current]
        parent = came_from[current]
        while parent is not None:
            current = parent
            path.append(current)
            parent = came_from[current]
        path.reverse()
        return path
