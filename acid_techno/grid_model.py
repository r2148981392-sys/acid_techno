from collections import deque

class AcidityGridSquare:
    # each measure grid square is 5x5cm
    DIMENSION: float = 0.05
    x: float = 0
    y: float = 0
    ph_sample: float = 0

    measured: bool = False

    def set_square(self, x: float, y: float, ph_sample: float):
        self.x = x
        self.y = y
        self.ph_sample = ph_sample
        self.measured = True

class NavGridSquare:
    # each navigatable grid square is 50x50cm
    DIMENSION: float = 0.5
    accessible: bool = True
    reached: bool = False
    bfs_searched: bool = False
    measured: bool = False

    def set_reached(self):
        self.reached = True
        self.measured = True

class MapModel:
    # grid is 3x3 meters, can be configured in constructor
    size_x: float = 3.0
    size_y: float = 3.0

    map_is_complete = False

    nav_grid: list[list[NavGridSquare]]
    acidity_grid: list[list[AcidityGridSquare]]

    def __init__(self, size_x: float, size_y: float):
        self.size_x = size_x
        self.size_y = size_y

        rows = int(size_y / NavGridSquare.DIMENSION)
        cols = int(size_x / NavGridSquare.DIMENSION)

        self.nav_grid = [
            [NavGridSquare() for _ in range(cols)]
            for _ in range(rows)
        ]

        rows = int(size_y / AcidityGridSquare.DIMENSION)
        cols = int(size_x / AcidityGridSquare.DIMENSION)

        self.acidity_grid = [
            [AcidityGridSquare() for _ in range(cols)]
            for _ in range(rows)
        ]

    # Sets a ph sample at a particular location
    def set_sample(self, x: float, y: float, ph: float):
        indices = self.get_relative_acidity_square(x, y);
        row = indices[0]
        col = indices[1]
        self.acidity_grid[row][col].set_square(x, y, ph)

    def set_reached(self, x: float, y: float):
        indices = self.get_relative_nav_square(x, y)
        row = indices[0]
        col = indices[1]
        self.nav_grid[row][col].set_reached()

    def get_sample(self, x: float, y: float) -> float:
        indices = self.get_relative_acidity_square(x, y);
        row = indices[0]
        col = indices[1]
        return self.acidity_grid[row][col].ph_sample

    def mark_location_unreachable(self, x: float, y: float):
        indices = self.get_relative_nav_square(x, y);
        row = indices[0]
        col = indices[1]
        if self.in_nav_bounds(row, col):
            self.nav_grid[row][col].accessible = False

    # Returns whether a particular location already has a sample taken from it
    def location_measured(self, x: float, y: float) -> bool:
        indices = self.get_relative_acidity_square(x, y);
        row = indices[0]
        col = indices[1]
        if self.in_acidity_bounds(row, col):
            return self.acidity_grid[row][col].measured
        return True

    # Get location that is relative to the starting point based on nav grid square
    def get_relative_nav_location(self, row: int, col: int) -> tuple[float, float]:
        rows = int(self.size_y / NavGridSquare.DIMENSION)
        cols = int(self.size_x / NavGridSquare.DIMENSION)
        return ((col * NavGridSquare.DIMENSION + NavGridSquare.DIMENSION / 2) - NavGridSquare.DIMENSION * cols / 2,
                (row * NavGridSquare.DIMENSION + NavGridSquare.DIMENSION / 2) - NavGridSquare.DIMENSION * rows / 2)

    # Get navigation grid square based on location that is relative to the starting point
    def get_relative_nav_square(self, x: float, y: float) -> tuple[int, int]:
        x = x + self.size_x / 2
        y = y + self.size_y / 2
        row = int(y / NavGridSquare.DIMENSION)
        col = int(x / NavGridSquare.DIMENSION)
        return (row, col)
                
    # Get acidity square based on location that is relative to the starting point
    def get_relative_acidity_square(self, x: float, y: float) -> tuple[int, int]:
        x = x + self.size_x / 2
        y = y + self.size_y / 2
        row = int(y / AcidityGridSquare.DIMENSION)
        col = int(x / AcidityGridSquare.DIMENSION)
        return (row, col)

    # Returns the closest location of an unmeasured sample (Manhattan distance, not euclidean)
    def get_closest_sample_location(self, current_x: float, current_y: float) -> tuple[float, float]: 
        indices = self.get_relative_nav_square(current_x, current_y);
        row = indices[0]
        col = indices[1]
        location = self.grid_free_bfs(row, col)
        self.reset_squares()
        # If there is no more free squares return to the strating point
        if location is None:
            return (0, 0)

        return self.get_relative_nav_location(location[0], location[1])

    def in_nav_bounds(self, row, col) -> bool:
        rows = int(self.size_y / NavGridSquare.DIMENSION)
        cols = int(self.size_x / NavGridSquare.DIMENSION)
        return 0 <= row < rows and 0 <= col < cols

    def in_acidity_bounds(self, row, col) -> bool:
        rows = int(self.size_y / AcidityGridSquare.DIMENSION)
        cols = int(self.size_x / AcidityGridSquare.DIMENSION)
        return 0 <= row < rows and 0 <= col < cols

    def grid_free_bfs(self, start_row, start_col) -> tuple[int, int] | None:
        queue = deque()
        if self.in_nav_bounds(start_row, start_col):
            queue.append((start_row, start_col))
            self.nav_grid[start_row][start_col].bfs_searched = True
        else:
            loc = self.get_unexplored_square()
            if loc is None:
                return None
            queue.append((loc[0], loc[1]))
            self.nav_grid[loc[0]][loc[1]].bfs_searched = True

        while queue:
            row, col = queue.popleft()

            square = self.nav_grid[row][col]
            if square.accessible and not square.reached:
                return (row, col)

            for dr, dc in [(-1, 0), (0, -1), (1, 0), (0, 1)]:
                nr, nc = row + dr, col + dc
                if self.in_nav_bounds(nr, nc) and not self.nav_grid[nr][nc].bfs_searched:
                    self.nav_grid[nr][nc].bfs_searched = True
                    queue.append((nr, nc))

        return None

    def reset_squares(self):
        rows = int(self.size_y / NavGridSquare.DIMENSION)
        cols = int(self.size_x / NavGridSquare.DIMENSION)
        for x in range(0, cols):
            for y in range(0, rows):
                self.nav_grid[y][x].bfs_searched = False

    def get_unexplored_square(self) -> tuple[int, int] | None:
        rows = int(self.size_y / NavGridSquare.DIMENSION)
        cols = int(self.size_x / NavGridSquare.DIMENSION)
        for x in range(0, cols):
            for y in range(0, rows):
                if not self.nav_grid[y][x].reached and self.nav_grid[y][x].accessible:
                    return (y, x)
        return None
