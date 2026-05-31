from collections import deque

class GridSquare:
    # each grid square is 10x10cm
    DIMENSION: float = 0.5
    x: float = 0
    y: float = 0
    ph_sample: float = 0

    accessible: bool = True
    measured: bool = False
    
    bfs_searched: bool = False

    def set_square(self, x: float, y: float, ph_sample: float):
        self.x = x
        self.y = y
        self.ph_sample = ph_sample
        self.measured = True

class MapModel:
    # grid is 3x3 meters, can be configured in constructor
    size_x: float = 3.0
    size_y: float = 3.0

    map_is_complete = False

    grid: list[list[GridSquare]]

    def __init__(self, size_x: float, size_y: float):
        self.size_x = size_x
        self.size_y = size_y

        rows = int(size_y / GridSquare.DIMENSION)
        cols = int(size_x / GridSquare.DIMENSION)

        self.grid = [
            [GridSquare() for _ in range(cols)]
            for _ in range(rows)
        ]

    # Sets a ph sample at a particular location
    def set_sample(self, x: float, y: float, ph: float):
        indices = self.get_relative_square(x, y);
        row = indices[0]
        col = indices[1]
        self.grid[row][col].set_square(x, y, ph)

    def get_sample(self, x: float, y: float) -> float:
        indices = self.get_relative_square(x, y);
        row = indices[0]
        col = indices[1]
        return self.grid[row][col].ph_sample

    def mark_location_unreachable(self, x: float, y: float):
        indices = self.get_relative_square(x, y);
        row = indices[0]
        col = indices[1]
        self.grid[row][col].accessible = False

    # Returns whether a particular location already has a sample taken from it
    def location_measured(self, x: float, y: float) -> bool:
        indices = self.get_relative_square(x, y);
        row = indices[0]
        col = indices[1]
        return self.grid[row][col].measured

    # Get location that is relative to the starting point based on grid square
    def get_relative_location(self, row: int, col: int) -> tuple[float, float]:
        rows = int(self.size_y / GridSquare.DIMENSION)
        cols = int(self.size_x / GridSquare.DIMENSION)
        return ((col * GridSquare.DIMENSION + GridSquare.DIMENSION / 2) - GridSquare.DIMENSION * cols / 2,
                (row * GridSquare.DIMENSION + GridSquare.DIMENSION / 2) - GridSquare.DIMENSION * rows / 2)

    # Get square based on location that is relative to the starting point
    def get_relative_square(self, x: float, y: float) -> tuple[int, int]:
        x = x + self.size_x / 2
        y = y + self.size_y / 2
        row = int(y / GridSquare.DIMENSION)
        col = int(x / GridSquare.DIMENSION)
        return (row, col)
                

    # Returns the closest location of an unmeasured sample (Manhattan distance, not euclidean)
    def get_closest_sample_location(self, current_x: float, current_y: float) -> tuple[float, float]: 
        indices = self.get_relative_square(current_x, current_y);
        row = indices[0]
        col = indices[1]
        location = self.grid_free_bfs(row, col)
        self.reset_squares()
        # If there is no more free squares return to the strating point
        if location is None:
            return (0, 0)

        return self.get_relative_location(location[0], location[1])

    def in_bounds(self, row, col) -> bool:
        rows = int(self.size_y / GridSquare.DIMENSION)
        cols = int(self.size_x / GridSquare.DIMENSION)
        return 0 <= row < rows and 0 <= col < cols

    def grid_free_bfs(self, start_row, start_col) -> tuple[int, int] | None:
        queue = deque()
        queue.append((start_row, start_col))
        self.grid[start_row][start_col].bfs_searched = True

        while queue:
            row, col = queue.popleft()

            square = self.grid[row][col]
            if square.accessible and not square.measured:
                return (row, col)

            for dr, dc in [(-1, 0), (0, -1), (1, 0), (0, 1)]:
                nr, nc = row + dr, col + dc
                if self.in_bounds(nr, nc) and not self.grid[nr][nc].bfs_searched:
                    self.grid[nr][nc].bfs_searched = True
                    queue.append((nr, nc))

        return None

    def reset_squares(self):
        rows = int(self.size_y / GridSquare.DIMENSION)
        cols = int(self.size_x / GridSquare.DIMENSION)
        for x in range(0, cols):
            for y in range(0, rows):
                self.grid[y][x].bfs_searched = False
