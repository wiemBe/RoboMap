import numpy as np
import heapq
import time
from collections import deque

class IndoorMap:
    def __init__(self, length=25, width=30, resolution=0.2):
        self.resolution = resolution
        self.rows = int(length / resolution)
        self.cols = int(width / resolution)
        self.matrix = np.zeros((self.rows, self.cols))
        self.matrix[0, :] = 1
        self.matrix[-1, :] = 1
        self.matrix[:, 0] = 1
        self.matrix[:, -1] = 1
        start_row = (self.rows - 15) // 2
        start_col = (self.cols - 15) // 2
        self.matrix[start_row:start_row+15, start_col:start_col+15] = 1
        self.poi_locations = {
            '1': (4, 6),
            '2': (2, 27),
            '3': (22, 6),
            '4': (22, 27)
        }
        for poi_num, location in self.poi_locations.items():
            self.matrix[location] = 2
        self.current_location = (5, 5)
        self.matrix[self.current_location] = 3
        self.length_meters = length
        self.width_meters = width

    def update_current_location(self, new_position):
        self.matrix[self.current_location] = 0
        self.current_location = new_position
        self.matrix[new_position] = 3

    def get_cell_size(self):
        return f"Each cell represents {self.resolution}x{self.resolution} meters"

    def get_coordinates(self, row, col):
        x = row * self.resolution
        y = col * self.resolution
        return (x, y)

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, pos):
        row, col = pos
        neighbors = [
            (row-1, col), (row+1, col),
            (row, col-1), (row, col+1)
        ]
        return [(r, c) for r, c in neighbors 
                if 0 <= r < self.rows and 0 <= c < self.cols 
                and self.matrix[r][c] != 1]

    def find_path(self, start, end):
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == end:
                break

            for next_pos in self.get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(end, next_pos)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        path = []
        current = end
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path