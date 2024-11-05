import pygame
from time import sleep
import numpy as np
import heapq
import requests
import json
import time

class IndoorMap:
    def __init__(self, length=25, width=30, resolution=1):
        # resolution in meters (e.g., 1 means 1 meter per cell)
        self.resolution = resolution
        
        # Calculate grid dimensions
        self.rows = int(length / resolution)  # 25 meters / resolution
        self.cols = int(width / resolution)   # 30 meters / resolution
        
        # Create binary matrix with borders
        self.matrix = np.zeros((self.rows, self.cols))
        
        # Set borders to 1
        self.matrix[0, :] = 1  # Top border
        self.matrix[-1, :] = 1  # Bottom border
        self.matrix[:, 0] = 1  # Left border
        self.matrix[:, -1] = 1  # Right border
        
        # Add 15x15 rectangle in the middle
        start_row = (self.rows - 15) // 2
        start_col = (self.cols - 15) // 2
        self.matrix[start_row:start_row+15, start_col:start_col+15] = 1
        
        # Dictionary to store POI locations and their numbers
        self.poi_locations = {
            '1': (4, 6),    # POI 1 location
            '2': (2, 27),   # POI 2 location
            '3': (22, 6),   # POI 3 location
            '4': (22, 27)   # POI 4 location
        }
        
        # Add Points of Interest with their specific locations
        for poi_num, location in self.poi_locations.items():
            self.matrix[location] = 2
        
        # Add current location (value = 3)
        self.current_location = (5, 5)  # Starting position
        self.matrix[self.current_location] = 3
        
        # Store real-world dimensions
        self.length_meters = length
        self.width_meters = width
    def update_current_location(self, new_position):
        # Clear old position
        self.matrix[self.current_location] = 0
        # Update to new position
        self.current_location = new_position
        self.matrix[new_position] = 3
    
    def get_cell_size(self):
        return f"Each cell represents {self.resolution}x{self.resolution} meters"
    
    def get_coordinates(self, row, col):
        # Convert grid coordinates to real-world meters
        x = row * self.resolution
        y = col * self.resolution
        return (x, y)
        
    def heuristic(self, a, b):
        # Manhattan distance heuristic
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def get_neighbors(self, pos):
        # Get valid neighboring cells (up, right, down, left)
        row, col = pos
        neighbors = [
            (row-1, col), (row+1, col),
            (row, col-1), (row, col+1)
        ]
        # Return only valid neighbors (within bounds and not walls)
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
        
        # Reconstruct path
        path = []
        current = end
        while current is not None:
            path.append(current)
            current = came_from.get(current)
        path.reverse()
        
        return path if path[0] == start else []

    def update_current_location(self, new_position):
        # Clear old position
        self.matrix[self.current_location] = 0
        # Update to new position
        self.current_location = new_position
        self.matrix[new_position] = 3

class IndoorMapGUI(IndoorMap):
    def __init__(self):
        super().__init__()
        pygame.init()
        self.CELL_SIZE = 20
        self.WIDTH = len(self.matrix[0]) * self.CELL_SIZE
        self.HEIGHT = len(self.matrix) * self.CELL_SIZE
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Indoor Map Navigation")
        
        # Colors
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.RED = (255, 0, 0)
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 0, 255)
        self.YELLOW = (255, 255, 0)  # Color for current location

    def draw_map(self, path=None):
        self.screen.fill(self.WHITE)
        
        for i in range(len(self.matrix)):
            for j in range(len(self.matrix[0])):
                rect = pygame.Rect(
                    j * self.CELL_SIZE, 
                    i * self.CELL_SIZE, 
                    self.CELL_SIZE, 
                    self.CELL_SIZE
                )
                
                if self.matrix[i][j] == 1:  # Wall
                    pygame.draw.rect(self.screen, self.BLACK, rect)
                elif self.matrix[i][j] == 2:  # POI
                    pygame.draw.rect(self.screen, self.GREEN, rect)
                elif self.matrix[i][j] == 3:  # Current location
                    pygame.draw.rect(self.screen, self.YELLOW, rect)
                
                pygame.draw.rect(self.screen, self.BLACK, rect, 1)

        # Draw path if exists
        if path:
            for i in range(len(path) - 1):
                start_pos = (
                    path[i][1] * self.CELL_SIZE + self.CELL_SIZE // 2,
                    path[i][0] * self.CELL_SIZE + self.CELL_SIZE // 2
                )
                end_pos = (
                    path[i + 1][1] * self.CELL_SIZE + self.CELL_SIZE // 2,
                    path[i + 1][0] * self.CELL_SIZE + self.CELL_SIZE // 2
                )
                pygame.draw.line(self.screen, self.BLUE, start_pos, end_pos, 4)

        pygame.display.flip()

    def find_nearest_poi(self, current_pos):
        pois = []
        min_distance = float('inf')
        nearest_poi = None
        
        # Find all POIs
        for i in range(self.rows):
            for j in range(self.cols):
                if self.matrix[i][j] == 2:
                    dist = self.heuristic(current_pos, (i, j))
                    if dist < min_distance:
                        min_distance = dist
                        nearest_poi = (i, j)
        
        return nearest_poi

    def fetch_pois_from_server(self):
        try:
            response = requests.get('http://localhost:8080/available')
            if response.status_code == 200:
                data = response.json()
                active_poi = str(data.get('station'))  # Convert number to string
                
                # Clear existing POIs
                self.matrix[self.matrix == 2] = 0
                
                # Add only the active POI
                if active_poi in self.poi_locations:
                    location = self.poi_locations[active_poi]
                    self.matrix[location] = 2
                    print(f"POI {active_poi} is now active at location {location}")
                    
        except requests.exceptions.RequestException as e:
            print(f"Error fetching POIs: {e}")

    def check_if_on_poi(self):
        row, col = self.current_location
        if self.matrix[row][col] == 2:
            # Find which POI number we're on
            for poi_num, location in self.poi_locations.items():
                if location == (row, col):
                    print(f"Reached POI {poi_num}!")
                    return True
        return False

    def run(self):
        last_poll_time = 0
        poll_interval = 0.5  # Poll every 0.5 seconds
        clock = pygame.time.Clock()
        current_poi = None
        
        running = True
        while running:
            current_time = time.time()
            
            # Regular polling for POI updates
            if current_time - last_poll_time >= poll_interval:
                try:
                    response = requests.get('http://localhost:8080/available')
                    if response.status_code == 200:
                        data = response.json()
                        new_poi = str(data.get('station'))
                        
                        # Only update if POI has changed and we're not currently on a POI
                        if new_poi != current_poi and not self.check_if_on_poi():
                            current_poi = new_poi
                            # Clear existing POIs
                            self.matrix[self.matrix == 2] = 0
                            # Set new POI
                            if new_poi in self.poi_locations:
                                location = self.poi_locations[new_poi]
                                self.matrix[location] = 2
                                print(f"New target POI {new_poi} at location {location}")
                except Exception as e:
                    print(f"Server polling error: {e}")
                last_poll_time = current_time
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    row, col = self.current_location
                    
                    # Handle movement
                    if event.key == pygame.K_UP and self.matrix[row-1][col] != 1:
                        self.update_current_location((row-1, col))
                        if self.check_if_on_poi():
                            current_poi = None  # Reset current POI when reached
                    elif event.key == pygame.K_DOWN and self.matrix[row+1][col] != 1:
                        self.update_current_location((row+1, col))
                        if self.check_if_on_poi():
                            current_poi = None  # Reset current POI when reached
                    elif event.key == pygame.K_LEFT and self.matrix[row][col-1] != 1:
                        self.update_current_location((row, col-1))
                        if self.check_if_on_poi():
                            current_poi = None  # Reset current POI when reached
                    elif event.key == pygame.K_RIGHT and self.matrix[row][col+1] != 1:
                        self.update_current_location((row, col+1))
                        if self.check_if_on_poi():
                            current_poi = None  # Reset current POI when reached
            
            # Calculate path only when needed
            if self.matrix[self.matrix == 2].any():  # Check if any POI exists
                nearest_poi = self.find_nearest_poi(self.current_location)
                if nearest_poi:
                    path = self.find_path(self.current_location, nearest_poi)
                else:
                    path = []
            else:
                path = []
                
            self.draw_map(path)
            pygame.display.flip()
            clock.tick(30)  # Limit to 30 FPS

        pygame.quit()

if __name__ == "__main__":
    gui = IndoorMapGUI()
    gui.run()