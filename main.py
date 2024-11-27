import pygame
from time import sleep
import numpy as np
import heapq
import requests
import json
import time
import serial
import serial.tools.list_ports

class IndoorMap:
    def __init__(self, length=25, width=30, resolution=0.2):
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
        self.CELL_SIZE = 10
        self.WIDTH = len(self.matrix[0]) * self.CELL_SIZE
        self.HEIGHT = len(self.matrix) * self.CELL_SIZE
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Indoor Map Navigation")
        
        # Colors
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.YELLOW = (255, 255, 0)
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 0, 255)
        
        # Movement control flags
        self.autonomous_mode = False
        self.current_path = []
        self.path_index = 0
        
        # Arduino setup
        self.arduino = None
        self.connect_arduino()

    def connect_arduino(self):
        """Attempt to connect to Arduino"""
        try:
            # Directly connect to COM6
            self.arduino = serial.Serial('COM6', 9600, timeout=1)
            print("Connected to Arduino on COM6")
        except Exception as e:
            print(f"Error connecting to Arduino: {e}")
    
    def send_movement_command(self, direction):
        """Send movement command to Arduino"""
        if not self.arduino:
            return
            
        try:
            # Define command protocol (adjust based on your Arduino code)
            commands = {
                'UP': 'F',    # Forward
                'DOWN': 'B',  # Backward
                'LEFT': 'L',  # Left
                'RIGHT': 'R', # Right
                'STOP': 'S'   # Stop
            }
            
            if direction in commands:
                self.arduino.write(commands[direction].encode())
                # Wait for acknowledgment if needed
                # response = self.arduino.readline().decode().strip()
        except Exception as e:
            print(f"Error sending command to Arduino: {e}")

    def calculate_direction(self, current_pos, next_pos):
        """Calculate direction based on current and next position"""
        curr_row, curr_col = current_pos
        next_row, next_col = next_pos
        
        if next_row < curr_row:
            return 'UP'
        elif next_row > curr_row:
            return 'DOWN'
        elif next_col < curr_col:
            return 'LEFT'
        elif next_col > curr_col:
            return 'RIGHT'
        return 'STOP'

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

    def get_cell_from_mouse(self, mouse_pos):
        """Convert mouse coordinates to grid cell coordinates"""
        x, y = mouse_pos
        row = y // self.CELL_SIZE
        col = x // self.CELL_SIZE
        
        # Ensure coordinates are within bounds
        row = max(0, min(row, self.rows - 1))
        col = max(0, min(col, self.cols - 1))
        
        return (row, col)

    def move_along_path(self):
        """Move one step along the current path based on the mouse position"""
        if not self.current_path or self.path_index >= len(self.current_path) - 1:
            self.autonomous_mode = False
            if self.arduino:
                self.send_movement_command('STOP')
            print("Path completed or no valid path.")
            return False

        next_pos = self.current_path[self.path_index + 1]

        # Check if we've manually moved closer to the next position
        if self.current_location == next_pos:
            self.path_index += 1  # Progress along the path

        else:
            # Calculate direction and send commands
            direction = self.calculate_direction(self.current_location, next_pos)
            if self.arduino:
                self.send_movement_command(direction)
            
            # Update the display, keeping the path highlighted
            self.update_current_location(next_pos)
            self.path_index += 1
        
        # Check if we reached the target POI
        if self.check_if_on_poi():
            print("Arrived at Point of Interest!")
            self.autonomous_mode = False
            return False

        return True

    def run(self):
        pygame.event.set_grab(True)
        clock = pygame.time.Clock()
        running = True
        last_mouse_pos = pygame.mouse.get_pos()
        move_delay = 0.5  # Delay between autonomous moves (seconds)
        last_move_time = time.time()

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    if self.arduino:
                        self.arduino.close()
                    running = False

                elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            pygame.event.set_grab(False)
                            print("Escape key pressed. Ungrabbing mouse.")
                        # Toggle autonomous mode
                        self.autonomous_mode = not self.autonomous_mode
                        if self.autonomous_mode:
                            nearest_poi = self.find_nearest_poi(self.current_location)
                            if nearest_poi:
                                self.current_path = self.find_path(self.current_location, nearest_poi)
                                self.path_index = 0
                        else:
                            if self.arduino:
                                self.send_movement_command('STOP')

            # Track mouse movement for manual updates
            current_mouse_pos = pygame.mouse.get_pos()
            dx = current_mouse_pos[0] - last_mouse_pos[0]
            dy = current_mouse_pos[1] - last_mouse_pos[1]
            last_mouse_pos = current_mouse_pos

            # Convert mouse movement to grid updates
            cell_dx = dx // self.CELL_SIZE
            cell_dy = dy // self.CELL_SIZE

            if cell_dx != 0 or cell_dy != 0:
                new_row = self.current_location[0] + cell_dy
                new_col = self.current_location[1] + cell_dx

                # Ensure the new position is valid
                if (0 <= new_row < self.rows and 0 <= new_col < self.cols 
                        and self.matrix[new_row][new_col] != 1):
                    self.update_current_location((new_row, new_col))
                    print(f"Moved to grid: ({new_row}, {new_col})")

                    # Check if reached a POI
                    if self.check_if_on_poi():
                        print("Reached POI!")
                        self.autonomous_mode = False
                    else:
                        # Recalculate the path dynamically
                        nearest_poi = self.find_nearest_poi(self.current_location)
                        if nearest_poi:
                            print("Recalculating path to nearest POI...")
                            self.current_path = self.find_path(self.current_location, nearest_poi)
                            self.path_index = 0

            # Handle autonomous path-following
            if self.autonomous_mode and time.time() - last_move_time >= move_delay:
                if not self.move_along_path():
                    print("Path completed.")
                last_move_time = time.time()

            # Draw the map
            path_to_draw = self.current_path if self.autonomous_mode else []
            self.draw_map(path_to_draw)
            clock.tick(30)

        pygame.quit()


if __name__ == "__main__":
    gui = IndoorMapGUI()
    gui.run()