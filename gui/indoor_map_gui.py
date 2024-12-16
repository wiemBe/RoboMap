import pygame
import time
import requests
import serial
from maps.indoor_map import IndoorMap
from sensors.mouse_sensor import MouseSensor

class IndoorMapGUI(IndoorMap):
    def __init__(self):
        super().__init__()
        pygame.init()
        self.CELL_SIZE = 10
        self.WIDTH = len(self.matrix[0]) * self.CELL_SIZE
        self.HEIGHT = len(self.matrix) * self.CELL_SIZE
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Indoor Map Navigation")
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.YELLOW = (255, 255, 0)
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 0, 255)
        self.autonomous_mode = False
        self.target_poi = None
        self.current_path = []
        self.path_index = 0
        self.arduino = None
        self.connect_arduino()
        self.mouse_sensor = MouseSensor()
        self.start_row, self.start_col = self.current_location
        self.active_server_poi = None  # Track active POI from server
        self.last_sensor_update = time.time()
        self.position_tolerance = 0.1  # meters
        self.update_interval = 0.1  # seconds
        self.last_poi_check = time.time()  # Initialize with current time
        self.poi_check_interval = 1.0  # Check POI every 1 second
        self.last_known_poi = None

    def connect_arduino(self):
        try:
            self.arduino = serial.Serial('COM6', 9600, timeout=1)
            print("Connected to Arduino on COM6")
        except Exception as e:
            print(f"Error connecting to Arduino: {e}")

    def send_movement_command(self, direction):
        if not self.arduino:
            return
        try:
            commands = {
                'UP': 'F',
                'DOWN': 'B',
                'LEFT': 'L',
                'RIGHT': 'R',
                'STOP': 'S'
            }
            if direction in commands:
                command = commands[direction]
                self.arduino.write(command.encode())
                timestamp = time.strftime("%H:%M:%S")
                print(f"[{timestamp}] Sent Arduino command: {direction} ({command})")
        except Exception as e:
            print(f"Error sending command to Arduino: {e}")

    def calculate_direction(self, current_pos, next_pos):
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
                if self.matrix[i][j] == 1:
                    pygame.draw.rect(self.screen, self.BLACK, rect)
                elif self.matrix[i][j] == 2:
                    pygame.draw.rect(self.screen, self.GREEN, rect)
                elif self.matrix[i][j] == 3:
                    pygame.draw.rect(self.screen, self.YELLOW, rect)
                pygame.draw.rect(self.screen, self.BLACK, rect, 1)
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
        min_distance = float('inf')
        nearest_poi = None
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
                active_poi = str(data.get('station'))
                for i in range(self.rows):
                    for j in range(self.cols):
                        if self.matrix[i][j] == 2:
                            self.matrix[i][j] = 0
                if active_poi in self.poi_locations:
                    location = self.poi_locations[active_poi]
                    self.matrix[location[0]][location[1]] = 2
                    print(f"POI {active_poi} is now active at location {location}")
        except requests.exceptions.RequestException as e:
            print(f"Error fetching POIs: {e}")

    def check_if_on_poi(self):
        row, col = self.current_location
        if self.matrix[row][col] == 2:
            for poi_num, location in self.poi_locations.items():
                if location == (row, col):
                    print(f"Reached POI {poi_num}!")
                    return True
        return False

    def get_cell_from_mouse(self, mouse_pos):
        x, y = mouse_pos
        row = y // self.CELL_SIZE
        col = x // self.CELL_SIZE
        row = max(0, min(row, self.rows - 1))
        col = max(0, min(col, self.cols - 1))
        return (row, col)

    def move_along_path(self):
        if not self.current_path or self.path_index >= len(self.current_path) - 1:
            print("\n[AUTONOMOUS] Path completed or invalid")
            self.stop_movement()
            return False

        # Get next target position
        next_pos = self.current_path[self.path_index + 1]
        
        # Print current navigation state
        print(f"\n[AUTONOMOUS] Current position: {self.current_location}")
        print(f"[AUTONOMOUS] Target position: {next_pos}")
        print(f"[AUTONOMOUS] Progress: {self.path_index + 1}/{len(self.current_path)}")
        
        # Update current position from sensor
        position_updated = self.update_position_from_sensor()
        
        if position_updated:
            # Check if reached next position
            if self.is_position_reached(next_pos):
                self.path_index += 1
                print(f"[AUTONOMOUS] Reached waypoint {self.path_index}")
                if self.check_if_on_poi():
                    print("[AUTONOMOUS] Destination POI reached!")
                    self.stop_movement()
                    return False
            else:
                # Calculate and send movement command
                direction = self.calculate_direction(self.current_location, next_pos)
                timestamp = time.strftime("%H:%M:%S")
                print(f"\n[{timestamp}] NAVIGATION:")
                print(f"├── Current: {self.current_location}")
                print(f"├── Target:  {next_pos}")
                print(f"└── Command: {direction}")
                self.send_movement_command(direction)
        
        return True

    def update_position_from_sensor(self):
        current_time = time.time()
        if current_time - self.last_sensor_update < self.update_interval:
            return False

        parsed_data = self.mouse_sensor.read_and_parse_data()
        if parsed_data:
            x_mm, y_mm = self.mouse_sensor.get_displacement_mm()
            # Convert mm to grid cells (assuming resolution is in meters)
            col_offset = int(x_mm / (self.resolution * 1000))
            row_offset = int(y_mm / (self.resolution * 1000))
            new_row = self.start_row + row_offset
            new_col = self.start_col + col_offset
            new_position = (new_row, new_col)
            # Ensure new position is within bounds
            if 0 <= new_row < self.rows and 0 <= new_col < self.cols:
                self.update_current_location(new_position)
                self.last_sensor_update = current_time
                return True
        return False

    def is_position_reached(self, target_pos):
        curr_row, curr_col = self.current_location
        target_row, target_col = target_pos
        
        # Convert grid positions to meters
        curr_x = curr_col * self.resolution
        curr_y = curr_row * self.resolution
        target_x = target_col * self.resolution
        target_y = target_row * self.resolution
        
        # Check if within tolerance
        distance = ((curr_x - target_x)**2 + (curr_y - target_y)**2)**0.5
        return distance <= self.position_tolerance

    def stop_movement(self):
        if self.arduino:
            self.send_movement_command('STOP')
        self.autonomous_mode = False

    def toggle_autonomous_mode(self):
        self.autonomous_mode = not self.autonomous_mode
        print(f"Autonomous mode {'enabled' if self.autonomous_mode else 'disabled'}")
        if self.autonomous_mode:
            print("Initializing autonomous navigation...")
            self.start_autonomous_navigation()
        else:
            self.stop_autonomous_navigation()

    def start_autonomous_navigation(self):
        try:
            print("Fetching POI from server...")
            response = requests.get('http://localhost:8080/available', timeout=2)
            if response.status_code == 200:
                data = response.json()
                active_poi = str(data.get('station'))
                print(f"Server returned POI: {active_poi}")
                
                if active_poi and active_poi in self.poi_locations:
                    target_location = self.poi_locations[active_poi]
                    self.active_server_poi = active_poi
                    self.last_known_poi = active_poi
                    self.current_path = self.find_path(self.current_location, target_location)
                    self.path_index = 0
                    print(f"Path calculated to POI {active_poi} at {target_location}")
                else:
                    print(f"Invalid POI from server: {active_poi}")
                    self.autonomous_mode = False
            else:
                print(f"Server returned status code: {response.status_code}")
                self.autonomous_mode = False
        except requests.exceptions.RequestException as e:
            print(f"Server communication error: {e}")
            self.autonomous_mode = False
        except Exception as e:
            print(f"Unexpected error in autonomous navigation: {e}")
            self.autonomous_mode = False

    def stop_autonomous_navigation(self):
        self.current_path = []
        self.path_index = 0
        if self.arduino:
            self.send_movement_command('STOP')
        print("Stopping autonomous navigation")

    def check_and_update_poi(self):
        current_time = time.time()
        if current_time - self.last_poi_check < self.poi_check_interval:
            return False

        self.last_poi_check = current_time  # Update timestamp before request
        
        try:
            print("Checking for POI updates...")
            response = requests.get('http://localhost:8080/available', timeout=2)
            if response.status_code == 200:
                data = response.json()
                active_poi = str(data.get('station'))
                print(f"Current active POI: {active_poi}")
                
                if active_poi != self.last_known_poi:
                    print(f"POI changed from {self.last_known_poi} to {active_poi}")
                    self.last_known_poi = active_poi
                    
                    # Update map and navigation
                    self._update_poi_on_map(active_poi)
                    return True
        
        except requests.exceptions.RequestException as e:
            print(f"Server communication error in POI check: {e}")
        
        return False

    def _update_poi_on_map(self, active_poi):
        # Clear old POIs
        for i in range(self.rows):
            for j in range(self.cols):
                if self.matrix[i][j] == 2:
                    self.matrix[i][j] = 0
        
        # Set new active POI
        if active_poi in self.poi_locations:
            location = self.poi_locations[active_poi]
            self.matrix[location[0]][location[1]] = 2
            print(f"Updated map with POI {active_poi} at {location}")
            
            # Update navigation if in autonomous mode
            if self.autonomous_mode:
                self.current_path = self.find_path(self.current_location, location)
                self.path_index = 0
                print("Recalculated path for new POI")

    def run(self):
        clock = pygame.time.Clock()
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_a:
                        self.toggle_autonomous_mode()
            
            # Regular position update
            self.update_position_from_sensor()
            
            # Check for POI updates
            poi_updated = self.check_and_update_poi()
            
            # Handle autonomous navigation
            if self.autonomous_mode:
                if poi_updated or not self.move_along_path():
                    if self.last_known_poi in self.poi_locations:
                        target = self.poi_locations[self.last_known_poi]
                        self.current_path = self.find_path(self.current_location, target)
                        self.path_index = 0
            
            # Draw current state
            self.draw_map(self.current_path if self.autonomous_mode else None)
            clock.tick(30)
        
        self.close()

    def close(self):
        if self.mouse_sensor:
            self.mouse_sensor.close()
        pygame.quit()