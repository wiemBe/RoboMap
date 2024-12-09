# gui/indoor_map_gui.py
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
        self.current_path = []
        self.path_index = 0
        self.arduino = None
        self.connect_arduino()

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
                self.arduino.write(commands[direction].encode())
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
            self.autonomous_mode = False
            if self.arduino:
                self.send_movement_command('STOP')
            print("Path completed or no valid path.")
            return False
        next_pos = self.current_path[self.path_index + 1]
        if self.current_location == next_pos:
            self.path_index += 1
        else:
            direction = self.calculate_direction(self.current_location, next_pos)
            if self.arduino:
                self.send_movement_command(direction)
            self.update_current_location(next_pos)
            self.path_index += 1
        if self.check_if_on_poi():
            print("Arrived at Point of Interest!")
            self.autonomous_mode = False
            return False
        return True

    def run(self):
        clock = pygame.time.Clock()
        running = True
        last_move_time = time.time()
        move_delay = 0.5

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        self.autonomous_mode = not self.autonomous_mode
                        if self.autonomous_mode:
                            self.fetch_pois_from_server()
                            active_poi = self.find_nearest_poi(self.current_location)
                            if active_poi:
                                self.current_path = self.find_path(self.current_location, active_poi)
                                self.path_index = 0
                            else:
                                print("No active POI available")

            if self.autonomous_mode and time.time() - last_move_time >= move_delay:
                self.fetch_pois_from_server()
                current_target = self.current_path[-1] if self.current_path else None
                if current_target and self.matrix[current_target[0]][current_target[1]] == 2:
                    if not self.move_along_path():
                        print("Path completed.")
                        self.autonomous_mode = False
                else:
                    print("Target POI is no longer active")
                    self.autonomous_mode = False
                    self.current_path = []
                last_move_time = time.time()

            path_to_draw = self.current_path if self.autonomous_mode else []
            self.draw_map(path_to_draw)
            clock.tick(30)

        pygame.quit()