import serial
import serial.tools.list_ports
from collections import deque

class MouseSensor:
    def __init__(self, resolution=0.2):
        ports = list(serial.tools.list_ports.comports())
        self.port = None
        for p in ports:
            if 'Arduino' in p.description:
                self.port = p.device
                break

        self.serial = serial.Serial(
            port=self.port or 'COM3',
            baudrate=115200,
            timeout=0.1
        )

        self.resolution = resolution
        self.counts_per_mm = 8.0
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.heading = 0.0
        self.filter_size = 5
        self.x_history = deque(maxlen=self.filter_size)
        self.y_history = deque(maxlen=self.filter_size)

    def read_position(self):
        if self.serial.in_waiting >= 6:
            try:
                data = self.serial.readline().decode().strip().split(',')
                if len(data) == 3:
                    dx, dy, dh = map(int, data)
                    dx_mm = dx / self.counts_per_mm
                    dy_mm = dy / self.counts_per_mm
                    self.x_mm += dx_mm
                    self.y_mm += dy_mm
                    self.heading = (self.heading + dh) % 360
                    self.x_history.append(self.x_mm)
                    self.y_history.append(self.y_mm)
                    x_filtered = sum(self.x_history) / len(self.x_history)
                    y_filtered = sum(self.y_history) / len(self.y_history)
                    row = int(y_filtered / (self.resolution * 1000))
                    col = int(x_filtered / (self.resolution * 1000))
                    return row, col, self.heading
            except Exception as e:
                print(f"Sensor read error: {e}")
        return None

    def close(self):
        if self.serial:
            self.serial.close()