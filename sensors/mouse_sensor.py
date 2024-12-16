import usb.core
import usb.util
import struct

class MouseSensor:
    def __init__(self):
        self.device = usb.core.find(idVendor=0x17ef, idProduct=0x60d1)
        if self.device is None:
            raise ValueError("Device not found")
        
        if self.device.is_kernel_driver_active(0):
            self.device.detach_kernel_driver(0)
        
        usb.util.claim_interface(self.device, 0)
        self.x_total = 0
        self.y_total = 0
        self.counts_per_mm = 39  # Counts per mmgit 

    def read_data(self):
        try:
            # Read 8 bytes from endpoint 0x81
            data = self.device.read(0x81, 8, timeout=1000)
            print(f"Raw Data: {list(data)}")
            return data
        except usb.core.USBError as e:
            if e.errno == 110:  # Timeout error on Linux
                print("Read timeout occurred.")
            else:
                print(f"USB Error: {e}")
        return None

    def read_and_parse_data(self):
        data = self.read_data()
        if data:
            parsed = self.parse_frame(data)
            if parsed:
                self.x_total += parsed['x']
                self.y_total += parsed['y']
                return parsed
        return None

    def parse_frame(self, frame_bytes):
        if len(frame_bytes) != 8:
            print("Invalid frame length")
            return None
        
        # Unpack the frame
        id_flag, x, y, reserved = struct.unpack('<HhhH', frame_bytes)
        return {
            "id_flag": id_flag,  # First two bytes
            "x": x,              # Third and fourth bytes
            "y": y,              # Fifth and sixth bytes
            "reserved": reserved # Seventh and eighth bytes
        }

    def get_displacement_mm(self):
        x_mm = self.x_total / self.counts_per_mm
        y_mm = self.y_total / self.counts_per_mm
        return x_mm, y_mm

    def close(self):
        if self.device:
            usb.util.release_interface(self.device, 0)
            if not self.device.is_kernel_driver_active(0):
                try:
                    self.device.attach_kernel_driver(0)
                except usb.core.USBError as e:
                    print(f"Error reattaching kernel driver: {e}")

if __name__ == "__main__":
    mouse_reader = MouseSensor()
    try:
        while True:
            raw_data = mouse_reader.read_data()
            if raw_data:
                parsed_frame = mouse_reader.parse_frame(raw_data)
                if parsed_frame:
                    print(f"Parsed Data: {parsed_frame}")
    except KeyboardInterrupt:
        print("Stopping reader.")
    finally:
        mouse_reader.close()
