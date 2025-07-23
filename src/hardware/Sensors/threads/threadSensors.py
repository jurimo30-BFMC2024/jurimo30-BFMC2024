from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import FrontSensors, SideSensors, Heading
import serial
import json

# Configure serial port
SERIAL_PORT = '/dev/ttyUSB0'  # Change this to match your USB port
BAUD_RATE = 115200  # Must match ESP32's baud rate

class threadSensors(ThreadWithStop):
    """This thread handles Sensors.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.subscribe()
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        self.frontSensorSender = messageHandlerSender(self.queuesList, FrontSensors)
        self.sideSensorSender = messageHandlerSender(self.queuesList, SideSensors)
        self.headingSender = messageHandlerSender(self.queuesList, Heading)

        self.heading_error = 23.0  # Change this to match location error value

        super(threadSensors, self).__init__()

    def run(self):
        try:
            while self._running:
                try:
                    # Read a line from serial
                    line = self.ser.readline().decode('utf-8').strip()
                    
                    if line:
                        # Parse JSON data
                        data = json.loads(line)
                        
                        if self.debugging:
                            self.logging.info(f"Received from esp32: {data}")

                        # Extract front sensor data safely
                        front_data = data.get('f', [10000.0, 0.0])
                        if not isinstance(front_data, list) or len(front_data) < 2:
                            front_data = [10000.0, 0.0]

                        # Extract side sensor data safely
                        left_data = data.get('l', 10000.0)
                        right_data = data.get('r', 10000.0)

                        # Extract heading data safely
                        yaw = data.get('yaw', 0.0)
                        yaw = 360 - yaw
                        # Normalize to 0-360
                        yaw = yaw % 360
                        # Apply heading error correction
                        yaw = (yaw - self.heading_error) % 360
                        
                        self.frontSensorSender.send({
                            "distance": front_data[0] if front_data[0] != 0.0 else 10000.0,
                            "relative_speed": front_data[1],
                        })

                        self.sideSensorSender.send({
                            "left": left_data if isinstance(left_data, (int, float)) and left_data != 0.0 else 10000.0,
                            "right": right_data if isinstance(right_data, (int, float)) and right_data != 0.0 else 10000.0,
                        })

                        self.headingSender.send(float(yaw))
                        
                except json.JSONDecodeError:
                    if self.debugging:
                        self.logging.warning(f"Error decoding JSON: {line}")
                except UnicodeDecodeError:
                    if self.debugging:
                        self.logging.warning("Error decoding serial data")
                except KeyError as e:
                    if self.debugging:
                        self.logging.warning(f"Missing expected key in data: {e}")
                except IndexError:
                    if self.debugging:
                        self.logging.warning("Received incomplete sensor data")

        except serial.SerialException as e:
            if self.debugging:
                self.logging.error(f"Serial port error: {e}")
        finally:
            # Ensure serial port is closed safely
            if hasattr(self, 'ser') and self.ser and self.ser.is_open:
                self.ser.close()

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        pass
