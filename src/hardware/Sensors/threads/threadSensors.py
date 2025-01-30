from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import FrontSensors, SideSensors
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

                        # Process received data
                        self.frontSensorSender.send({
                            "distance": data['f'][0],
                            "relative_speed": data['f'][1],
                        })

                        self.sideSensorSender.send({
                            "left": data['l'],
                            "right": data['r'],
                        })
                        
                except json.JSONDecodeError:
                    if self.debugging:
                        self.logging.warning(f"Error decoding JSON: {line}")
                except UnicodeDecodeError:
                    if self.debugging:
                        self.logging.warning("Error decoding serial data")
                except KeyError as e:
                    if self.debugging:
                        self.logging.warning(f"Missing expected key in data: {e}")

        except serial.SerialException as e:
            if self.debugging:
                self.logging.error(f"Serial port error: {e}")
        finally:
            if 'ser' in locals() and self.ser.is_open:
                self.ser.close()

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        pass
