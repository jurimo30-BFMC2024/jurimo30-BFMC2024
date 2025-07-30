# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC orginazers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# import eventlet
# eventlet.monkey_patch()

if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

import inspect, psutil, json, threading

from flask import Flask, request
from flask_socketio import SocketIO, emit
from flask_cors import CORS
from enum import Enum
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.templates.workerprocess import WorkerProcess
from src.dashboard.threads.threadStartFrontend import ThreadStartFrontend  
from src.utils.messages.allMessages import Semaphores
import src.utils.messages.allMessages as allMessages
from src.hardware.camera.encoder import decode_frame_to_base64

class processDashboard(WorkerProcess):
    """This process handles the dashboard interactions, updating the UI based on the system's state.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        deviceID (int): The identifier for the specific device.
    """
    # ====================================== INIT ==========================================
    def __init__(self, queueList, logging, debugging = False):
        self.running = True
        self.queueList = queueList
        self.logger = logging
        self.debugging = debugging
        self.messages = {}
        self.sendMessages = {}
        self.messagesAndVals = {}
        self.memoryUsage = 0
        self.cpuCoreUsage = 0
        self.cpuTemperature = 0

        self.sessionActive = False
        
        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", async_mode='eventlet')
        CORS(self.app, supports_credentials=True)

        self.getNamesAndVals()
        # Remove the mainCamera and Semaphores message sender
        self.messagesAndVals.pop("mainCamera")
        self.messagesAndVals.pop("Semaphores")

        self.subscribe()

        # Define WebSocket event handlers
        self.socketio.on_event('message', self.handleMessage)
        self.socketio.on_event('save', self.handleSaveTableState)
        self.socketio.on_event('load', self.handleLoadTableState)

        # Setting up a background task to automatically send the information to the host
        self.sendContinuousHardwareData()
        self.socketio.start_background_task(self.sendContinuousMessages)
        super(processDashboard, self).__init__(self.queueList)

    # ===================================== STOP ==========================================
    def stop(self):
        super(processDashboard, self).stop()

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads."""
        self._init_threads()

        for th in self.threads:
            th.daemon = self.daemon
            th.start()

        self.socketio.run(self.app, host='0.0.0.0', port=5005)

    def subscribe(self):
        """Subscribe function. In this function we make all the required subscribe to process gateway"""
        
        for name, enum in self.messagesAndVals.items():
            if enum["owner"] != "Dashboard":
                subscriber = messageHandlerSubscriber(self.queueList, enum["enum"], "lastOnly", True)
                self.messages[name] = {"obj": subscriber}
            else:
                sender = messageHandlerSender(self.queueList,enum["enum"])
                self.sendMessages[str(name)] = {"obj": sender}
        subscriber = messageHandlerSubscriber(self.queueList, Semaphores, "fifo", True) # we need it as a fifo so we can see all the semaphores status
        self.messages["Semaphores"] = {"obj": subscriber}
                
    def getNamesAndVals(self):
        classes = inspect.getmembers(allMessages, inspect.isclass)
        for name, cls in classes:
            if name != "Enum" and issubclass(cls, Enum):
                self.messagesAndVals[name] = {"enum": cls, "owner": cls.Owner.value}

    def handleMessage(self, data):
        if self.debugging:
            self.logger.info("Received message: " + str(data))

        dataDict = json.loads(data)
        dataName = dataDict["Name"]
        socketId = request.sid
        
        if dataName == "SessionAccess":
            if not self.sessionActive:
                self.sessionActive = True
                self.socketio.emit('session_access', {'data': True}, room=socketId)
            else:
                self.socketio.emit('session_access', {'data': False}, room=socketId)
        elif dataName == "SessionEnd":
            self.sessionActive = False
        else:
            self.sendMessages[dataName]["obj"].send(dataDict["Value"])

        emit('response', {'data': 'Message received: ' + str(data)}, room=socketId)

    def handleSaveTableState(self, data):
        if self.debugging:
            self.logger.info("Received message: " + data)
        dataDict = json.loads(data)
        with open('/home/pi/Brain/src/utils/table_state.json', 'w') as json_file: # change me(path) 
            json.dump(dataDict, json_file, indent=4)  

    def handleLoadTableState(self, data):
        file_path = '/home/pi/Brain/src/utils/table_state.json' # change me(path)
        
        try:
            with open(file_path, 'r') as json_file:
                dataDict = json.load(json_file) 
            emit('loadBack', {'data': dataDict})
        except FileNotFoundError:
            emit('response', {'error': 'File not found. Please save the table state first.'})
        except json.JSONDecodeError:
            emit('response', {'error': 'Failed to parse JSON data from the file.'})

    def sendContinuousHardwareData(self):   
        self.memoryUsage = psutil.virtual_memory().percent
        self.cpuCoreUsage = psutil.cpu_percent(interval=1, percpu=True)
        self.cpuTemperature = round(psutil.sensors_temperatures()['cpu_thermal'][0].current)
        threading.Timer(1, self.sendContinuousHardwareData).start()

    def getSystemStatus(self):
        """Collect and return system status information - 'kako ide' (how is it going)"""
        import time
        import psutil
        
        # Collect basic system info
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        uptime = time.time() - psutil.boot_time()
        
        # Process status - check which processes are active
        process_status = {}
        active_processes = 0
        
        # Check message activity (how many messages received recently)
        message_activity = {}
        for msg_name in self.messages:
            # Check if there are recent messages
            msg_obj = self.messages[msg_name]["obj"]
            # This is a simple indicator - in a real implementation we could track message rates
            message_activity[msg_name] = "active" if hasattr(msg_obj, 'receive') else "inactive"
            if message_activity[msg_name] == "active":
                active_processes += 1
        
        # System health assessment
        cpu_ok = max(self.cpuCoreUsage) < 90 if self.cpuCoreUsage else True
        memory_ok = self.memoryUsage < 85
        temp_ok = self.cpuTemperature < 70 if self.cpuTemperature else True
        
        overall_status = "dobro" if (cpu_ok and memory_ok and temp_ok) else "paznja"  # "good" or "attention"
        
        status_data = {
            "timestamp": current_time,
            "uptime_seconds": int(uptime),
            "overall_status": overall_status,
            "system_health": {
                "cpu_ok": cpu_ok,
                "memory_ok": memory_ok,
                "temperature_ok": temp_ok
            },
            "active_processes": active_processes,
            "total_processes": len(self.messages),
            "message_activity": message_activity,
            "performance": {
                "cpu_usage": max(self.cpuCoreUsage) if self.cpuCoreUsage else 0,
                "memory_usage": self.memoryUsage,
                "temperature": self.cpuTemperature
            }
        }
        
        return status_data

    def sendContinuousMessages(self):
        counter = 1   
        socketSleep = 0.025
        sendTime = 1 

        while self.running == True:
            for msg in self.messages:
                resp = self.messages[msg]["obj"].receive()
                if resp is not None:
                    if msg == "FrontSensors" or msg == "SideSensors":
                        resp = str(resp)
                    elif msg == "serialCamera":
                        resp = decode_frame_to_base64(resp)
                        
                    self.socketio.emit(msg, {"value": resp})
                    if self.debugging:
                        self.logger.info(str(msg))
                        self.logger.info(str(resp))

            if counter < sendTime:
                counter += socketSleep
            else:
                self.socketio.emit('memory_channel', {'data': self.memoryUsage})
                self.socketio.emit('cpu_channel', {'data': {'usage': self.cpuCoreUsage, 'temp': self.cpuTemperature}})
                # Emit system status - kako ide (how is it going)
                system_status = self.getSystemStatus()
                self.socketio.emit('SystemStatus', {'value': system_status})
                counter = 0
            self.socketio.sleep(socketSleep)

    # ===================================== INIT TH ======================================
    def _init_threads(self):
        """Create the Dashboard thread and add to the list of threads."""
        dashboardThreadFrontend = ThreadStartFrontend(self.logger)
        self.threads.append(dashboardThreadFrontend)
