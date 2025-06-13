# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
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
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
from twisted.internet import task
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.allMessages import VehicleToEverything


class periodicTask(task.LoopingCall):
    def __init__(self, interval, queuesList, tcp_factory):
        super().__init__(self.periodicCheck)
        self.interval = interval
        self.tcp_factory = tcp_factory
        
        # Single subscriber for all message types
        self.subscriber = messageHandlerSubscriber(queuesList, VehicleToEverything, "FIFO", True)

    def start(self):
        """
        Start the periodic task with the specified interval.
        """
        super().start(self.interval)

    def _format_outgoing_data(self, type, values):
        """
        Format data for server transmission.
        
        Message types and their values:
        - devicePos: [x, y] - Position coordinates of the vehicle
            value1: x coordinate (float)
            value2: y coordinate (float)
        
        - deviceRot: [angle] - Rotation/heading of the vehicle
            value1: angle in degrees (float)
            
        - deviceSpeed: [speed] - Current speed of the vehicle
            value1: speed value (float)
            
        - historyData: [x, y, angle] - Historical position of obstacles
            value1: x coordinate (float)
            value2: y coordinate (float)
            value3: type of obstacle
        """
        data = {"reqORinfo": "info", "type": type}
        for i, v in enumerate(values[:3], 1):
            data[f"value{i}"] = v
        return data

    def periodicCheck(self):
        """
        Perform the periodic check and send data to the server.
        """
        # Only process messages if connected
        if self.tcp_factory.isConnected():
            while self.subscriber.isDataInPipe():
                msg = self.subscriber.receive()
                formatted_msg = self._format_outgoing_data(msg["type"], msg["values"])
                self.tcp_factory.send_data_to_server(formatted_msg)
