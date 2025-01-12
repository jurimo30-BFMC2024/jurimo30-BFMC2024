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

import json
import time
import threading

from src.hardware.serialhandler.threads.messageconverter import MessageConverter
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    Klem,
    Control,
    SteerMotor,
    SpeedMotor,
    Brake,
    ToggleBatteryLvl,
    ToggleImuData,
    ToggleInstant,
    ToggleResourceMonitor,
    CoreControl,
    CoreBrake,
    CoreSpeedMotor,
    CoreSteerMotor
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender


class threadWrite(ThreadWithStop):
    """This thread write the data that Raspberry PI send to NUCLEO.\n

    Args:
        queues (dictionar of multiprocessing.queues.Queue): Dictionar of queues where the ID is the type of messages.
        serialCom (serial.Serial): Serial connection between the two boards.
        logFile (FileHandler): The path to the history file where you can find the logs from the connection.
        example (bool, optional): Flag for exmaple activation. Defaults to False.
    """

    # ===================================== INIT =========================================
    def __init__(self, queues, serialCom, logFile, logger, debugger = False, example=False):
        super(threadWrite, self).__init__()
        self.queuesList = queues
        self.serialCom = serialCom
        self.logFile = logFile
        self.exampleFlag = example
        self.logger = logger
        self.debugger = debugger

        self.mutex = threading.Lock()
        self.running = threading.Event()
        self.engineEnabled = threading.Event()
        self.messageConverter = MessageConverter()
        self.steerMotorSender = messageHandlerSender(self.queuesList, SteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.configPath = "src/utils/table_state.json"

        self.loadConfig("init")
        self.subscribe()

        threading.Thread(target=self.klemWrite, daemon=True).start()
        threading.Thread(target=self.brakeWrite, daemon=True).start()
        threading.Thread(target=self.speedWrite, daemon=True).start()
        threading.Thread(target=self.steerWrite, daemon=True).start()
        threading.Thread(target=self.controlWrite, daemon=True).start()
        threading.Thread(target=self.instantWrite, daemon=True).start()
        threading.Thread(target=self.batteryWrite, daemon=True).start()
        threading.Thread(target=self.resourceMonitorWrite, daemon=True).start()
        threading.Thread(target=self.imuWrite, daemon=True).start()

        if example:
            self.i = 0.0
            self.j = -1.0
            self.s = 0.0
            self.example()

    def subscribe(self):
        """Subscribe function. In this function we make all the required subscribe to process gateway"""

        self.klSubscriber = messageHandlerSubscriber(self.queuesList, Klem, "lastOnly", True)
        self.controlSubscriber = messageHandlerSubscriber(self.queuesList, CoreControl, "lastOnly", True)
        self.steerMotorSubscriber = messageHandlerSubscriber(self.queuesList, CoreSteerMotor, "lastOnly", True)
        self.speedMotorSubscriber = messageHandlerSubscriber(self.queuesList, CoreSpeedMotor, "lastOnly", True)
        self.brakeSubscriber = messageHandlerSubscriber(self.queuesList, CoreBrake, "lastOnly", True)
        self.instantSubscriber = messageHandlerSubscriber(self.queuesList, ToggleInstant, "lastOnly", True)
        self.batterySubscriber = messageHandlerSubscriber(self.queuesList, ToggleBatteryLvl, "lastOnly", True)
        self.resourceMonitorSubscriber = messageHandlerSubscriber(self.queuesList, ToggleResourceMonitor, "lastOnly", True)
        self.imuSubscriber = messageHandlerSubscriber(self.queuesList, ToggleImuData, "lastOnly", True)

    # ==================================== SENDING =======================================

    def sendToSerial(self, msg):
        command_msg = self.messageConverter.get_command(**msg)
        if command_msg != "error":
            with self.mutex:  # Lock the critical section
                self.serialCom.write(command_msg.encode("ascii"))
                self.logFile.write(command_msg)
                time.sleep(0.05)

    def loadConfig(self, configType):
        with open(self.configPath, "r") as file:
            data = json.load(file)

        if configType == "init":
            data = data[len(data)-1]
            command = {"action": "batteryCapacity", "capacity": data["batteryCapacity"]["capacity"]}
            self.sendToSerial(command)
            time.sleep(0.05)
        else:
            for e in range(4):
                if data[e]["value"] == "False":
                    value = 0
                else:
                    value = 1 
                command = {"action": data[e]['command'], "activate": value}
                self.sendToSerial(command)
                time.sleep(0.05)

    def convertFc(self,instantRecv):
        if instantRecv =="True":
            return 1
        else :
            return 0
        
    def klemWrite(self):
        while self._running:
            try:
                klRecv = self.klSubscriber.receiveWithBlock()
                if self.debugger:
                    self.logger.info(f"klRecv: {klRecv}")
                if klRecv == "30":
                    self.running.set()
                    self.engineEnabled.set()
                    command = {"action": "kl", "mode": 30}
                    self.sendToSerial(command)
                    self.loadConfig("sensors")
                elif klRecv == "15":
                    self.running.set()
                    self.engineEnabled.clear()
                    command = {"action": "kl", "mode": 15}
                    self.sendToSerial(command)
                    self.loadConfig("sensors")
                elif klRecv == "0":
                    self.running.clear()
                    self.engineEnabled.clear()
                    command = {"action": "kl", "mode": 0}
                    self.sendToSerial(command)
            except Exception as e:
                self.logger.error(e)
    
    def brakeWrite(self):
        while self._running:
            try:
                brakeRecv = self.brakeSubscriber.receiveWithBlock()
                if self.running.is_set():
                    if self.engineEnabled.is_set():
                        if self.debugger:
                            self.logger.info(f"brakeRecv: {brakeRecv}")
                        command = {"action": "brake", "steerAngle": int(brakeRecv)}
                        self.sendToSerial(command)
            except Exception as e:
                self.logger.error(e)

    def speedWrite(self):
        while self._running:
            try:
                speedRecv = self.speedMotorSubscriber.receiveWithBlock()
                if self.running.is_set():
                    if self.engineEnabled.is_set():
                        if self.debugger:
                            self.logger.info(f"speedRecv: {speedRecv}")
                        command = {"action": "speed", "speed": int(speedRecv)}
                        self.sendToSerial(command)
            except Exception as e:
                self.logger.error(e)

    def steerWrite(self):
        while self._running:
            try:
                steerRecv = self.steerMotorSubscriber.receiveWithBlock()
                if self.running.is_set():
                    if self.engineEnabled.is_set():
                        if self.debugger:
                            self.logger.info(f"steerRecv: {steerRecv}")
                        command = {"action": "steer", "steerAngle": int(steerRecv)}
                        self.sendToSerial(command)
            except Exception as e:
                self.logger.error(e)

    def controlWrite(self):
        while self._running:
            try:
                controlRecv = self.controlSubscriber.receiveWithBlock()
                if self.running.is_set():
                    if self.engineEnabled.is_set():
                        if self.debugger:
                            self.logger.info(f"controlRecv: {controlRecv}") 
                        command = {
                            "action": "vcd",
                            "time": int(controlRecv["Time"]),
                            "speed": int(controlRecv["Speed"]),
                            "steer": int(controlRecv["Steer"]),
                        }
                        self.sendToSerial(command)
            except Exception as e:
                self.logger.error(e)

    def instantWrite(self):
        while self._running:
            try:
                instantRecv = self.instantSubscriber.receiveWithBlock()
                if self.running.is_set():
                    if self.debugger:
                        self.logger.info(f"instantRecv: {instantRecv}") 
                    command = {"action": "instant", "activate": int(instantRecv)}
                    self.sendToSerial(command)
            except Exception as e:
                self.logger.error(e)

    def batteryWrite(self):
        while self._running:
            try:
                batteryRecv = self.batterySubscriber.receiveWithBlock()
                if self.running.is_set():
                    if self.debugger:
                        self.logger.info(f"batteryRecv: {batteryRecv}")
                    command = {"action": "battery", "activate": int(batteryRecv)}
                    self.sendToSerial(command)
            except Exception as e:
                self.logger.error(e)
    
    def resourceMonitorWrite(self):
        while self._running:
            try:
                resourceMonitorRecv = self.resourceMonitorSubscriber.receiveWithBlock()
                if self.running.is_set():
                    if self.debugger:
                        self.logger.info(f"resourceMonitorRecv: {resourceMonitorRecv}")
                    command = {"action": "resourceMonitor", "activate": int(resourceMonitorRecv)}
                    self.sendToSerial(command) 
            except Exception as e:
                self.logger.error(e)

    def imuWrite(self):
        while self._running:
            try:
                imuRecv = self.imuSubscriber.receiveWithBlock()
                if self.running.is_set():
                    if self.debugger:
                        self.logger.info(f"imuRecv: {imuRecv}")
                    command = {"action": "imu", "activate": int(imuRecv)}
                    self.sendToSerial(command)
            except Exception as e:
                self.logger.error(e)
        
    # ===================================== RUN ==========================================
    def run(self):
        pass

    # ==================================== START =========================================
    def start(self):
        super(threadWrite, self).start()

    # ==================================== STOP ==========================================
    def stop(self):
        """This function will close the thread and will stop the car."""

        self.exampleFlag = False
        command = {"action": "kl", "mode": 0.0}
        self.sendToSerial(command)
        time.sleep(2)
        super(threadWrite, self).stop()

    # ================================== EXAMPLE =========================================
    def example(self):
        """This function simulte the movement of the car."""

        if self.exampleFlag:
            self.signalRunningSender.send({"Type": "Run", "value": True})
            self.speedMotorSender.send({"Type": "Speed", "value": self.s})
            self.steerMotorSender.send({"Type": "Steer", "value": self.i})
            self.i += self.j
            if self.i >= 21.0:
                self.i = 21.0
                self.s = self.i / 7
                self.j *= -1
            if self.i <= -21.0:
                self.i = -21.0
                self.s = self.i / 7
                self.j *= -1.0
            threading.Timer(0.01, self.example).start()
