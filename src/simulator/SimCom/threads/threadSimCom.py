from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    mainCamera,
    SpeedMotor,
    SteerMotor,
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
import json
import rospy
from sensor_msgs.msg import Image, String

class threadSimCom(ThreadWithStop):
    """This thread handles SimCom.
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
        super(threadSimCom, self).__init__()

    def run(self):
        while self._running:
            speedRecv = self.speedMotorSubscriber.receive()
            if speedRecv is not None: 
                if self.debugging:
                    self.logging.info(speedRecv)
                command = {"action": "1", "speed": int(speedRecv) / 10}
                self.commandPublisherRospy.publish(json.dumps(command))

            steerRecv = self.steerMotorSubscriber.receive()
            if steerRecv is not None:
                if self.debugging:
                    self.logging.info(steerRecv)
                command = {"action": "steer", "steerAngle": int(steerRecv)}
                self.commandPublisherRospy.publish(json.dumps(command))
            rospy.sleep(0.1)  # Adjust loop frequency if needed

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.commandPublisherRospy = rospy.Publisher('/automobile/command', String, queue_size=1)
        self.steerMotorSubscriber = messageHandlerSubscriber(self.queuesList, SteerMotor, "lastOnly", True)
        self.speedMotorSubscriber = messageHandlerSubscriber(self.queuesList, SpeedMotor, "lastOnly", True)

        # steer +-20.5, speed 20
        self.cameraSubscriberRospy = rospy.Subscriber("/automobile/image_raw", Image, self.cameraCallback)
        self.cameraSender = messageHandlerSender(self.queueList, mainCamera)
        pass

    def cameraCallback(self, data):
        if self.debugging:
            self.logging.info("camData")
        pass