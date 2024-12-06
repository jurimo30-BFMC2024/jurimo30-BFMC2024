from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    mainCamera,
    serialCamera,
    SpeedMotor,
    SteerMotor,
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
import json
import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
import base64
import numpy as np

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
        rospy.init_node('SimCom', anonymous=False)

    def run(self):
        # reset position
        command = {"action": "1", "speed": 0}
        self.commandPublisherRospy.publish(json.dumps(command))
        command = {"action": "steer", "steerAngle": 0}
        self.commandPublisherRospy.publish(json.dumps(command))

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

            try:
                rospy.sleep(.1)
            except:
                pass

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.commandPublisherRospy = rospy.Publisher('/automobile/command', String, queue_size=1)
        self.steerMotorSubscriber = messageHandlerSubscriber(self.queuesList, SteerMotor, "lastOnly", True)
        self.speedMotorSubscriber = messageHandlerSubscriber(self.queuesList, SpeedMotor, "lastOnly", True)

        # steer +-20.5, speed 20
        self.mainCameraSubscriberRospy = rospy.Subscriber("/automobile/image_raw", Image, self.mainCameraCallback)
        self.mainCameraSender = messageHandlerSender(self.queuesList, mainCamera)
        self.serialCameraSubscriberRospy = rospy.Subscriber("/automobile/image_raw/compressed", CompressedImage, self.compressedCameraCallback)
        self.serialCameraSender = messageHandlerSender(self.queuesList, serialCamera)
        pass

    def mainCameraCallback(self, data):
        bridge = CvBridge()
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

            # cv2.imshow("Camera Feed", cv_image)
            # cv2.waitKey(1)
            
            # Convert to Base64-encoded JPEG
            _, encodedImg = cv2.imencode(".jpg", cv_image)
            encodedImageData = base64.b64encode(encodedImg).decode("utf-8")

            self.mainCameraSender.send(encodedImageData)
        except Exception as e:
            print(f"Failed to process image: {e}")

    def compressedCameraCallback(self, data):
        try:
            # Decode the compressed image
            np_arr = np.frombuffer(data.data, np.uint8)  # Convert to NumPy array
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode image as BGR
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YUV_I420)

            _, encodedImg = cv2.imencode(".jpg", cv_image)
            encodedImageData = base64.b64encode(encodedImg).decode("utf-8")
            
            # Display the image
            # cv2.imshow("Compressed Camera Feed", cv_image)
            # cv2.waitKey(1)

            self.serialCameraSender.send(encodedImageData)
        except Exception as e:
            print(f"Failed to process image: {e}")