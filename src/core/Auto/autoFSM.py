if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.core.Auto.Parking.Parking import Parking
from src.core.Auto.Overtake.Overtake import Overtake
from src.core.Auto.Localization.Localization import Localization
from src.core.Auto.pathPlanning.PositionFinder import PositionFinder
from src.core.Core.ControlModeThread.ControlModeThread import ControlModeThread
from src.core.Auto.LaneFollow.LaneFollow import LaneFollower as LaneFollowController
from src.core.Auto.SpeedControl import SpeedControl
from src.core.Auto.IntersectionControl import IntersectionControl
from src.core.Auto.RoundaboutControl import RoundaboutController 
from src.core.Auto.Crosswalk import CrosswalkController
from src.utils.messages.allMessages import (
    LaneDetect,
    CoreSteerMotor,
    CoreSpeedMotor,
    StopLineDetect,
    ObjectDetection,
    TrafficSignsDetection,
    SideSensors,
    FrontSensors,
    ParkingSpotDetect,
    Location,
    Heading,
    VehicleToEverything,
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.core.Auto.pathPlanning.pathPlanning import PathPlanner
from src.core.Auto.TrafficSignController import TrafficSignController
from src.core.Auto.CollisionDetector import CollisionDetector # DODATO

import time
from enum import Enum, auto
from src.data.TrafficCommunication.useful.Obstacles import Obstacles
from src.core.Auto.SpecialReactions.IntersectionCrosswalk import IntersectionCrosswalkController
from src.core.Auto.SpecialReactions.CrosswalkIntersection import CrosswalkIntersectionController

"""
    - stopline staviti u jedan stopline subscriber
"""

class autoFSMState(Enum):
    DRIVE = auto()
    DRIVE_OVERTAKE = auto()
    PARKING = auto()
    INTERSECTION = auto()
    ROUNDABOUT = auto()
    CROSSWALK = auto()
    HIGHWAY = auto()
    INTERSECTION_CROSSWALK = auto()
    CROSSWALK_INTERSECTION = auto()

class autoFSM(ControlModeThread):
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        
        self.steerMotorSender = messageHandlerSender(self.queuesList, CoreSteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, CoreSpeedMotor)
        self.vehicleToEverythingSender = messageHandlerSender(self.queuesList, VehicleToEverything)

        # Define special node lists with descriptive names
        self.tunnel_nodes = [
            "7", "151", "152", "153", "154", "155", "156", "157", "158", "159", "160", "161", "162", "163", "164", "82",
            "81", "165", "166", "167", "168", "169", "170", "171", "172", "173", "174", "175", "176", "177", "8"
        ]

        self.overtake_nodes = [
            "55", "305", "306", "307", "308", "309", "318", "319", "320", "321",
            "348", "349", "350", "351", "352", "385", "386", "387", "388", "389",
            "444", "445", "446", "447", "448", "449", "450", "451", "452", "453", 
            "454", "455", "456", "457", "458", "459", "460", "461", "462", "463", 
            "464", "465", "466", "467", "468", "469", "470", "471", "472", "473", 
            "474", "475", "476", "477", "478", "479", "480", "481", "482"
        ]

        self.special_intersection_nodes = [
            "190", "191", "192"
        ]

        # Nodes where intersection and crosswalk appear together
        self.intersection_crosswalk_nodes = ["149", "2", "219", "4", "301", "6", "443", "34", "180", "80"]  # Example nodes, adjust based on your map
        self.crosswalk_intersection_nodes = ["175", "176", "177", "8", "162", "163", "164", "82"]  # Example nodes, adjust based on your map

        self.subscribe()
        super().__init__()

    def start(self):
        self.positionFinder = PositionFinder("Small_map_roundabout.graphml")
        self.laneFollowContrler = LaneFollowController(512, 270, self.logging, False)
        self.collisionDetector = CollisionDetector(512, 270, 50, 50, self.logging, True)
        self.speedControler = SpeedControl(self.logging, False)
        self.intersectionController = IntersectionControl(self.logging, self.debugging)
        self.parkingController = Parking(self.logging, self.debugging)
        self.overtakeController = Overtake(self.logging, self.debugging)
        self.roundaboutController = RoundaboutController(512, 270, self.logging, True)
        
        # Maximum wheel angle for allowing overtaking (in degrees)
        self.max_wheel_angle_for_overtake = 7
        self.crosswalkController = CrosswalkController()
        self.intersectionCrosswalkController = IntersectionCrosswalkController(self.intersectionController, self.crosswalkController)
        self.crosswalkIntersectionController = CrosswalkIntersectionController(self.intersectionController, self.crosswalkController)

        self.laneDetectSubscriber.empty()
        self.stopLineDetectionSubscriber.empty()
        self.objectDetectionSubscriber.empty()
        self.trafficSignsSubscriber.empty()
        self.sideSensorSubscriber.empty()
        self.frontSensorSubscriber.empty()
        self.parkingSpotDetectionSubscriber.empty()
        self.locationSubscriber.empty()
        self.headingSubscriber.empty()

        self.last_sent_time_v2x = 0
        self.oldAngle = 0
        self.oldSpeed = 0
        self.steerMotorSender.send("0")
        self.speedMotorSender.send("0")

        best_node = 52  # Starting node, for testing purposes, remove later

        if best_node is None:
            print("No predefined node, using localization system to find best node")
            heading = self.headingSubscriber.receiveWithBlock()
            print(f'Current heading: {heading}')
            location = self.locationSubscriber.receiveWithBlock()
            print(f'Current location: {location}')

            # Initialize localization systems
            best_node, best_node_offset = self.positionFinder.find_best_node(float(location['x']), float(location['y']), heading)
            print(f'Current node: {best_node} with offset: {best_node_offset}cm ')
        else:
            print(f'Using predefined node: {best_node}')
            print("Predefined node is used, this should be removed later!")
            print("Make sure to set the best_node variable to None for automatic node detection!")

        # od start pointa do kruznog
        self.planer = PathPlanner(start=192, goal=317, mode="pacman")
        # kroz maglu
        # self.planer = PathPlanner(start=97, goal=186, mode="pacman")
        self.navigateCommand, segments = self.planer.planPath()
        #self.navigateCommand.append("Left")
        #self.navigateCommand.append("Left")
        #self.navigateCommand.append("Straight")
        self.localization = Localization(segments)

        print(f'Navigation commands {self.navigateCommand}')
        self.traffic_signs = TrafficSignController([
            "stop", "crosswalk", "highway_entrance", "highway_exit",
            "one_way", "no_entry", "parking", "priority",
            "round_about", "round_about2"
        ])

        self.traffic_light_states = TrafficSignController([
            "red", "green", "yellow", "red_yellow"
        ])

        self.sign_car_position = None
        self.obstacle_start_time = None
        self.stephanie_position = None
        self.roundaboutExit_position = None
        self.intersectionSign = "None"

        # DODATO: Nove promenljive za praćenje kolizije u centralnoj zoni
        self.stefanija_in_danger_zone = False
        self.car_in_danger_zone = False

        self.state = autoFSMState.DRIVE
        self.localization.start_new_segment()

        self.last_parking_exit_time = 0  # Track last time parking state was exited

        super().start()
    
    def stop(self):
        super().stop()

    def loop(self):
        self.leftX, self.rightX, self.leftVisible, self.rightVisible = self.laneDetectSubscriber.receiveWithBlock()
        stop_line_present, stop_line_distance, stop_line_angle = self.stopLineDetectionSubscriber.receiveWithBlock() # stopLine je sad tuple (intersection(bool), slope_degrees (float))
        stop_line_present_close = stop_line_present and stop_line_distance < 130
        stop_line_present_semaphore = stop_line_present and stop_line_distance < 110

        while self.objectDetectionSubscriber.isDataInPipe():
            detected_objects_dict = self.objectDetectionSubscriber.receive() # This is now a dict
            object_name = detected_objects_dict.get("name")
            object_position = detected_objects_dict.get("position")

            # DODATO: Logika za proveru kolizije za 'stefanija' i 'car'
            is_in_central_zone = False
            if object_position and (object_name == "stefanija" or object_name == "car"):
                is_in_central_zone = self.collisionDetector.check_collision(object_position, object_name)
                print(f"Detekcija opasne zone: {is_in_central_zone}")
            if object_name == "stefanija":
                self.stephanie_position = object_position
                self.stefanija_in_danger_zone = is_in_central_zone # Ažuriranje flag-a
                #print(f"Stephanie present: {self.stephanie_position}")
            elif object_name == "exit":
                self.roundaboutExit_position = object_position
                print(f"Roundabout Exit present: {self.roundaboutExit_position}")
            elif object_name == "car":
                self.sign_car_position = object_position
                self.car_in_danger_zone = is_in_central_zone # Ažuriranje flag-a
                print(f"In danger zone: {self.car_in_danger_zone}")
            else:
                raise ValueError(f'Unknown object detected: {detected_objects_dict}')
            
            if self.debugging:
                print(f"Preuzet je objekat: {object_name}, Prisutan: {object_position}")

        while self.trafficSignsSubscriber.isDataInPipe():
            sign = self.trafficSignsSubscriber.receive() # This is a string (sign name)
            if sign in self.traffic_light_states:
                self.traffic_light_states.set_active(sign)
                if self.debugging: print(f"Traffic light detected: {sign}")
            elif sign in self.traffic_signs:
                self.traffic_signs.set_active(sign)
                #if self.debugging: 
                print(f"Traffic sign detected: {sign}")
            else:
                raise ValueError(f'Unknown sign detected: {sign}')
            
            if self.debugging:
                print(f"Preuzet je saobracajni znak: {sign}")

        parking_spot_detected = self.parkingSpotDetectionSubscriber.receive() != None

        #ulaz obrade sa ESP
        front_sensors = self.frontSensorSubscriber.receiveWithBlock()
        side_sensors = self.sideSensorSubscriber.receiveWithBlock()
        heading = self.headingSubscriber.receiveWithBlock()

        traffic_light_present = self.traffic_light_states.get_active() != None

        obstacle = front_sensors["distance"] <= 80 and self.sign_car_position
        angle = self.laneFollowContrler.process_following(self.leftX, self.rightX) # calculate steering angle from lane follow data

        if not obstacle:
            self.obstacle_start_time = None  # Reset if obstacle is not present

        if not self._running.is_set():
            return
        
        ##############################         SET STATES            #############################

        # Get current node for special cases
        current_node = self.localization.get_current_node()

        if self.state == autoFSMState.HIGHWAY:
            if self.traffic_signs.get_active() == "highway_exit" or stop_line_present:
                print("Izlazak sa auto puta")
                self.traffic_signs.clear()
                self.state = autoFSMState.DRIVE
                self.laneFollowContrler.set_pid_highway(False)

        
        if self.state == autoFSMState.DRIVE:
            # Ignore parking sign if detected within 30 seconds after exiting parking
            parking_sign_detected = self.traffic_signs.get_active() == "parking"
            recently_exited_parking = (time.time() - self.last_parking_exit_time) < 30
            if parking_sign_detected and not recently_exited_parking:
                self.traffic_signs.clear()
                self.state = autoFSMState.PARKING
            elif parking_sign_detected and recently_exited_parking:
                # Ignore this parking sign
                print("Parking sign detected but recently exited parking, ignoring sign")
                self.traffic_signs.clear()
            

            elif stop_line_present_close and (self.traffic_signs.get_active() in ["stop", "priority"] or traffic_light_present):
                if current_node in self.intersection_crosswalk_nodes:
                    if self.debugging:
                        print("Intersection and crosswalk detected together")

                    self.intersectionCrosswalkController.setCourse(
                        sign=self.traffic_signs.get_active(), 
                        direction=self.navigateCommand.pop(0),
                        traffic_light_present=traffic_light_present
                    )
                    self.traffic_signs.clear()
                    self.state = autoFSMState.INTERSECTION_CROSSWALK
                else: 
                    if self.debugging:
                        print("Krecemo sa raskrsnicom")
                    
                    self.intersectionController.setCourse(
                        sign=self.traffic_signs.get_active(), 
                        direction=self.navigateCommand.pop(0),
                        traffic_light_present=traffic_light_present
                    )
                    self.traffic_signs.clear()
                    self.state = autoFSMState.INTERSECTION


            elif stop_line_present_semaphore and traffic_light_present:
                print("Intersection with traffic light detected")

                self.intersectionController.setCourse(
                    sign=self.traffic_light_states.get_active(), 
                    direction=self.navigateCommand.pop(0),
                    traffic_light_present=True
                )
                self.state = autoFSMState.INTERSECTION

            elif stop_line_present_close and self.traffic_signs.get_active() in ["round_about", "round_about2"]:
                print("Entering roundabout")
                isStarted = self.roundaboutController.start(self.navigateCommand.pop(0))
                self.traffic_signs.clear()
                self.roundaboutExit_position = None
                self.state = autoFSMState.ROUNDABOUT

            elif stop_line_present and self.traffic_signs.get_active() == "crosswalk":
                if current_node in self.crosswalk_intersection_nodes:
                    if self.debugging:
                        print("Detected intersection after crosswalk")
                    
                    self.intersectionController.setCourse(
                        sign="priority", 
                        direction=self.navigateCommand.pop(0),
                        traffic_light_present=False
                    )
                    self.traffic_signs.clear()
                    self.state = autoFSMState.CROSSWALK_INTERSECTION
                else:
                    if self.debugging:
                        print("Krecemo sa pesackim prelazom")
                    self.state = autoFSMState.CROSSWALK

            elif stop_line_present_close:
                print("Stop line detected, no sign, entering intersection")

                self.intersectionController.setCourse(
                    sign="stop", 
                    direction=self.navigateCommand.pop(0),
                    traffic_light_present=False
                )

                self.state = autoFSMState.INTERSECTION
                        
            elif self.traffic_signs.get_active() == "highway_entrance":
                print("Ulazak na autoput")
                self.traffic_signs.clear()
                self.state = autoFSMState.HIGHWAY
                self.laneFollowContrler.set_pid_highway(True)
            
            elif obstacle and self.oldSpeed == 0:
                # Check if vehicle is in overtake nodes and wheel angle is acceptable
                current_node = self.localization.get_current_node()
                if (current_node in self.overtake_nodes and 
                    abs(angle) <= self.max_wheel_angle_for_overtake * 10):  # angle is in tenths of degrees
                    if self.obstacle_start_time is None:
                        self.obstacle_start_time = time.time()
                    
                    if time.time() - self.obstacle_start_time >= 1:
                        print("Pass static obstacle start")
                        self.sign_car_position = None
                        self.state = autoFSMState.DRIVE_OVERTAKE
                elif current_node not in self.overtake_nodes:
                    if self.debugging:
                        print(f"Overtaking not allowed at node {current_node} - not in overtake zone")
                elif abs(angle) > self.max_wheel_angle_for_overtake * 10:
                    if self.debugging:
                        print(f"Overtaking not allowed - wheel angle too high: {abs(angle)/10}°")

        ##############################         FSM            #############################

        if self.state == autoFSMState.PARKING:
            park_angle, speed, module_running = self.parkingController.run(parking_spot_detected, side_sensors)
            if park_angle is not None:
                angle = park_angle

            self.localization.update_position_with_steering(speed / 10, angle / 10, heading)

            if not module_running:
                self.localization.clamp_location_to_graph()
                self.state = autoFSMState.DRIVE
                self.last_parking_exit_time = time.time()  # Update parking exit time
        
        elif self.state == autoFSMState.INTERSECTION:
            angle, speed, module_running = self.intersectionController.getControlData(
                stop_line_present=stop_line_present_close,
                stop_line_slope=stop_line_angle,
                trafficLights=self.traffic_light_states
            )

            self.localization.update_position_with_steering(speed / 10, angle / 10, heading)
            
            if not module_running:
                self.traffic_light_states.clear()
                self.localization.start_new_segment()
                self.state = autoFSMState.DRIVE
                self.laneFollowContrler.restartPid()

        elif self.state == autoFSMState.DRIVE_OVERTAKE:
            overtake_angle, speed, module_running = self.overtakeController.run(False, front_sensors, side_sensors, angle)
            if overtake_angle is not None:
                angle = overtake_angle

            self.localization.update_position_with_steering(speed / 10, angle / 10, heading)

            if not module_running:
                self.localization.clamp_location_to_graph()
                self.state = autoFSMState.DRIVE

        elif self.state == autoFSMState.CROSSWALK:
            angle, speed, module_stoping = self.crosswalkController.control(self.stephanie_position)
            self.localization.update_position(speed / 10)
            if module_stoping:
                self.state = autoFSMState.DRIVE
                self.stephanie_position = None
                self.laneFollowContrler.restartPid()

        elif self.state == autoFSMState.ROUNDABOUT:
            angle, module_stoping = self.roundaboutController.process_frame(self.leftX, self.rightX, self.roundaboutExit_position, self.leftVisible, self.rightVisible)
            speed = 150

            self.localization.update_position_with_steering(speed / 10, angle / 10, heading)

            if module_stoping:
                self.localization.start_new_segment()
                self.state = autoFSMState.DRIVE
                self.laneFollowContrler.restartPid()

        elif self.state == autoFSMState.INTERSECTION_CROSSWALK:
            angle, speed, module_running = self.intersectionCrosswalkController.getControlData(
                stop_line_present=stop_line_present_close,
                stop_line_slope=stop_line_angle,
                trafficLights=self.traffic_light_states,
                stephanie_position=self.stephanie_position,
                crosswalk_sign_present=self.traffic_signs.get_active() == "crosswalk"
            )

            self.localization.update_position_with_steering(speed / 10, angle / 10, heading)
            
            if not module_running:
                self.traffic_light_states.clear()
                self.stephanie_position = None
                self.localization.start_new_segment()
                self.state = autoFSMState.DRIVE
                self.laneFollowContrler.restartPid()

        elif self.state == autoFSMState.CROSSWALK_INTERSECTION:
            angle, speed, module_running = self.crosswalkIntersectionController.getControlData(
                stop_line_present=stop_line_present_close,
                stop_line_slope=stop_line_angle,
                trafficLights=self.traffic_light_states,
                stephanie_position=self.stephanie_position
            )

            self.localization.update_position_with_steering(speed / 10, angle / 10, heading)
            
            if not module_running:
                self.traffic_light_states.clear()
                self.stephanie_position = None
                self.localization.start_new_segment()
                self.state = autoFSMState.DRIVE
                self.laneFollowContrler.restartPid()

        elif self.state == autoFSMState.DRIVE or self.state == autoFSMState.HIGHWAY:
            no_active_sign = self.traffic_signs.get_active() is None and self.traffic_light_states.get_active() is None
            stephanie_crossing = self.stefanija_in_danger_zone and self.traffic_signs.get_active() != "crosswalk"

            speed = self.speedControler.getControlData(
                angle=angle,
                stopLine=stop_line_present_close,
                lowDistance=stop_line_present,
                highway=self.state == autoFSMState.HIGHWAY,
                frontDistance=front_sensors["distance"],
                enable_emergency_stop=no_active_sign,
                car_in_front=self.car_in_danger_zone,
                stephanie_in_front=stephanie_crossing
            )

            self.localization.update_position(speed / 10)
            self.localization.calibrate_heading(heading)

        ############################ Sending data ##############################

        if angle != self.oldAngle:
            self.steerMotorSender.send(f"{angle}")
            self.oldAngle = angle
            # if self.debugging:
            #     self.logging.info(f"New steering angle: {angle}")

        if speed != self.oldSpeed:
            self.speedMotorSender.send(f"{speed}")
            self.oldSpeed = speed
            # if self.debugging:
            #     self.logging.info(f"New speed: {speed}")

        # TODO: Implement historyData sending for detected obstacles
        # Example format: {"type": "historyData", "values": [x, y, TrafficSign.STOP.value]} 
        # Use TrafficSign enum from obstacles.py for identifying obstacle types
        # Send when traffic signs, pedestrians, or other obstacles are detected

        # Send vehicle position and heading to VehicleToEverything
        current_time = time.time()
        if current_time - self.last_sent_time_v2x >= 1.0:  # Check if 1 second has passed
            vehicle_position = self.localization.get_location()
            self.vehicleToEverythingSender.send({
                "type": "devicePos",
                "values": [vehicle_position[0], vehicle_position[1]]
            })
            self.vehicleToEverythingSender.send({
                "type": "deviceRot",
                "values": [heading - self.localization.heading_error]
            })
            self.vehicleToEverythingSender.send({
                "type": "deviceSpeed",
                "values": [speed / 10]
            })
            self.last_sent_time_v2x = current_time
        
        # time.sleep(0.05)

    def getTime(self):
        return round(time.time()*1000)

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.laneDetectSubscriber = messageHandlerSubscriber(self.queuesList, LaneDetect, "LastOnly", True)
        self.stopLineDetectionSubscriber = messageHandlerSubscriber(self.queuesList, StopLineDetect, "LastOnly", True)
        self.objectDetectionSubscriber = messageHandlerSubscriber(self.queuesList, ObjectDetection, "FIFO", True)
        self.trafficSignsSubscriber = messageHandlerSubscriber(self.queuesList, TrafficSignsDetection, "FIFO", True)
        self.sideSensorSubscriber = messageHandlerSubscriber(self.queuesList, SideSensors, "LastOnly", True)
        self.frontSensorSubscriber = messageHandlerSubscriber(self.queuesList, FrontSensors, "LastOnly", True)
        self.parkingSpotDetectionSubscriber = messageHandlerSubscriber(self.queuesList, ParkingSpotDetect, "LastOnly", True)
        self.locationSubscriber = messageHandlerSubscriber(self.queuesList, Location, "LastOnly", True)
        self.headingSubscriber = messageHandlerSubscriber(self.queuesList, Heading, "LastOnly", True)