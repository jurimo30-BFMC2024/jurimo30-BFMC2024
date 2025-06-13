from enum import Enum, unique

@unique
class Obstacles(Enum):
    """Enum class containing all traffic signs and obstacles"""
    
    STOP = 1
    PRIORITY = 2
    PARKING = 3
    CROSSWALK = 4
    HIGHWAY_ENTRANCE = 5
    HIGHWAY_EXIT = 6
    ROUNDABOUT = 7
    ONE_WAY = 8
    NO_ENTRY = 9
    PARKED_CAR = 10
    PEDESTRIAN_CROSSWALK = 11
    PEDESTRIAN_ROAD = 12
    ROADBLOCK = 13
    TRAFFIC_LIGHT = 14
    FOG = 15
    TUNNEL = 16
    RAMP = 17  # Note: In the original list, both Tunnel and Ramp had ID 16. I assigned 17 to Ramp.

    @classmethod
    def get_description(cls, id):
        """Get the description of a traffic sign by its ID"""
        descriptions = {
            1: "Traffic Sign - Stop",
            2: "Traffic Sign - Priority",
            3: "Traffic Sign - Parking",
            4: "Traffic Sign - Crosswalk",
            5: "Traffic Sign - Highway entrance",
            6: "Traffic Sign - Highway exit",
            7: "Traffic Sign - Roundabout",
            8: "Traffic Sign - One way road",
            9: "Traffic Sign - No Entry",
            10: "Static car on parking",
            11: "Pedestrian on crosswalk",
            12: "Pedestrian on road",
            13: "Roadblock",
            14: "Traffic light",
            15: "Fog",
            16: "Tunnel",
            17: "Ramp"
        }
        return descriptions.get(id, "Unknown sign")

if __name__ == "__main__":
    # Test the enum
    print(f"Stop sign ID: {TrafficSign.STOP.value}")
    print(f"Description: {TrafficSign.get_description(TrafficSign.STOP.value)}")
    
    # Print all signs
    print("\nAll traffic signs:")
    for sign in TrafficSign:
        print(f"{sign.name}: {sign.value} - {TrafficSign.get_description(sign.value)}")
