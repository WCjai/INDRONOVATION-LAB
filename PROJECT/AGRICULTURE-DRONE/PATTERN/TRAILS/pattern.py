import time
from pymavlink import mavutil
from geographiclib.geodesic import Geodesic
import math


def calculate_new_coordinates(latitude, longitude, x_distance, y_distance):
    earth_radius = 6371000  # Radius of the Earth in meters

    # Convert distances to radians
    delta_latitude = y_distance / earth_radius
    delta_longitude = x_distance / (earth_radius * math.cos(math.radians(latitude)))

    # Convert radians to degrees
    new_latitude = latitude + math.degrees(delta_latitude)
    new_longitude = longitude + math.degrees(delta_longitude)

    return new_latitude, new_longitude


# Connect to the vehicle
the_connection = mavutil.mavlink_connection('udp:0.0.0.0:14550')

# Wait for the heartbeat
the_connection.wait_heartbeat()

# Set the target system and component IDs
target_system = the_connection.target_system
target_component = the_connection.target_component
latitude = -35.36084944
longitude = 149.16179323

# Define the square area coordinates
# The square coordinates are in the local NED frame
# Adjust the values as per your requirements
top_left = (0, 0, 0)
top_right = (10, 0, 0)
bottom_right = (10, 10, 0)
bottom_left = (0, 10, 0)

# Define the survey parameters
survey_altitude = 5  # Survey altitude in meters
survey_distance = 1  # Distance between survey lines in meters

# Calculate the number of survey lines and waypoints per line
num_lines = int((bottom_right[1] - top_left[1]) / survey_distance)
num_waypoints_per_line = 1

# Generate the survey coverage waypoints
waypoints = []

for line in range(num_lines):
    # Determine the direction of the survey line (left to right or right to left)
    if line % 2 == 0:
        start = top_left[0]
        end = bottom_right[0] + survey_distance
        step = survey_distance
    else:
        start = bottom_right[0]
        end = top_left[0] - survey_distance
        step = -survey_distance

    # Generate waypoints for the current survey line
    for i in range(num_waypoints_per_line + 1):
        x = start + i * step
        y = top_left[1] + line * survey_distance
        z = survey_altitude

        x_distance = x  # Distance in meters to the right
        y_distance = y  # Distance in meters to the top
        new_latitude, new_longitude = calculate_new_coordinates(latitude, longitude, x_distance, y_distance)

        print("({}, {}, 50)".format(new_latitude,new_longitude))

        waypoint = mavutil.mavlink.MAVLink_mission_item_int_message(
            target_system=target_system,
            target_component=target_component,
            seq=len(waypoints),  # Sequence number
            frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            current=0,  # Not the current waypoint
            autocontinue=1,  # Autocontinue to the next waypoint
            param1=0,  # Hold time in seconds
            param2=0,  # Acceptance radius in meters
            param3=0,  # Pass-through parameter for the specific command
            param4=0,  # Pass-through parameter for the specific command
            x=int(new_latitude * 10 ** 7),  # Latitude in degrees * 1e7
            y=int(new_longitude * 10 ** 7),  # Longitude in degrees * 1e7
            z = 0,  # Altitude in meters * 1000
        )
        waypoints.append(waypoint)

# Send the MISSION_COUNT message to indicate the number of mission items
mission_count = mavutil.mavlink.MAVLink_mission_count_message(
    target_system=target_system,
    target_component=target_component,
    count=len(waypoints)
)
the_connection.mav.send(mission_count)
time.sleep(1)  # Wait for the message to be sent

# Send the MISSION_ITEM_INT messages to upload the mission items
for waypoint in waypoints:
    the_connection.mav.send(waypoint)
    time.sleep(0.2)  # Delay between each mission item

# End the mission
end_mission = mavutil.mavlink.MAVLink_mission_item_int_message(
    target_system=target_system,
    target_component=target_component,
    seq=len(waypoints),  # Sequence number
    frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    command=mavutil.mavlink.MAV_CMD_NAV_LAND,
    current=0,  # Not the current waypoint
    autocontinue=1,  # Autocontinue to the next waypoint
    param1=0,  # Hold time in seconds
    param2=0,  # Acceptance radius in meters
    param3=0,  # Pass-through parameter for the specific command
    param4=0,  # Pass-through parameter for the specific command
    x=0,  # Latitude not used for LAND command
    y=0,  # Longitude not used for LAND command
    z=0  # Altitude not used for LAND command
)
the_connection.mav.send(end_mission)
