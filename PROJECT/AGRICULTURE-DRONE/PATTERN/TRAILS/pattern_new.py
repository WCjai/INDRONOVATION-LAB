import time
from pymavlink import mavutil

top_left = (0, 0, 0)
top_right = (10, 0, 0)
bottom_right = (10, 10, 0)
bottom_left = (0, 10, 0)

# Define the survey parameters
survey_altitude = 5  # Survey altitude in meters
survey_distance = 1  # Distance between survey lines in meters

# Calculate the number of survey lines and waypoints per line
num_lines = int((bottom_right[1] - top_left[1]) / survey_distance)
#num_waypoints_per_line = int((bottom_right[0] - top_left[0]) / survey_distance)
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

        waypoint = (x,y,z)
        waypoints.append(waypoint)

# Send the survey coverage waypoints to the vehicle
for waypoint in waypoints:
    print(waypoint)






