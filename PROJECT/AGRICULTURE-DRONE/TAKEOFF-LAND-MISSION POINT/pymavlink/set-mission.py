"""
    Send MISSION_COUNT message to the vehicle first
    Vehicle will respond to this with MISSION_REQUEST message
    This message contains requested mission item sequence number
    Respond to this message with MISSION_ITEM_INT message as soon as possible
    Vehicle will wait and re-request the MISSION_ITEM_INT messages with limited time and timeout
    After sending the last mission item, vehicle will send MISSION_ACK message

    https://mavlink.io/en/messages/common.html#MISSION_COUNT
    https://mavlink.io/en/messages/common.html#MISSION_REQUEST
    https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT
    https://mavlink.io/en/messages/common.html#MISSION_ACK
"""

import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

# create mission item list
target_locations = ((-35.3650810845,149.1611074465,30),
(-35.3650810845,149.16187987950002,30),
(-35.3650810845,149.1626523125,30),
(-35.3650810845,149.1634247455,30),
(-35.3650810845,149.16419717850002,30),
(-35.3650810845,149.16496961150003,30),
(-35.3650810845,149.1657420445,30),
(-35.3650810845,149.16651447750002,30),
(-35.3650810845,149.16728691050002,30),
(-35.3650810845,149.1680593435,30),
(-35.3645421135,149.1680593435,30),
(-35.3645421135,149.16728691050002,30),
(-35.3645421135,149.16651447750002,30),
(-35.3645421135,149.1657420445,30),
(-35.3645421135,149.16496961150003,30),
(-35.3645421135,149.16419717850002,30),
(-35.3645421135,149.1634247455,30),
(-35.3645421135,149.1626523125,30),
(-35.3645421135,149.16187987950002,30),
(-35.3645421135,149.1611074465,30),
(-35.364003142499996,149.1611074465,30),
(-35.364003142499996,149.16187987950002,30),
(-35.364003142499996,149.1626523125,30),
(-35.364003142499996,149.1634247455,30),
(-35.364003142499996,149.16419717850002,30),
(-35.364003142499996,149.16496961150003,30),
(-35.364003142499996,149.1657420445,30),
(-35.364003142499996,149.16651447750002,30),
(-35.364003142499996,149.16728691050002,30),
(-35.364003142499996,149.1680593435,30),
(-35.3634641715,149.1680593435,30),
(-35.3634641715,149.16728691050002,30),
(-35.3634641715,149.16651447750002,30),
(-35.3634641715,149.1657420445,30),
(-35.3634641715,149.16496961150003,30),
(-35.3634641715,149.16419717850002,30),
(-35.3634641715,149.1634247455,30),
(-35.3634641715,149.1626523125,30),
(-35.3634641715,149.16187987950002,30),
(-35.3634641715,149.1611074465,30),
(-35.3629252005,149.1611074465,30),
(-35.3629252005,149.16187987950002,30),
(-35.3629252005,149.1626523125,30),
(-35.3629252005,149.1634247455,30),
(-35.3629252005,149.16419717850002,30),
(-35.3629252005,149.16496961150003,30),
(-35.3629252005,149.1657420445,30),
(-35.3629252005,149.16651447750002,30),
(-35.3629252005,149.16728691050002,30),
(-35.3629252005,149.1680593435,30),
(-35.362386229500004,149.1680593435,30),
(-35.362386229500004,149.16728691050002,30),
(-35.362386229500004,149.16651447750002,30),
(-35.362386229500004,149.1657420445,30),
(-35.362386229500004,149.16496961150003,30),
(-35.362386229500004,149.16419717850002,30),
(-35.362386229500004,149.1634247455,30),
(-35.362386229500004,149.1626523125,30),
(-35.362386229500004,149.16187987950002,30),
(-35.362386229500004,149.1611074465,30),
(-35.3618472585,149.1611074465,30),
(-35.3618472585,149.16187987950002,30),
(-35.3618472585,149.1626523125,30),
(-35.3618472585,149.1634247455,30),
(-35.3618472585,149.16419717850002,30),
(-35.3618472585,149.16496961150003,30),
(-35.3618472585,149.1657420445,30),
(-35.3618472585,149.16651447750002,30),
(-35.3618472585,149.16728691050002,30),
(-35.3618472585,149.1680593435,30),
(-35.3613082875,149.1680593435,30),
(-35.3613082875,149.16728691050002,30),
(-35.3613082875,149.16651447750002,30),
(-35.3613082875,149.1657420445,30),
(-35.3613082875,149.16496961150003,30),
(-35.3613082875,149.16419717850002,30),
(-35.3613082875,149.1634247455,30),
(-35.3613082875,149.1626523125,30),
(-35.3613082875,149.16187987950002,30),
(-35.3613082875,149.1611074465,30),
(-35.360769316500004,149.1611074465,30),
(-35.360769316500004,149.16187987950002,30),
(-35.360769316500004,149.1626523125,30),
(-35.360769316500004,149.1634247455,30),
(-35.360769316500004,149.16419717850002,30),
(-35.360769316500004,149.16496961150003,30),
(-35.360769316500004,149.1657420445,30),
(-35.360769316500004,149.16651447750002,30),
(-35.360769316500004,149.16728691050002,30),
(-35.360769316500004,149.1680593435,30),
(-35.3602303455,149.1680593435,30),
(-35.3602303455,149.16728691050002,30),
(-35.3602303455,149.16651447750002,30),
(-35.3602303455,149.1657420445,30),
(-35.3602303455,149.16496961150003,30),
(-35.3602303455,149.16419717850002,30),
(-35.3602303455,149.1634247455,30),
(-35.3602303455,149.1626523125,30),
(-35.3602303455,149.16187987950002,30),
(-35.3602303455,149.1611074465,30))

# connect to vehicle
vehicle = utility.mavlink_connection(device="udpin:0.0.0.0:14550")

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# create mission count message
message = dialect.MAVLink_mission_count_message(target_system=vehicle.target_system,
                                                target_component=vehicle.target_component,
                                                count=len(target_locations) + 2,
                                                mission_type=dialect.MAV_MISSION_TYPE_MISSION)

# send mission count message to the vehicle
vehicle.mav.send(message)

# this loop will run until receive a valid MISSION_ACK message
while True:

    # catch a message
    message = vehicle.recv_match(blocking=True)

    # convert this message to dictionary
    message = message.to_dict()

    # check this message is MISSION_REQUEST
    if message["mavpackettype"] == dialect.MAVLink_mission_request_message.msgname:

        # check this request is for mission items
        if message["mission_type"] == dialect.MAV_MISSION_TYPE_MISSION:

            # get the sequence number of requested mission item
            seq = message["seq"]

            # create mission item int message
            if seq == 0:
                # create mission item int message that contains the home location (0th mission item)
                message = dialect.MAVLink_mission_item_int_message(target_system=vehicle.target_system,
                                                                   target_component=vehicle.target_component,
                                                                   seq=seq,
                                                                   frame=dialect.MAV_FRAME_GLOBAL,
                                                                   command=dialect.MAV_CMD_NAV_WAYPOINT,
                                                                   current=0,
                                                                   autocontinue=0,
                                                                   param1=0,
                                                                   param2=0,
                                                                   param3=0,
                                                                   param4=0,
                                                                   x=0,
                                                                   y=0,
                                                                   z=0,
                                                                   mission_type=dialect.MAV_MISSION_TYPE_MISSION)

            # send takeoff mission item
            elif seq == 1:

                # create mission item int message that contains the takeoff command
                message = dialect.MAVLink_mission_item_int_message(target_system=vehicle.target_system,
                                                                   target_component=vehicle.target_component,
                                                                   seq=seq,
                                                                   frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                                   command=dialect.MAV_CMD_NAV_TAKEOFF,
                                                                   current=0,
                                                                   autocontinue=0,
                                                                   param1=0,
                                                                   param2=0,
                                                                   param3=0,
                                                                   param4=0,
                                                                   x=0,
                                                                   y=0,
                                                                   z=target_locations[0][2],
                                                                   mission_type=dialect.MAV_MISSION_TYPE_MISSION)

            # send target locations to the vehicle
            else:

                # create mission item int message that contains a target location
                message = dialect.MAVLink_mission_item_int_message(target_system=vehicle.target_system,
                                                                   target_component=vehicle.target_component,
                                                                   seq=seq,
                                                                   frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                                   command=dialect.MAV_CMD_NAV_WAYPOINT,
                                                                   current=0,
                                                                   autocontinue=0,
                                                                   param1=0,
                                                                   param2=0,
                                                                   param3=0,
                                                                   param4=0,
                                                                   x=int(target_locations[seq - 2][0] * 1e7),
                                                                   y=int(target_locations[seq - 2][1] * 1e7),
                                                                   z=target_locations[seq - 2][2],
                                                                   mission_type=dialect.MAV_MISSION_TYPE_MISSION)

            # send the mission item int message to the vehicle
            vehicle.mav.send(message)

    # check this message is MISSION_ACK
    elif message["mavpackettype"] == dialect.MAVLink_mission_ack_message.msgname:

        # check this acknowledgement is for mission and it is accepted
        if message["mission_type"] == dialect.MAV_MISSION_TYPE_MISSION and \
                message["type"] == dialect.MAV_MISSION_ACCEPTED:
            # break the loop since the upload is successful
            print("Mission upload is successful")
            break
