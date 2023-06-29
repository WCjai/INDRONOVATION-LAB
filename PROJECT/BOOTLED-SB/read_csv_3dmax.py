import csv
from pymavlink import mavutil
import time
connection = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
connection.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

mode = 'GUIDED'
mode_id = connection.mode_mapping()[mode]
connection.mav.set_mode_send(connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
response = connection.recv_match(type='COMMAND_ACK', blocking=True)
response = connection.recv_match(blocking=True)

connection.mav.command_long_send(connection.target_system, connection.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 0, 1, 0, 0, 0, 0, 0, 0)
connection.mav.command_long_send(connection.target_system, connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 1)
time.sleep(4)
def send_position_target(x, y, z,time_boot_ms):
    print("x:{},y:{},z:{},TUME:{}".format(x,y,z,time_boot_ms))
    msg = connection.mav.set_position_target_local_ned_encode(
        time_boot_ms=int(time_boot_ms * 1e6),
        target_system=connection.target_system,
        target_component=connection.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask=0b010111111000,  # Set position and velocity
        x=x,
        y=y,
        z=-z,
        vx=0,
        vy=0,
        vz=0,
        afx=0,
        afy=0,
        afz=0,
        yaw=0,
        yaw_rate=0
    )

    connection.mav.send(msg)
    time.sleep(time_boot_ms / 1000)


def read_csv_file(file_path):
    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            x = float(row[1])
            y = float(row[2])
            z = float(row[3])
            time_boot_ms = float(row[4])
            # vx = float(row[5])
            # vy = float(row[6])
            # vz = float(row[7])
            send_position_target(x, y, z,time_boot_ms)


# Provide the path to your CSV file
csv_file_path = 'TEST 1 CSV.csv'

# Call the function to read the CSV file
read_csv_file(csv_file_path)

