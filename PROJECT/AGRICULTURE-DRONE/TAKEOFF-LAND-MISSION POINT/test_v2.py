import sys
from pymavlink import mavutil
import time
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()

set_lat = int( -35.35472512   * 10 ** 7) #latitude  int(<HERE>* 10 ** 7)
set_lon = int(149.15396143 * 10 ** 7)  #longitude 
set_alt = 10                           #set height to reach in m, before movement


height = 0
lat_error = 0
lon_error = 0

def set_mode():
    mode = 'GUIDED'
    if mode not in master.mode_mapping():
         print('Unknown mode : {}'.format(mode))
         print('Try:', list(master.mode_mapping().keys()))
         sys.exit(1)

    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)
    response = master.recv_match (type='COMMAND_ACK', blocking=True)

    while not response.result == 0:
        time.sleep(1)
    return True

def arm_and_takeoff():
    global height
    global set_alt
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0, set_alt)
    response = master.recv_match (type='COMMAND_ACK', blocking=True)
    if response.result !=  0:
        print ("failed arming")
    while(True):
          height = master.recv_match (type='LOCAL_POSITION_NED', blocking=True)
          print("height reached: {} m".format(abs(height.z)))
          if(abs(height.z) < (set_alt - 1)):
              continue
          else:
             return True
          
#    while height > 5:
#        return True
#    do:
#        height = int(abs(int(master.messages['LOCAL_POSITION_NED'].z)))
#
#         print(abs(int(master.messages['LOCAL_POSITION_NED'].z)))
#        time.sleep(1)
    

def go_to():
    global set_lat 
    global set_lon 
    global lat_error
    global lon_error
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, master.target_system,
                        master.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, int(0b110111111000), set_lat, set_lon, 10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))

    while(True):
            lat = master.recv_match (type='GLOBAL_POSITION_INT', blocking=True)
            lon = master.recv_match (type='GLOBAL_POSITION_INT', blocking=True)
            lat_error = abs(abs(set_lat) - abs(int(lat.lat)))
            lon_error = abs(abs(set_lon) - abs(int(lon.lon)))
            print("PRINTING LATITUDE AND LONGITUDE Err...")
            if(lat_error > 3 and lon_error > 3):
                 print("Lat: {} , Lon: {}".format(lat_error,lon_error))
                 continue
            else:
                 return True
          
#    while not (lat_error < 3 and lon_error < 3):
#        lat = master.recv_match (type='GLOBAL_POSITION_INT', blocking=True)
#        lon = master.recv_match (type='GLOBAL_POSITION_INT', blocking=True)
#        lat_error = abs(int(-35.3629849 * 10 ** 7)) - abs(int(lat.lat))
#        lon_error = abs(int(149.1649185 * 10 ** 7)) - abs(int(lon.lon))
#        time.sleep(1)
#        print("Latitude: {} , Longitude: {}".format(lat_error,lon_error))
#    time.sleep(10)
#    return True

def land():
    global set_lat
    global set_lon

    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0, set_lat, set_lon, 0)
    return True

# Execute functions sequentially
if set_mode():
    if arm_and_takeoff():
        if go_to():
            if land():
               print("All functions executed successfully.")
            else:
                print("LANDING failed")
        else:
            print("failed.")
    else:
        print("ARM/TAKEOFF failed.")
else:
    print("failed to set GUIDED mode")