from pymavlink import mavutil

# Start a connection listening on a UDP port
connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)

# Wait for the first heartbeat to set the system and component ID of remote system for the link
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

# ARM
connection.mav.command_long_send(connection.target_system, connection.target_component
                                 , mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,1,0,0,0,0,0,0)

msg = connection.recv_match(type="COMMAND_ACK",blocking=True)
print(msg)


# TAKE OFF
connection.mav.command_long_send(connection.target_system, connection.target_component
                                 , mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,0,0,0,0,0,0,10)

msg = connection.recv_match(type="COMMAND_ACK",blocking=True)
print(msg)

connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, connection.target_system, 
                                                                                  connection.target_component,
                                                                                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,int(0b110111111000), 20,0,-10,0,0,0,0,0,0,0,0))

while 1:
    msg = connection.recv_match(
        type="MAV_CONTROLLER_OUTPUT", blocking=True)
    print(msg)

# LEND

connection.mav.command_long_send( connection.target_system,           
                                connection.target_component,        
                                        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

msg = connection.recv_match(type="COMMAND_ACK",blocking=True)
print(msg)