from pymavlink import mavutil

# Start a connection listening on a UDP port
connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)

# Wait for the first heartbeat to set the system and component ID of remote system for the link
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

# Define command_long_encode message to send MAV_CMD_SET_MESSAGE_INTERVAL command
# param1: MAVLINK_MSG_ID_BATTERY_STATUS (message to stream)
# param2: 1000000 (Stream interval in microseconds)
connection.mav.command_long_send(
    1,
    1,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # ARM komutu
    0,  # Confirmation
    1, # param2: Interval in microseconds
    0,       # param3 (unused)
    0,       # param4 (unused)
    0,       # param5 (unused)
    0,       # param5 (unused)
    0,        # param6 (unused)
    0
    )



# Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result
response = connection.recv_match(type='COMMAND_ACK', blocking=True)
if response and response.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("Command accepted")
else:
    print("Command failed")