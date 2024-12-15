from pymavlink import mavutil

# Start a connection listening on a UDP port
connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)

# Wait for the first heartbeat to set the system and component ID of remote system for the link
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))