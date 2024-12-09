from pymavlink import mavutil

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:192.168.4.161:14540')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()

print("ok")
print(the_connection.recv_match().to_dict())

# Once connected, use 'the_connection' to get and send messages