from pymavlink import mavutil

# Start a connection listening on a UDP port


# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
#recv_match
# Bağlantıyı başlat
master = mavutil.mavlink_connection('/dev/serial10', baud=57600)
print("Connected")

# Heartbeat mesajlarını dinle
while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    print(f"Received heartbeat message: {msg}")

# Once connected, use 'the_connection' to get and send messages