from pymavlink import mavutil
import time

# Start a connection listening on a UDP port


# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
#recv_match
# Bağlantıyı başlat
master = mavutil.mavlink_connection('udpout:192.168.4.161:14540')
print("Connected")
try:

    master.wait_heartbeat(timeout=1)
    print("Heartbeat alındı")
except:
    print("zayıf")

while True:
    # Gelen mesajı al
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    print(f"Gelen Heartbeat mesajı: {msg}")