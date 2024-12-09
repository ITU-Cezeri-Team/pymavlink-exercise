from pymavlink import mavutil
import time

# Start a connection listening on a UDP port


# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
#recv_match
# Bağlantıyı başlat
master = mavutil.mavlink_connection('udpout:192.168.4.161:14540')
print("Connected")

while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    system_status = msg.system_status
    if system_status == mavutil.mavlink.MAV_STATE_ACTIVE:
        print("Cihaz aktif ve uçuşa/harekete hazır!")
    elif system_status == mavutil.mavlink.MAV_STATE_UNINIT:
        print("Cihaz başlatılmamış.")
    else:
        print(f"Diğer durum: {system_status}")