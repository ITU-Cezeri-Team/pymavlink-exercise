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

    master.wait_heartbeat(timeout=10)
    print("Heartbeat alındı")
except:
    print("zayıf")

while True:
    master.mav.heartbeat_send(
        6,  # SYSTEM_TYPE_GCS (Yer İstasyonu)
        0,  # COMPONENT_TYPE_SYSTEM (Bileşen türü)
        0,  # Sistem durumu (0 = aktif)
    )
    print("Heartbeat mesajı gönderildi")
    time.sleep(1)  # 1 saniyede bir heartbeat gönder
# Once connected, use 'the_connection' to get and send messages