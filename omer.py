from pymavlink import mavutil
import time

# Start a connection listening on a UDP port


# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
#recv_match
# Bağlantıyı başlat
master = mavutil.mavlink_connection('udpout:192.168.4.161:14540')
print("Connected")

master.mav.command_long_send(
    1,  # Sistem ID (burada 1 genellikle drone'dur)
    1,  # Bileşen ID (genellikle 1)
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Arming komutu
    0,  # 0 = disarmed, 1 = armed
    0,  # Parametreler (ihtiyaç yok)
    0,
    0,
    0,
    0,
    0,
    0
)
print("Cihaz arming komutu gönderildi.")