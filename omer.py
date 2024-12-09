from pymavlink import mavutil
import time

# Start a connection listening on a UDP port


# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
#recv_match
# Bağlantıyı başlat
master = mavutil.mavlink_connection('udpout:172.20.10.3:14540')
print("Connected")

master.set_mode(mavutil.mavlink.MAV_MODE_GUIDED_ARMED)
print("Cihaz arming komutu gönderildi.")

master.mav.command_long_send(
    1,  # Sistem ID
    1,  # Bileşen ID
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Takeoff komutu
    0,  # Parametre (0 = success, 1 = failure)
    0,  # Parametre (hareket yönü)
    0,  # Parametre (belirli bir yükseklik hedefi)
    10,  # Kalkış için hedef yükseklik (10 metre)
    0,  # Parametre (yakınsama noktası)
    0,  # Parametre (yer değiştirme yönü)
    0,  # Parametre
    0   # Parametre
)
master.arducopter_disarm()

print("Kalkış komutu gönderildi. Drone 10 metreye çıkacak.")

while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    system_status = msg.system_status
    if system_status == mavutil.mavlink.MAV_STATE_ACTIVE:
        print("Drone aktif ve uçuşa geçti.")
    else:
        print(f"Sistem durumu: {system_status}")
'''
master.mav.command_long_send(
    1,  # Sistem ID
    1,  # Bileşen ID
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Disarming komutu
    0,  # 0 = disarmed
    0,  # Parametreler
    0,
    0,
    0,
    0,
    0,
    0
)
print("Cihaz disarming komutu gönderildi.")
'''