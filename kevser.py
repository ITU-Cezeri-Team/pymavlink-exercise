#190 268 4.161

from pymavlink import mavutil

# Bağlantıyı kur
master = mavutil.mavlink_connection('dev/serial0')

# Heartbeat mesajını bekle
master.wait_heartbeat()
print("Heartbeat alındı! İHA iletişime hazır.")

# Alınan heartbeat mesajını incele
heartbeat = master.recv_match(type='HEARTBEAT', blocking=True)
print(f"Sistem Durumu: {heartbeat.system_status}")
print(f"Temel Mod: {heartbeat.base_mode}")
print(f"Özel Mod: {heartbeat.custom_mode}")