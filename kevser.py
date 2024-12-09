#190 268 4.161

from pymavlink import mavutil

# Bağlantıyı kur
master = mavutil.mavlink_connection('/dev/serial0')
print("Connection established")

# Heartbeat mesajını bekle
try:
    master.wait_heartbeat(timeout=10)  # 10 saniye bekler
    print("Heartbeat alındı! İHA iletişime hazır.")
except Exception as e:
    print(f"Heartbeat alınamadı: {e}")

battery = master.recv_match(type='BATTERY_STATUS', blocking=True)
print(battery.voltages[0] / 1000)
heartbeat = master.recv_match(type='HEARTBEAT', blocking=True)
print(f"Sistem Durumu: {heartbeat.system_status}")
print(f"Temel Mod: {heartbeat.base_mode}")
print(f"Özel Mod: {heartbeat.custom_mode}")