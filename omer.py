from pymavlink import mavutil
import time

# MAVLink bağlantısını başlat
connection = mavutil.mavlink_connection('udp:172.20.10.3:14551')  # Bağlantı adresini uygun şekilde değiştirin
print("connected")
msg = connection.recv_match(type='GPS_RAW_INT', blocking=False)
print("gps received")
# Cihazın armed durumda olduğundan emin olun
print("Arming the drone...")
connection.mav.command_long_send(
    1,  # Sistem ID (Genellikle 1)
    1,  # Bileşen ID (Genellikle 1)
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Arming komutu
    0,  # Parametre (0 = disarmed, 1 = armed)
    1,  # Parametre (0 = disarmed, 1 = armed)
    0,
    0,
    0,
    0,
    0,
    0
)

# Birkaç saniye bekleyelim ki cihaz arming işlemini gerçekleştirsin
time.sleep(3)

# Modu değiştir (GUIDED mode)
print("Changing mode to GUIDED...")
connection.mav.set_mode_send(
    1,  # Sistem ID
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # Custom Mode Flag
    0   # GUIDED mode (4: Guided Mode)
)

# Mod değişikliklerinin gerçekleşmesini bekleyelim
time.sleep(2)

# Kalkış komutunu gönder (10 metre yükseklik hedefi)
print("Sending takeoff command...")
connection.mav.command_long_send(
    1,  # Sistem ID
    1,  # Bileşen ID
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Takeoff komutu
    0,  # Parametre (0 = success, 1 = failure)
    0,  # Parametre
    0,  # Parametre
    10,  # Kalkış yüksekliği (10 metre)
    0,  # Parametre
    0,  # Parametre
    0,  # Parametre
    0   # Parametre
)

# Kalkış komutunun gönderildiğini belirten mesaj
print("Takeoff command sent. Drone is taking off to 10 meters.")
