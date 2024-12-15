from pymavlink import mavutil
import time

# MAVLink bağlantısını başlat
connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)

print("Connected to MAVLink")

# Drone'u arm etme fonksiyonu (force arm)
def force_arm_drone():
    print("Sending force arm command to the drone...")
    connection.mav.command_long_send(
        1,  # System ID (genellikle 1)
        1,  # Component ID (genellikle 1)
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Arm komutu
        0,  # Confirmation (0 = no confirmation)
        1,  # Armed (1 = armed, 0 = disarmed)
        21196,  # Unused
        0,  # Unused
        0,  # Unused
        0,  # Unused
        0,  # Unused
        0   # Unused
    )
    time.sleep(3)  # Arming'in tamamlanması için bekle
    print("Drone force armed.")

# Drone'un arm durumu kontrolü
def check_if_armed():
    # HEARTBEAT mesajı al
    msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout = 5)
    
    if msg:
        # ARM durumunu kontrol et
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Drone is ARMED")
        else:
            print("Drone is DISARMED")
    else:
        print("No HEARTBEAT message received")

# Bağlantıyı bekle
print("Heartbeat received")

# Force arm komutunu gönder
force_arm_drone()

# Arm durumunu kontrol et
check_if_armed()