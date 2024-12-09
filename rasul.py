#rasul
"""192
168
4
113"""



from pymavlink import mavutil

# Bağlantıyı kur
master = mavutil.mavlink_connection('/dev/serial0', baud=57600)
print("Connection established")

arm_control = input("For ARM press Y: ").lower()

if arm_control=="y":
    master.set_mode(mavutil.mavlink.MAV_MODE_STABILIZE_ARMED)
    master.arducopter_arm()
    print("Motorlar aktif edildi.")
else:
    pass 

takeoff_control = input("For take off press Y: ").lower()

if takeoff_control=="y":
    master.set_mode(mavutil.mavlink.MAV_MODE_GUIDED_ARMED)
    print("Switched to GUIDED mode.")
    takeoff_altitude = 10  # 10 metre
    master.mav.command_long_send(
    master.target_system, 
    master.target_component, 
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
    0,  # confirmation (0: no confirmation needed)
    0, 0, 0, 0, 0, 0, takeoff_altitude  # kalkış yüksekliği
    )
    print(f"Takeoff command sent to reach {takeoff_altitude} meters.")
    master.arducopter_disarm()
    print("Motorlar pasif edildi.")












"""
# Heartbeat mesajını bekle
try:
    master.wait_heartbeat(timeout=10)  # 10 saniye bekler
    print("Heartbeat alındı! İHA iletişime hazır.")
except Exception as e:
    print(f"Heartbeat alınamadı: {e}")
    exit()  # Eğer heartbeat alınmazsa programı sonlandır
"""
"""
# Batarya durumunu al
try:
    battery = master.recv_match(type='BATTERY_STATUS', blocking=False)
    print("Battery connected")
    if battery:
        print(f"Batarya Voltajı: {battery.voltages[0] / 1000} V")  # mV -> V dönüşümü
        print(f"Batarya Akımı: {battery.current_battery / 100} A")  # cA -> A dönüşümü
        print(f"Kalan Batarya: {battery.battery_remaining} %")
    else:
        print("Batarya durumu alınamadı.")
except Exception as e:
    print(f"Batarya durumu alınamadı: {e}")
"""
"""
# Heartbeat mesajını tekrar al
try:
    heartbeat = master.recv_match(type='HEARTBEAT', blocking=True)
    print(f"Sistem Durumu: {heartbeat.system_status}")
    print(f"Temel Mod: {heartbeat.base_mode}")
    print(f"Özel Mod: {heartbeat.custom_mode}")
except Exception as e:
    print(f"Heartbeat mesajı alınamadı: {e}")
"""