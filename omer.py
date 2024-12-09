import time
import threading
from pymavlink import mavutil


# MAVLink bağlantısını başlat
master = mavutil.mavlink_connection('udp:192.168.4.113:14551')  # Bağlantı adresini uygun şekilde değiştirin
print("connected")
msg = master.recv_match(type='GPS_RAW_INT', blocking=False)
print("gps received")
if msg:
        # Extract the GPS information from the message
        lat = msg.lat / 1e7  # Convert to degrees (lat is in 1e7 degrees)
        lon = msg.lon / 1e7  # Convert to degrees (lon is in 1e7 degrees)
        alt = msg.alt / 1000  # Convert to meters (alt is in mm)
        
        fix_type = msg.fix_type  # GPS fix type (0 = no fix, 1 = 2D fix, 2 = 3D fix, etc.)

        # Print the GPS data
        print(f"Latitude: {lat}°")
        print(f"Longitude: {lon}°")
        print(f"Altitude: {alt} meters")
        print(f"Fix Type: {fix_type}")
        
        if fix_type == 3:
            print("GPS 3D fix is available!")
        elif fix_type == 2:
            print("GPS 2D fix is available!")
        else:
            print("No GPS fix.")

# print("gps received")

# Bağlantıyı kur
print("Connection established")

arm_control = input("For ARM press Y: ").lower()

if arm_control=="y":
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


