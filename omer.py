

from pymavlink import mavutil

# Bağlantıyı kur
master = mavutil.mavlink_connection('/dev/serial0', baud=57600)
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

