'''import time
from pymavlink import mavutil


# MAVLink bağlantısını başlat
connection = mavutil.mavlink_connection('udp:192.168.4.113:14550')  # Bağlantı adresini uygun şekilde değiştirin
print("connected")
msg = connection.recv_match(type='GPS_RAW_INT', blocking=False)
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

# Function to arm the drone
def arm_drone():
    print("Arming the drone...")
    connection.mav.command_long_send(
        1,  # System ID
        1,  # Component ID
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Arm command
        0,  # Confirmation (0 = no confirmation)
        1,  # Armed (1 = armed, 0 = disarmed)
        0,  # Unused
        0,  # Unused
        0,  # Unused
        0,  # Unused
        0,  # Unused
        0   # Unused
    )
    if mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        print("Drone armed.")

    time.sleep(3)  # Wait for arming to complete

# Function to change the drone's mode to stabilize
def stabilize_drone():
    print("Changing drone mode to stabilize...")
    connection.mav.set_mode_send(
        1,  # System ID
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # Custom mode enabled
        mavutil.mavlink.MAV_MODE_GUIDED_ARMED  # Set mode to GUIDED, which is stabilized flight
    )
    time.sleep(2)  # Wait for the drone to switch modes
    print("Drone is stabilized and in GUIDED mode.")

# Function to send takeoff command
def takeoff_drone(altitude=10):
    print(f"Taking off to {altitude} meters.")
    connection.mav.command_long_send(
        1,  # System ID
        1,  # Component ID
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Takeoff command
        0,  # Confirmation (0 = no confirmation)
        0,  # Latitude (not used)
        0,  # Longitude (not used)
        altitude,  # Target altitude (in meters)
        0,  # Yaw angle (0 = no change)
        0,  # Reserved
        0,  # Reserved
        0   # Reserved
    )
    time.sleep(5)  # Wait for takeoff to begin
    print("Drone is in the air.")

# Function to send land command
def land_drone():
    print("Landing the drone...")
    connection.mav.command_long_send(
        1,  # System ID
        1,  # Component ID
        mavutil.mavlink.MAV_CMD_NAV_LAND,  # Land command
        0,  # Confirmation (0 = no confirmation)
        0,  # Latitude (not used)
        0,  # Longitude (not used)
        0,  # Altitude (not used)
        0,  # Reserved
        0,  # Reserved
        0,  # Reserved
        0   # Reserved
    )
    print("Drone is landing.")

# Ask the user if they are ready for takeoff
ready_for_takeoff = input("Are you ready for takeoff? (yes/no): ").strip().lower()

if ready_for_takeoff == "yes":
    stabilize_drone()
    time.sleep(2)

    # Arm the drone
    arm_drone()

    # Stabilize the drone before takeoff

    # Takeoff the drone to 10 meters
    takeoff_drone(altitude=10)

    # Listen for the 'Y' key to land
    while True:
        user_input = input("Drone is in the air. Press 'Y' to land the drone: ").strip().lower()
        if user_input == "y":
            # Land the drone if 'Y' is pressed
            land_drone()
            break  # Exit the loop after landing
else:
    print("Takeoff aborted.")
'''

from pymavlink import mavutil
import time

# MAVLink bağlantısını başlat
connection = mavutil.mavlink_connection('udp:192.168.4.113:14550')  # Bağlantı adresini uygun şekilde değiştirin
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
        0,  # Unused
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
    msg = connection.recv_match(type='HEARTBEAT', blocking=False)
    
    if msg:
        # ARM durumunu kontrol et
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Drone is ARMED")
        else:
            print("Drone is DISARMED")
    else:
        print("No HEARTBEAT message received")

# Bağlantıyı bekle
connection.wait_heartbeat()
print("Heartbeat received")

# Force arm komutunu gönder
force_arm_drone()

# Arm durumunu kontrol et
check_if_armed()
