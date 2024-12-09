import time
import threading
from pymavlink import mavutil


# MAVLink bağlantısını başlat
connection = mavutil.mavlink_connection('udp:192.168.4.113:14551')  # Bağlantı adresini uygun şekilde değiştirin
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

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()
print("Heartbeat received, connection successful.")

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
    time.sleep(3)  # Wait for arming to complete
    print("Drone armed.")

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
    # Arm the drone
    arm_drone()

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
