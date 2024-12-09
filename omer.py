import time
import threading
from pynput import keyboard
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

# Wait for heartbeat to make sure we're connected
connection.wait_heartbeat()
print("Heartbeat received, connection successful.")

# Function to listen for 'Y' key press to land
def listen_for_land_key():
    while True:
        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

# Handle key press event
def on_press(key):
    try:
        if key.char == 'y':  # Check if the 'Y' key was pressed
            print("Y key pressed, landing the drone.")
            land_drone()
    except AttributeError:
        pass

# Function to arm the drone
def arm_drone():
    print("Arming the drone...")
    connection.mav.command_long_send(
        1,  # System ID
        1,  # Component ID
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Arming command
        0,  # Unused
        1,  # Armed (1 = armed, 0 = disarmed)
        0,  # Unused
        0,  # Unused
        0,  # Unused
        0,  # Unused
        0,  # Unused
        0   # Unused
    )
    time.sleep(3)  # Wait a moment for arming to complete
    print("Drone armed.")

# Function to send takeoff command
def takeoff_drone(altitude=10):
    print(f"Taking off to {altitude} meters.")
    connection.mav.command_long_send(
        1,  # System ID
        1,  # Component ID
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Takeoff command
        0,  # Confirmation (0 for no confirmation)
        0,  # Latitude (not used)
        0,  # Longitude (not used)
        altitude,  # Target altitude in meters
        0,  # Yaw angle (0 = no change)
        0,  # Reserved
        0,  # Reserved
        0   # Reserved
    )
    time.sleep(5)  # Wait a moment for takeoff to begin
    print("Drone is in the air.")

# Function to land the drone
def land_drone():
    print("Landing the drone...")
    connection.mav.command_long_send(
        1,  # System ID
        1,  # Component ID
        mavutil.mavlink.MAV_CMD_NAV_LAND,  # Land command
        0,  # Confirmation (0 for no confirmation)
        0,  # Latitude (not used)
        0,  # Longitude (not used)
        0,  # Altitude (not used)
        0,  # Reserved
        0,  # Reserved
        0,  # Reserved
        0   # Reserved
    )
    print("Drone is landing.")

# Ask user if the drone is ready for takeoff
ready_for_takeoff = input("Are you ready for takeoff? (y/n): ").lower()

if ready_for_takeoff == "y":
    # Arm the drone
    arm_drone()

    # Takeoff the drone to 10 meters
    takeoff_drone(altitude=10)

    # Start listening for the 'Y' key to land
    land_listener_thread = threading.Thread(target=listen_for_land_key)
    land_listener_thread.daemon = True  # Allow the thread to be killed when the program ends
    land_listener_thread.start()

    print("Press 'Y' to land the drone.")

    # Keep the program running
    while True:
        time.sleep(1)
else:
    print("Takeoff aborted.")
