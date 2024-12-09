from pymavlink import mavutil
import time

# Replace with your connection string. Here, I'm using a serial connection as an example.
# For example, replace 'COM3' with the actual port name on your computer, or use 'udp:127.0.0.1:14550' for UDP connection.
  # for Linux/OS X, 'COM3' for Windows
baud_rate = 57600  # Standard baud rate for most flight controllers

# Create a connection
master = mavutil.mavlink_connection('udp:192.168.4.113:14550')

# Wait for the heartbeats to confirm the connection
print("Waiting for heartbeat from the drone...")
print("Heartbeat from system (System ID: %d, Component ID: %d)" % (master.target_system, master.target_component))

# Request basic information (for example, GPS data)        
master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)

# Let's print the GPS data


# Arm the drone
print("Arming the drone...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
)

time.sleep(5)

# Disarm the drone (optional, you can skip this if you don't want to disarm after arming)
print("Disarming the drone...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
)

# Close the connection
master.close()
