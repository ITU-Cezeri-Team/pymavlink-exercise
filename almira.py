from pymavlink import mavutil

connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)

arm_control = input("Press y to arm: ").lower()

if arm_control == "y":
    connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

    connection.set_mode(mavutil.mavlink.MAV_MODE_STABILIZE_ARMED)
    print("Switched to GUIDED mode.")
    # to disarm change 1 to 0.
    connection.mav.command_long_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    msg = connection.recv_match('COMMAND_ACK', blocking=True)
    print(msg)