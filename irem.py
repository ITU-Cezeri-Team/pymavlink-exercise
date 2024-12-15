from pymavlink import mavutil

drone_connection= mavutil.mavlink_connection('/dev/serial0', baud=57600)
print("connection is completed successfully")

print(drone_connection.wait_heartbeat())

drone_connection.mav.command_long_send(drone_connection.target_system,  # Hedef sistem kimliği
                                         drone_connection.target_component,  # Hedef bileşen kimliği
                                         mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, 5, 0, 0, 0, 0, 0)

armcontrol = input("for ARM enter Y").lower()

if armcontrol == 'y':
    drone_connection.mav.command_long_send(drone_connection.target_system, 
                                        drone_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)


disarmcontrol = input("for DİSARM enter Y").lower()

if disarmcontrol == 'y':
    drone_connection.mav.command_long_send(drone_connection.target_system, 
                                        drone_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

take_off_control = input("for TAKE OFF enter Y").lower()

if take_off_control == 'y': 
    drone_connection.mav.command_long_send(drone_connection.target_system,
                                        drone_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)

land_control = input("for LAND enter Y").lower()

if land_control == 'y': 
    drone_connection.mav.command_long_send( drone_connection.target_system,           
                                        drone_connection.target_component,        
                                        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

msg = drone_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

