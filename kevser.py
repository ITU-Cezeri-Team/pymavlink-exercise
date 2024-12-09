from pymavlink import mavutil

# Port aralığında deneme yap
ip_address = 'dev/serial0'
for port in range(14550, 14556):
    try:
        print(f"Bağlanmayı deniyor: udp:{ip_address}:{port}")
        master = mavutil.mavlink_connection(f'udp:{ip_address}:{port}')
        master.wait_heartbeat(timeout=5)
        print(f"Bağlantı başarılı! Port: {port}")
        break
    except Exception as e:
        print(f"Port {port} başarısız.")
