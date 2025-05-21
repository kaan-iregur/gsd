from pymavlink import mavutil

#def create_connection(device: str = '/dev/ttyAMA0', baud: int = 57600):
def create_connection(device: str = 'udp://127.0.0.1:14550', baud: int = 57600):
  
    """
    MAVLink üzerinden Pixhawk ile bağlantı kurar ve heartbeat bekler.
    """
    master = mavutil.mavlink_connection(device, baud=baud)
    print("Bağlantı bekleniyor...")
    master.wait_heartbeat()
    print(f"Bağlandı: Sistemi ID={master.target_system}, Komponent ID={master.target_component}")
    return master
