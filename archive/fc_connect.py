from pymavlink import mavutil
import random

# Pixhawk’a seri bağlantı başlat (TELEM1 portu /dev/serial0 olabilir)
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)

# MAVLink bağlantısını kurmak için "heartbeat" bekle
print("Bağlantı bekleniyor...")
master.wait_heartbeat()
print(f"Bağlandı: Sistemi ID={master.target_system}, Komponent ID={master.target_component}")

def oku_telemetri():
    while True:
        # Her döngüde rastgele bir PWM değeri (1000–2000 µs) gönder
     
        # Telemetri mesajlarını oku ve işle
        msg = master.recv_match(blocking=True)
        if not msg:
            continue

        msg_type = msg.get_type()

        if msg_type == "BATTERY_STATUS":
            volt = msg.voltages[0] / 1000.0  # mV to V
            current = msg.current_battery / 100.0  # cA to A
            print(f"🔋 Batarya: {volt:.2f} V, {current:.2f} A")

        elif msg_type == "ALTITUDE":
            print(f"📏 Yükseklik: {msg.altitude_relative/1000:.2f} m")

        elif msg_type == "GPS_RAW_INT":
            print(f"📡 GPS: lat={msg.lat/1e7:.7f}, lon={msg.lon/1e7:.7f}, alt={msg.alt/1000:.2f} m")

        elif msg_type == "SYS_STATUS":
            print(f"🔧 Pil Seviyesi: {msg.battery_remaining}% | CPU Yükü: {msg.load/10}%")

try:
    oku_telemetri()
except KeyboardInterrupt:
    print("Program sonlandırıldı.")
