import random
from pymavlink import mavutil

def send_random_pwm(master):
    """
    Servo kanalına rastgele PWM değeri gönderir.
    """
    pwm = random.randint(1000, 2000)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        5,      # Servo kanalı
        pwm,    # Rastgele PWM
        0, 0, 0, 0, 0
    )
    print(f"🎛️ Gönderilen PWM: {pwm} µs")

def read_telemetry(master):
    """
    Telemetri mesajlarını okuyup türlerine göre ekrana basar.
    """
    msg = master.recv_match(blocking=True)
    if not msg:
        return

    t = msg.get_type()
    if t == "BATTERY_STATUS":
        volt = msg.voltages[0] / 1000.0
        current = msg.current_battery / 100.0
        print(f"🔋 Batarya: {volt:.2f} V, {current:.2f} A")
    elif t == "ALTITUDE":
        print(f"📏 Yükseklik: {msg.altitude_relative/1000:.2f} m")
    elif t == "GPS_RAW_INT":
        print(f"📡 GPS: lat={msg.lat/1e7:.7f}, lon={msg.lon/1e7:.7f}, alt={msg.alt/1000:.2f} m")
    elif t == "SYS_STATUS":
        print(f"🔧 Pil Seviyesi: {msg.battery_remaining}% | CPU Yükü: {msg.load/10}%")
