from pymavlink import mavutil

# Örnek: mavlink bağlantısını oluşturduğunuzu varsayalım
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
master.wait_heartbeat()

# DO_SET_SERVO komutunu doğrudan MAVLink ile gönder
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
    0,      # confirmation
    5,      # Param1: Servo kanalı (1–16)
    2000,   # Param2: PWM süresi (µs)
    0, 0, 0, 0, 0
)
