from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil
import time
#dronekit-sitl copter --model quad --home=40.74442430, 29.94059800, 0, 180
# 1) Başlangıç (home) noktası
home_lat, home_lon = 40.74442430, 29.94059800
target_altitude = 15  # metre

# 2) Verilen koordinatlar
waypoints = [
    (40.74465390, 29.94059530),
    (40.74502170, 29.94062220),
    (40.74513750, 29.94055780),
    (40.74513140, 29.94031100),
    (40.74509690, 29.94003470),
    (40.74477580, 29.94002940),
    (40.74441410, 29.94006160),
    (40.74436740, 29.94034320)
]

def upload_mission(vehicle):
    cmds = vehicle.commands
    cmds.clear()  # Önceki misyonu temizle :contentReference[oaicite:11]{index=11}

    # Takeoff komutu :contentReference[oaicite:12]{index=12}
    cmds.add(Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0,
        home_lat, home_lon, target_altitude
    ))

    # Waypoint’leri ekle :contentReference[oaicite:13]{index=13}
    for lat, lon in waypoints:
        cmds.add(Command(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0, 0, 0, 0, 0,
            lat, lon, target_altitude
        ))
        # cmds.add(Command(0, 0, 0,
        #     mavutil.mavlink.MAV_FRAME_MISSION,
        #     mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        #     0, 0,
        #     9, 1500, 0, 0, 0, 0, 0))
    # Land komutu :contentReference[oaicite:14]{index=14}
    cmds.add(Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0,
        home_lat, home_lon, 0
    ))

    cmds.upload()  # Görevi yükle :contentReference[oaicite:15]{index=15}
    print("Mission uploaded.")

def arm_and_start_mission(connection_str):
    # 3) Uçağa bağlan
    vehicle = connect(connection_str, wait_ready=True)
    #vehicle = connect('com12', wait_ready=True, baud=9600)  # :contentReference[oaicite:16]{index=16}
    vehicle.groundspeed = 5
    upload_mission(vehicle)

    # 4) Arming ve AUTO moda geçiş
    while not vehicle.is_armable:
        time.sleep(1)
    input('Görev başlasın mı?')
    vehicle.mode = VehicleMode("AUTO")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
    print("Armed and in AUTO mode.")
    
    # 5) (Gerekirse) MAV_CMD_MISSION_START ile başlat
    vehicle.send_mavlink(
        vehicle.message_factory.command_long_encode(
            vehicle._master.target_system,
            vehicle._master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0, 0, 0, 0, 0, 0, 0, 0
        )
    )
    print("Mission start command sent.")

    # 6) Görev ilerleyişini izle :contentReference[oaicite:17]{index=17}
    total_cmds = vehicle.commands.count
    while True:
        next_wp = vehicle.commands.next
        print(f"Next WP: {next_wp}/{total_cmds} Spd:-{vehicle.groundspeed} ")
        # İniş tamamlandı mı?
        if next_wp == total_cmds and vehicle.location.global_relative_frame.alt < 0.1:
            print("Landed at home.")
            break
        time.sleep(2)

    vehicle.close()
    print("Disconnected.")

if __name__ == "__main__":
    # SITL: 'udp:127.0.0.1:14550' veya Mission Planner TCP: 'tcp:127.0.0.1:5760'
    arm_and_start_mission('tcp:127.0.0.1:5763')