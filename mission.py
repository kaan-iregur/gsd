import math
from pymavlink import mavutil
from connection import create_connection
from telemetry import send_random_pwm, read_telemetry
from handlers import handle_waypoint


def upload_mission(master, waypoints):
    """
    Yalnızca waypoint ve iniş komutlarını içeren mission dosyasını ArduCopter'e yükler.
    0..n-1: Waypoints, n: Land
    """
    mission_items = []
    seq = 0
    # 1..n) Waypoints
    for lat, lon, alt in waypoints:
        mission_items.append({
            'seq': seq,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            'current': 0,
            'autocontinue': 1,
            'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0,
            'x': int(lat * 1e7),
            'y': int(lon * 1e7),
            'z': int(alt)
        })
        seq += 1
    # n+1) Land
    mission_items.append({
        'seq': seq,
        'command': mavutil.mavlink.MAV_CMD_NAV_LAND,
        'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        'current': 0,
        'autocontinue': 1,
        'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0,
        'x': 0, 'y': 0, 'z': 0
    })

    # Görev sayısını gönder
    count = len(mission_items)
    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        count
    )
    print(f"📥 {count} mission_item gönderiliyor...")

    # Öğeleri yükle
    while True:
        msg = master.recv_match(type=['MISSION_REQUEST', 'MISSION_ACK'], blocking=True)
        if not msg:
            continue
        if msg.get_type() == 'MISSION_REQUEST':
            idx = msg.seq
            item = mission_items[idx]
            master.mav.mission_item_int_send(
                master.target_system,
                master.target_component,
                item['seq'], item['frame'], item['command'],
                item['current'], item['autocontinue'],
                item['param1'], item['param2'], item['param3'], item['param4'],
                item['x'], item['y'], item['z']
            )
            print(f"📤 Gönderilen mission_item seq={idx}")
        elif msg.get_type() == 'MISSION_ACK':
            print("✅ Mission yükleme tamamlandı.")
            break


def failsafe_rtl(master):
    """
    Failsafe durumunda aracın modunu RTL'e (Return To Launch) alır.
    """
    print("⚠️ Failsafe tetiklendi — RTL moduna alınıyor...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        6, 0, 0, 0, 0, 0
    )


def wait_for_position(master, min_fix_type=3):
    """
    GPS 3D fix alıncaya kadar bekler.
    """
    print("🔍 GPS fix bekleniyor...")
    while True:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
        if not msg:
            continue
        fix = msg.fix_type
        print(f"   GPS fix_type: {fix}")
        if fix >= min_fix_type:
            print("✅ GPS hazır.")
            break


def run_mission(waypoints, takeoff_alt=10):
    """
    Manual GUIDED kalkış ve ardından AUTO moda geçişle görev ilerleyişini yönetir.
    """
    master = create_connection()
    try:
        # 1) Arm ve GUIDED mod
        master.arducopter_arm()
        master.motors_armed_wait()
        print("🚀 Motor arming tamamlandı.")
        master.set_mode_apm('GUIDED')

        # 2) Manual TAKEOFF
        print(f"⬆️ GUIDED modda kalkış: {takeoff_alt} m")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, takeoff_alt
        )
        # Yüksekliğe ulaşmayı bekle
        while True:
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if not msg:
                continue
            alt = msg.relative_alt / 1000.0
            print(f"   Güncel alt: {alt:.2f} m")
            if alt >= takeoff_alt * 0.9:
                print("✅ Kalkış yüksekliği sağlandı.")
                break

        # 3) GPS fix
        wait_for_position(master)

        # 4) Mission upload (waypoints + land)
        upload_mission(master, waypoints)

        # 5) AUTO moda geç ve görevi yürüt
        print("🚀 AUTO mod — görev başlatılıyor...")
        master.set_mode_apm('AUTO')

        # Görev ilerleyişini takip et
        total = len(waypoints) + 1  # waypoint sayısı + ensi
        while True:
            msg = master.recv_match(type='MISSION_CURRENT', blocking=True)
            if not msg:
                continue
            seq = msg.seq
            print(f"🔄 Görev aşaması: seq={seq}")
            if 0 <= seq < len(waypoints):
                handle_waypoint(waypoints[seq])
            if seq == total:
                print("🛬 İniş tamamlandı.")
                break

    except Exception as e:
        print(f"❌ Hata: {e}")
        failsafe_rtl(master)
        raise
    except KeyboardInterrupt:
        print("❌ Kullanıcı kesintisi — RTL moduna alınıyor...")
        failsafe_rtl(master)
    finally:
        print("🏁 Görev akışı sonlandı.")

