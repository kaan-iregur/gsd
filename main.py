from mission import run_mission
from video_stream import start_video_stream
import threading

if __name__ == "__main__":
    # Video akışını ayrı bir thread'de başlat
    stream_thread = threading.Thread(
        target=start_video_stream,
        kwargs={
            "width": 1280,
            "height": 720,
            "framerate": 30,
            "bitrate": 4000000,
            "destination_ip": "192.168.1.100",
            "destination_port": 5000
        },
        daemon=True
    )
    #stream_thread.start()

    # Örnek waypoint listesi: [(lat, lon, alt), ...]
    waypoints = [
    (40.74465390, 29.94059530, 10),
    (40.74502170, 29.94062220, 10),
    (40.74513750, 29.94055780, 10),
    (40.74513140, 29.94031100, 10),
    (40.74509690, 29.94003470, 10),
    (40.74477580, 29.94002940, 10),
    (40.74441410, 29.94006160, 10),
    (40.74436740, 29.94034320, 10)
]
    try:
        run_mission(waypoints, takeoff_alt=10)
    except KeyboardInterrupt:
        print("Program sonlandırıldı.")
