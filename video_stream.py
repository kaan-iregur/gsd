import subprocess
import threading
import os
import signal

def start_video_stream(
    width: int = 1280,
    height: int = 720,
    framerate: int = 30,
    bitrate: int = 4000000,
    destination_ip: str = "192.168.1.100",
    destination_port: int = 5000
):
    """
    Starts video streaming from Raspberry Pi 5 to the specified destination using libcamera-vid and GStreamer.
    Press 'q' and Enter in terminal to stop the stream.
    """
    libcamera_cmd = (
        f"libcamera-vid -t 0 --width {width} --height {height} "
        f"--framerate {framerate} --bitrate {bitrate} "
        f"--codec libav --libav-format h264 --inline --camera 0 -o -"
    )
    gstreamer_cmd = (
        f"gst-launch-1.0 fdsrc ! h264parse ! rtph264pay config-interval=1 pt=96 ! "
        f"udpsink host={destination_ip} port={destination_port} sync=false async=false"
    )
    full_cmd = f"{libcamera_cmd} | {gstreamer_cmd}"

    print(f"📡 Streaming to {destination_ip}:{destination_port} at {width}x{height}@{framerate}fps...")
    process = subprocess.Popen(
        full_cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid
    )

    def wait_for_exit():
        while True:
            if input("Press 'q' + Enter to stop: ").strip().lower() == "q":
                print("🛑 Stopping stream...")
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                break

    thread = threading.Thread(target=wait_for_exit, daemon=True)
    thread.start()

    try:
        process.wait()
    except KeyboardInterrupt:
        print("⏹ KeyboardInterrupt – terminating stream")
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
