import subprocess
import threading
import os
import signal

def start_video_stream(
    width=1280,
    height=720,
    framerate=30,
    bitrate=4000000,
    destination_ip="192.168.1.100",
    destination_port=5000
):
    """
    Starts video streaming from Raspberry Pi 5 to the specified destination using libcamera-vid and GStreamer.
    Press 'q' and Enter in terminal to stop the stream.
    """
    libcamera_cmd = (
        f"libcamera-vid -t 0 --width {width} --height {height} --framerate {framerate} "
        f"--bitrate {bitrate} --codec libav --libav-format h264 --inline --camera 0 -o -"
    )

    # HATALI: qfdsrc → DOĞRU: fdsrc
    gstreamer_cmd = (
        f"gst-launch-1.0 fdsrc ! h264parse ! rtph264pay config-interval=1 pt=96 ! "
        f"udpsink host={destination_ip} port={destination_port} sync=false async=false"
    )

    full_cmd = f"{libcamera_cmd} | {gstreamer_cmd}"

    print(f"Starting stream to {destination_ip}:{destination_port} at {width}x{height}@{framerate}fps...")

    process = subprocess.Popen(full_cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid)

    def wait_for_exit():
        while True:
            user_input = input("Press 'q' then Enter to stop streaming: ").strip().lower()
            if user_input == "q":
                print("Stopping stream...")
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                break

    input_thread = threading.Thread(target=wait_for_exit)
    input_thread.daemon = True
    input_thread.start()

    try:
        process.wait()
        print("Stream ended.")
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Terminating stream...")
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)

if __name__ == "__main__":
    start_video_stream()
