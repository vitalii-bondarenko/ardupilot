#!/usr/bin/env python3
"""
HDMI preview with OV5647 camera.

- Checks for camera presence.
- Chooses 720p or 1080p sensor mode depending on HDMI resolution.
- Optional MAVLink connection for OSD and digital stabilisation.
- Ctrl-C or SIGTERM exits cleanly.
"""

import sys
import time
import signal
import math
import threading
import argparse

import cv2
import numpy as np
from pymavlink import mavutil
from picamera2 import Picamera2, Preview, MappedArray

# --------------------------------------------------------------------------------------
# MAVLink globals
# --------------------------------------------------------------------------------------
mav_connected = False
attitude = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}  # radians
relay_state = {"relay1": "N/A", "relay2": "N/A"}

# Event to stop background threads gracefully
stop_event = threading.Event()


def detect_display_mode() -> tuple[int, int]:
    """Return (width, height) either 1280x720 or 1920x1080 based on HDMI mode."""
    candidates = [
        "/sys/class/graphics/fb0/virtual_size",
        "/sys/class/graphics/fb0/modes",
        "/sys/class/drm/card0-HDMI-A-1/modes",
        "/sys/class/drm/card1-HDMI-A-1/modes",
    ]
    for path in candidates:
        try:
            data = open(path).read().strip().split()[0]
        except Exception:
            continue
        if "x" in data:
            try:
                w, h = map(int, data.split("x")[:2])
            except Exception:
                continue
        elif "," in data:
            try:
                w, h = map(int, data.split(",")[:2])
            except Exception:
                continue
        else:
            continue
        if w >= 1920 or h >= 1080:
            return 1920, 1080
        break
    return 1280, 720


def mavlink_worker(conn_string: str) -> None:
    """Background thread: read MAVLink messages and update globals."""
    global mav_connected, attitude, relay_state
    while not stop_event.is_set():
        try:
            conn = mavutil.mavlink_connection(conn_string)
            conn.wait_heartbeat(timeout=5)
            mav_connected = True
            last_hb = time.time()
            while not stop_event.is_set():
                msg = conn.recv_match(blocking=True, timeout=0.1)
                if msg is None:
                    if time.time() - last_hb > 5:
                        mav_connected = False
                        break
                    continue
                if msg.get_type() == "HEARTBEAT":
                    last_hb = time.time()
                    mav_connected = True
                elif msg.get_type() == "ATTITUDE":
                    attitude["roll"] = msg.roll
                    attitude["pitch"] = msg.pitch
                    attitude["yaw"] = msg.yaw
                elif msg.get_type() == "NAMED_VALUE_INT":
                    name = msg.name.decode(errors="ignore").lower()
                    if name == "relay1":
                        relay_state["relay1"] = "ON" if msg.value else "OFF"
                    elif name == "relay2":
                        relay_state["relay2"] = "ON" if msg.value else "OFF"
        except Exception:
            mav_connected = False
            time.sleep(1)


def stabilize_frame_from_gyro(
    frame: np.ndarray,
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
    f: float,
) -> np.ndarray:
    """Apply 3D rotation compensation based on IMU data."""
    h, w = frame.shape[:2]

    roll = -roll_rad
    pitch = -pitch_rad
    yaw = -yaw_rad

    Rx = np.array(
        [
            [1, 0, 0],
            [0, math.cos(pitch), -math.sin(pitch)],
            [0, math.sin(pitch), math.cos(pitch)],
        ]
    )
    Ry = np.array(
        [
            [math.cos(yaw), 0, math.sin(yaw)],
            [0, 1, 0],
            [-math.sin(yaw), 0, math.cos(yaw)],
        ]
    )
    Rz = np.array(
        [
            [math.cos(roll), -math.sin(roll), 0],
            [math.sin(roll), math.cos(roll), 0],
            [0, 0, 1],
        ]
    )

    R = Rz @ Ry @ Rx

    K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]])

    H = K @ R @ np.linalg.inv(K)

    return cv2.warpPerspective(frame, H, (w, h))


def draw_overlay(request):
    """Draw connection and relay info at the top of the frame."""
    status = "MAV: OK" if mav_connected else "MAV: NO"
    relay = f"R1:{relay_state['relay1']} R2:{relay_state['relay2']}"
    text = f"{status}  {relay}"

    with MappedArray(request, "main") as m:
        arr = m.array
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.0
        thickness = 2
        y = 30
        cv2.putText(
            arr, text, (10, y), font, font_scale, (0, 255, 0), thickness, cv2.LINE_AA
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mavlink",
        default=None,
        help="MAVLink connection string, e.g. udp:127.0.0.1:14550",
    )
    parser.add_argument(
        "--stabilize",
        action="store_true",
        help="Enable digital video stabilisation using IMU",
    )
    args = parser.parse_args()

    # --- 1. check camera ------------------------------------------------------
    if not Picamera2.global_camera_info():
        sys.stderr.write(
            "❌  Камеру не знайдено — перевірте шлейф та dtoverlay=ov5647\n"
        )
        sys.exit(1)

    cam = Picamera2()
    print(cam.sensor_modes)
    # --- 2. configure according to HDMI --------------------------------------
    DISPLAY_W, DISPLAY_H = detect_display_mode()
    preview_cfg = cam.create_preview_configuration(
        main={"size": (DISPLAY_W, DISPLAY_H), "format": "RGB888"},
        buffer_count=4,
        sensor={
            "output_size": (1296, 972),
            "bit_depth": 10,
        },
    )
    preview_cfg["controls"] = {"FrameRate": 30, "AeMeteringMode": 1}

    cam.configure(preview_cfg)
    cam.pre_callback = draw_overlay

    # start MAVLink thread if requested
    if args.mavlink:
        t = threading.Thread(target=mavlink_worker, args=(args.mavlink,), daemon=True)
        t.start()

    # --- 3. shutdown handler --------------------------------------------------
    def shutdown(signum, frame):
        stop_event.set()
        cam.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # --- 4. start camera ------------------------------------------------------
    if args.stabilize:
        cam.start()
        cv2.namedWindow("preview", cv2.WINDOW_NORMAL)
        try:
            while not stop_event.is_set():
                frame = cam.capture_array()

                h, w = frame.shape[:2]
                crop_w, crop_h = w // 2, h // 2
                x0 = (w - crop_w) // 2
                y0 = (h - crop_h) // 2
                crop = frame[y0 : y0 + crop_h, x0 : x0 + crop_w]

                f_pix = (crop_w / 2) / math.tan(math.radians(160 / 2))
                stabilized = stabilize_frame_from_gyro(
                    crop,
                    attitude["roll"],
                    attitude["pitch"],
                    attitude["yaw"],
                    f_pix,
                )

                pip = cv2.resize(stabilized, (w // 3, h // 3))
                ph, pw = pip.shape[:2]
                frame[10 : 10 + ph, w - pw - 10 : w - 10] = pip

                status = "MAV: OK" if mav_connected else "MAV: NO"
                relay = f"R1:{relay_state['relay1']} R2:{relay_state['relay2']}"
                cv2.putText(
                    frame,
                    f"{status}  {relay}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

                cv2.imshow("preview", frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
        finally:
            stop_event.set()
            cam.stop()
            cv2.destroyAllWindows()
    else:
        cam.start_preview(Preview.DRM, x=0, y=0, width=DISPLAY_W, height=DISPLAY_H)
        cam.start()
        print(
            f"📷  Fullscreen {DISPLAY_W}×{DISPLAY_H} превʼю запущено.  Ctrl-C — вихід."
        )
        try:
            while True:
                time.sleep(60)
        finally:
            stop_event.set()
            cam.stop()


if __name__ == "__main__":
    main()
