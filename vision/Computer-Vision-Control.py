import cv2
import numpy as np
import time
import platform
import serial

from cvtest_tools import WebStreamer, detect_cameras


def process_vision(frame):
    result_frame = frame.copy()

    height, width, _ = frame.shape
    frame_cx = width // 2
    frame_cy = height // 2

    cv2.circle(result_frame, (frame_cx, frame_cy), 6, (255, 0, 0), -1)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_pink = np.array([145, 50, 80])
    upper_pink = np.array([179, 255, 255])

    mask = cv2.inRange(hsv, lower_pink, upper_pink)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    target_info = None
    best_contour = None
    best_area = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 1000:
            continue

        if area > best_area:
            best_area = area
            best_contour = cnt

    if best_contour is not None:
        x, y, w, h = cv2.boundingRect(best_contour)

        target_cx = x + w // 2
        target_cy = y + h // 2

        error_x = target_cx - frame_cx
        error_y = target_cy - frame_cy

        target_info = {
            "cx": target_cx,
            "cy": target_cy,
            "error_x": error_x,
            "error_y": error_y,
            "area": best_area
        }

        cv2.rectangle(result_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(result_frame, (target_cx, target_cy), 6, (0, 0, 255), -1)
        cv2.line(result_frame, (frame_cx, frame_cy), (target_cx, target_cy), (0, 255, 255), 2)

        cv2.putText(result_frame, f"error_x: {error_x}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(result_frame, f"error_y: {error_y}", (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    else:
        cv2.putText(result_frame, "target not found", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    return result_frame, target_info


if __name__ == "__main__":
    cams = detect_cameras()
    if not cams:
        print("没有检测到摄像头")
        exit()

    ser = serial.Serial(
        port="/dev/serial0",
        baudrate=115200,
        timeout=0.01
    )

    streamer = WebStreamer(port=8080)

    target_idx = cams[0]["index"]
    sys_name = platform.system()

    if sys_name == "Windows":
        cap = cv2.VideoCapture(target_idx, cv2.CAP_DSHOW)
    elif sys_name == "Linux":
        cap = cv2.VideoCapture(target_idx, cv2.CAP_V4L2)
    else:
        cap = cv2.VideoCapture(target_idx)

    last_send_time = 0
    send_interval = 0.05

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("摄像头读取失败")
                break
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            result_frame, target_info = process_vision(frame)

            streamer.update_frame(0, frame)
            streamer.update_frame(1, result_frame)

            current_time = time.monotonic()

            if current_time - last_send_time >= send_interval:
                if target_info is not None:
                    error_x = target_info["error_x"]
                    error_y = -target_info["error_y"]

                    message = f"{error_x},{error_y}\n"
                    ser.write(message.encode("ascii"))

                    print("send:", message.strip())
                else:
                    ser.write(b"L\n")
                    print("send: L")

                last_send_time = current_time

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass

    finally:
        cap.release()
        streamer.stop()
        ser.close()