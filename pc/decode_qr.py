#!/usr/bin/env python3
"""
decode_qr.py — PC-side QR decoder for the Sorting Robot (PC-decode variant).

Reads the ESP32-CAM MJPEG stream, decodes the QR with OpenCV/pyzbar (far more
robust than on-board quirc), and forwards "QR:<code>" to the brain over USB
serial. brainPC.ino routes any line starting with "QR:" into its sort logic;
anything else you type is forwarded as a normal brain command (HA, Q1, B..., P).

Install:
    pip install opencv-python pyzbar pyserial
    # pyzbar needs the zbar lib too:  Linux: sudo apt install libzbar0

Run:
    python decode_qr.py --url http://<cam-ip>/stream --port /dev/ttyUSB0
    (Windows: --port COM5).  Add --show to see the video window.
"""

import argparse
import sys
import time
import threading

import cv2
import serial

try:
    from pyzbar.pyzbar import decode as zbar_decode
    HAVE_ZBAR = True
except Exception:
    HAVE_ZBAR = False

_detector = cv2.QRCodeDetector()


def decode_qr(frame):
    """Return the decoded QR string, or None. Tries pyzbar, then OpenCV."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if HAVE_ZBAR:
        for obj in zbar_decode(gray):
            if obj.type == "QRCODE":
                try:
                    return obj.data.decode("utf-8").strip()
                except Exception:
                    pass
    # OpenCV fallback. detectAndDecode can throw cv2.error on degenerate frames
    # (contourArea==0) instead of returning empty — swallow that.
    try:
        data, _pts, _ = _detector.detectAndDecode(gray)
        if data:
            return data.strip()
    except cv2.error:
        pass
    return None


def serial_reader(ser):
    """Print whatever the brain sends back (so you see its logs)."""
    while True:
        try:
            line = ser.readline().decode("utf-8", "replace").rstrip()
            if line:
                print("[brain]", line)
        except Exception:
            break


def stdin_forwarder(ser):
    """Forward what you type to the brain as a command line."""
    for line in sys.stdin:
        ser.write((line.strip() + "\n").encode())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", required=True, help="MJPEG URL, e.g. http://192.168.1.50/stream")
    ap.add_argument("--port", required=True, help="brain serial port, e.g. /dev/ttyUSB0 or COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--cooldown", type=float, default=1.5,
                    help="min seconds before resending the SAME code")
    ap.add_argument("--show", action="store_true", help="show the video window")
    args = ap.parse_args()

    print("pyzbar:", "yes" if HAVE_ZBAR else "no (OpenCV fallback)")
    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    time.sleep(2.0)  # let the brain finish resetting after the port opens
    threading.Thread(target=serial_reader, args=(ser,), daemon=True).start()
    threading.Thread(target=stdin_forwarder, args=(ser,), daemon=True).start()

    cap = cv2.VideoCapture(args.url)
    if not cap.isOpened():
        print("ERROR: cannot open stream:", args.url)
        return

    last_code, last_sent, n_ok = None, 0.0, 0
    while True:
        ok, frame = cap.read()
        if not ok:
            print("stream read failed — reconnecting...")
            cap.release()
            time.sleep(1.0)
            cap = cv2.VideoCapture(args.url)
            continue

        code = decode_qr(frame)
        if code:
            now = time.time()
            if code != last_code or (now - last_sent) > args.cooldown:
                ser.write(("QR:%s\n" % code).encode())
                n_ok += 1
                print("[SENT -> brain] QR:%s   (#%d)" % (code, n_ok))
                last_code, last_sent = code, now

        if args.show:
            if code:
                cv2.putText(frame, code, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("cam (q to quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
