#!/usr/bin/env python3
"""
enroll_finger.py
A small script to enroll a fingerprint into ID #1 using the same
adafruit_fingerprint library. Run this once; then your main code
can detect a 'good' fingerprint.
"""

import time
import serial
import adafruit_fingerprint

UART_DEVICE = "/dev/ttyAMA0"
UART_BAUD   = 57600

def open_sensor_port():
    ser = serial.Serial(UART_DEVICE, UART_BAUD, timeout=1)
    return adafruit_fingerprint.Adafruit_Fingerprint(ser)

def enroll_finger(finger, enroll_id=1):
    print(f"Enrolling finger at ID #{enroll_id}")
    # Step 1: get first image
    while True:
        if finger.get_image() == adafruit_fingerprint.OK:
            print("  Got first image")
            break
        time.sleep(0.3)

    if finger.image_2_tz(1) != adafruit_fingerprint.OK:
        print("  image_2_tz(1) failed")
        return False
    print("Remove finger")
    time.sleep(2)

    print("Place the same finger again...")
    while True:
        if finger.get_image() == adafruit_fingerprint.OK:
            print("  Got second image")
            break
        time.sleep(0.3)

    if finger.image_2_tz(2) != adafruit_fingerprint.OK:
        print("  image_2_tz(2) failed")
        return False

    if finger.create_model() != adafruit_fingerprint.OK:
        print("  create_model() failed")
        return False

    if finger.store_model(enroll_id) == adafruit_fingerprint.OK:
        print(f"  Enrolled successfully at ID #{enroll_id}")
        return True
    else:
        print("  store_model() failed")
        return False

def main():
    try:
        finger = open_sensor_port()
        if finger.verify_password() != adafruit_fingerprint.OK:
            print("Could not verify fingerprint sensor password.")
            return
        ok = enroll_finger(finger, 1)
        if ok:
            print("Enrollment done.")
        else:
            print("Enrollment failed.")
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
