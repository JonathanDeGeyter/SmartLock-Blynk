#!/usr/bin/env python3
"""
smart_lock_rpi.py

This script runs on the Raspberry Pi (RPi 5) and manages the smart lock’s logic:
  1. The LCD prompts the user to enter a 4-digit passcode via a 3x4 keypad.
  2. If the passcode is correct, the system asks for an RFID card scan.
  3. If the RFID card is valid, it asks for a fingerprint scan.
  4. On successful fingerprint match, the lock opens (servo moves) and the system
     monitors the drawer using an ultrasonic sensor.
  5. If the drawer remains closed for 5 seconds, the system re-locks.
  6. All events and sensor data are logged to InfluxDB.
  7. Every 5 seconds, the Pi publishes the current system state and drawer state
     via MQTT so that the ESP32 can update Blynk SuperCharts (V10 and V11).
  8. The Pi also processes remote commands ("UNLOCK" and "reset_pressed")
     sent from the Blynk Terminal on the ESP32.
"""

import time
import sys
import re
import lgpio              # For keypad scanning
from LCD import LCD       # Custom I2C LCD library
import serial            # For fingerprint sensor communication
import adafruit_fingerprint
from gpiozero import Servo, DistanceSensor
import paho.mqtt.client as mqtt
from influxdb import InfluxDBClient

# ---- MQTT / InfluxDB Configuration ----
MQTT_ADDRESS     = '192.168.1.59'
MQTT_USER        = 'jona'
MQTT_PASSWORD    = 'jona'
MQTT_CLIENT_ID   = 'smart_lock_rpi'

INFLUXDB_ADDRESS  = '192.168.1.59'
INFLUXDB_USER     = 'jona'
INFLUXDB_PASSWORD = 'jona'
INFLUXDB_DATABASE = 'smart_lock'

# ---- MQTT Topics ----
TOPIC_CMD         = 'lock/esp32/cmd'
TOPIC_CARD        = 'lock/esp32/card_event'
TOPIC_BUTTON      = 'lock/esp32/button_event'
TOPIC_HEALTH      = 'lock/esp32/health'
TOPIC_LCD         = 'lock/esp32/lcd'
TOPIC_SYSTEMSTATE = 'lock/esp32/system_state'  # "1" if locked, "0" if not
TOPIC_DRAWERSTATE = 'lock/esp32/drawer_state'  # "1" if opened, "0" if closed

# ---- Keypad Configuration ----
KEYS = [
    ['1','2','3'],
    ['4','5','6'],
    ['7','8','9'],
    ['*','0','#']
]
ROW_PINS = [5, 6, 13, 19]
COL_PINS = [26, 16, 20]

# ---- Fingerprint Sensor Configuration ----
UART_DEVICE = "/dev/ttyAMA0"
UART_BAUD   = 57600

# ---- Servo Configuration ----
SERVO_GPIO = 18
servo = None
SERVO_INITIAL = -0.5
SERVO_DELTA   = 0.95

# ---- Ultrasonic Sensor Configuration ----
ultra_sensor = DistanceSensor(echo=24, trigger=23)

# ---- LCD Setup ----
lcd = LCD(2, 0x27, True)
lcd_text = ""  # Global variable to mirror LCD message
last_lcd_pub = 0  # Timestamp of the last LCD publish

# ---- State Machine Constants ----
STATE_IDLE         = 0
STATE_ENTER_CODE   = 1
STATE_CHECK_CARD   = 2
STATE_CHECK_FINGER = 3
STATE_UNLOCKED     = 4
STATE_WAIT_DRAWER  = 5
STATE_LOCKED       = 6

current_state = STATE_IDLE

bad_attempts_code    = 0
bad_attempts_card    = 0
bad_attempts_finger  = 0
max_attempts         = 3

drawer_open_deadline = None
drawer_open_countdown = 20

client = None             # MQTT client instance
influxdb_client = None    # InfluxDB client instance
finger = None             # Fingerprint sensor object
gpio_handle = None        # Keypad handle

def log_influx(measurement, value, extra_tags=None):
    if extra_tags is None:
        extra_tags = {}
    data_point = {
        "measurement": measurement,
        "tags": {"location": "lock_system"},
        "fields": {}
    }
    try:
        data_point["fields"]["value"] = float(value)
    except:
        data_point["fields"]["string_value"] = str(value)
    data_point["tags"].update(extra_tags)
    try:
        influxdb_client.write_points([data_point])
    except Exception as e:
        print("InfluxDB write error:", e)

def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT with rc=", rc)
    client.subscribe(TOPIC_CARD)
    client.subscribe(TOPIC_BUTTON)
    client.subscribe(TOPIC_HEALTH)
    client.subscribe(TOPIC_CMD)

def on_message(client, userdata, msg):
    global current_state, bad_attempts_card
    topic = msg.topic
    payload = msg.payload.decode('utf-8').strip()
    print(f"MQTT on {topic}: {payload}")
    
    if topic == TOPIC_CARD:
        log_influx("card_event", payload)
        handle_card_result(payload == "GOOD")
    elif topic == TOPIC_BUTTON:
        log_influx("button_event", payload)
        if current_state == STATE_LOCKED and payload == "reset_pressed":
            resetSystem()
    elif topic == TOPIC_HEALTH:
        log_influx("esp32_health", payload)
        m = re.search(r'RSSI:(-?\d+)', payload)
        if m:
            log_influx("esp32_rssi", int(m.group(1)))
    elif topic == TOPIC_CMD:
        # Handle remote commands from Blynk Terminal forwarded by ESP32
        if payload.upper() == "UNLOCK":
            force_unlock()
        elif payload.lower() == "reset_pressed":
            resetSystem()

def publishMQTT(cmd):
    if client:
        client.publish(TOPIC_CMD, cmd)

# NEW: publish system state ("1" if locked, "0" if unlocked)
def publishSystemState(state):
    if client:
        client.publish(TOPIC_SYSTEMSTATE, state)

# NEW: publish drawer state ("1" if opened, "0" if closed)
def publishDrawerState(state):
    if client:
        client.publish(TOPIC_DRAWERSTATE, state)

# NEW: Publish current LCD text
def publishLCDText():
    global last_lcd_pub
    now = time.time()
    # Publish LCD text only if at least 2 seconds have passed
    if now - last_lcd_pub >= 2:
        if client:
            client.publish(TOPIC_LCD, lcd_text)
        last_lcd_pub = now

def lockSystem(method=None):
    global current_state
    print("SYSTEM LOCKED!")
    current_state = STATE_LOCKED
    lcd.clear()
    lcd.message("SYSTEM LOCKED", 1)
    global lcd_text
    lcd_text = "SYSTEM LOCKED"
    publishMQTT("LOCK")
    publishMQTT("RED_ON")
    publishMQTT("PLAY_CONTINUOUS")
    log_influx("system_state", "1")  # "1" => locked
    if method:
        log_influx("security_event", "lockout", {"method": method})
    else:
        log_influx("security_event", "lockout")
    publishSystemState("1")

def unlockSystem():
    global current_state
    current_state = STATE_UNLOCKED
    lcd.clear()
    lcd.message("Lock opened", 1)
    lcd_text = "Lock opened"
    # We do not publish "UNLOCK" here to avoid repeated auto-unlock.
    publishMQTT("RED_OFF")
    publishMQTT("STOP_PLAY")
    log_influx("system_state", "0")  # "0" => unlocked
    publishSystemState("0")

def resetSystem():
    global current_state, bad_attempts_code, bad_attempts_card, bad_attempts_finger
    bad_attempts_code = 0
    bad_attempts_card = 0
    bad_attempts_finger = 0
    current_state = STATE_ENTER_CODE
    lcd.clear()
    lcd.message("CODE:", 1)
    global lcd_text
    lcd_text = "CODE:"
    # Do not publish "UNLOCK" here so as not to force hardware unlock.
    publishMQTT("RED_OFF")
    publishMQTT("STOP_PLAY")
    log_influx("system_state", "0")
    publishSystemState("0")

def force_unlock():
    """
    Force unlocks the system when a remote "UNLOCK" command is received.
    Simulates a successful authentication so that the servo opens.
    """
    global current_state
    print("Force unlocking system via remote command.")
    lcd.clear()
    lcd.message("Force Unlock", 1)
    global lcd_text
    lcd_text = "Force Unlock"
    publishMQTT("GREEN_ON")
    time.sleep(2)
    publishMQTT("GREEN_OFF")
    log_influx("fingerprint_event", "MATCHED (force)")
    unlockSystem()
    log_influx("servo_action", "opened")
    servo.value = SERVO_INITIAL + SERVO_DELTA
    time.sleep(1)
    servo.value = None
    current_state = STATE_WAIT_DRAWER
    global drawer_open_deadline
    drawer_open_deadline = time.time() + drawer_open_countdown

def handle_card_result(is_good):
    global current_state, bad_attempts_card
    if current_state == STATE_CHECK_CARD:
        if is_good:
            lcd.clear()
            lcd.message("Good keycard", 1)
            global lcd_text
            lcd_text = "Good keycard"
            publishMQTT("GREEN_ON")
            time.sleep(2)
            publishMQTT("GREEN_OFF")
            bad_attempts_card = 0
            current_state = STATE_CHECK_FINGER
        else:
            bad_attempts_card += 1
            lcd.clear()
            lcd.message("Bad keycard", 1)
            lcd.message("Bad keycard", 1)
            lcd_text = "Bad keycard"
            publishMQTT("RED_ON")
            time.sleep(2)
            publishMQTT("RED_OFF")
            if bad_attempts_card >= max_attempts:
                lockSystem(method="card")
            else:
                lcd.clear()
                lcd.message("Scan card again", 1)
                lcd_text = "Scan card again"
                publishMQTT("READ_CARD")

def check_fingerprint():
    print("Place your finger on sensor...")
    start_time = time.time()
    while True:
        if finger.get_image() == adafruit_fingerprint.OK:
            break
        if time.time() - start_time > 8:
            return False
        time.sleep(0.2)
    if finger.image_2_tz(1) != adafruit_fingerprint.OK:
        return False
    if finger.finger_fast_search() == adafruit_fingerprint.OK:
        print(f"Fingerprint matched ID={finger.finger_id}")
        return True
    else:
        print("No fingerprint match.")
        return False

def scan_keypad(h):
    for col_idx, col_pin in enumerate(COL_PINS):
        for i, cpin in enumerate(COL_PINS):
            val = 0 if i == col_idx else 1
            lgpio.gpio_write(h, cpin, val)
        time.sleep(0.0005)
        for row_idx, row_pin in enumerate(ROW_PINS):
            val = lgpio.gpio_read(h, row_pin)
            if val == 0:
                key = KEYS[row_idx][col_idx]
                while lgpio.gpio_read(h, row_pin) == 0:
                    time.sleep(0.01)
                for cpin in COL_PINS:
                    lgpio.gpio_write(h, cpin, 1)
                return key
    for cpin in COL_PINS:
        lgpio.gpio_write(h, cpin, 1)
    return None

def publish_states_periodically():
    """
    Publishes system state ("1" if locked, "0" if not) and
    drawer state ("1" if opened, "0" if closed) every 5 seconds.
    """
    if current_state == STATE_LOCKED:
        publishSystemState("1")
    else:
        publishSystemState("0")

    if current_state == STATE_WAIT_DRAWER:
        if ultra_sensor.distance * 100 > 4.0:
            publishDrawerState("1")
        else:
            publishDrawerState("0")
    else:
        publishDrawerState("0")
 
    publishLCDText()

def main_loop():
    global current_state, bad_attempts_code, bad_attempts_finger, drawer_open_deadline
    last_pub_time = time.time()
    while True:
        dist_cm = ultra_sensor.distance * 100
        log_influx("ultrasonic_distance", dist_cm)
        time.sleep(0.05)

        # Publish system and drawer state every 5 seconds and LCD?
        if time.time() - last_pub_time > 5:
            publish_states_periodically()
            last_pub_time = time.time()

        if current_state == STATE_IDLE:
            resetSystem()

        elif current_state == STATE_ENTER_CODE:
            key = scan_keypad(gpio_handle)
            if key is not None:
                static_code = "1234"  # Correct passcode
                entered_code = ""
                lcd.clear()
                lcd.message("Enter 4 digits:", 1)
                global lcd_text
                lcd_text = "Enter 4 digits:"
                while len(entered_code) < 4:
                    k = None
                    while k is None:
                        k = scan_keypad(gpio_handle)
                        time.sleep(0.05)
                    entered_code += k
                    lcd.clear()
                    lcd.message("Code: " + entered_code, 1)
                    lcd_text = "Code: " + entered_code
                print("Entered code:", entered_code)
                if entered_code == static_code:
                    lcd.clear()
                    lcd.message("Good Code", 1)
                    lcd_text = "Good Code"
                    publishMQTT("GREEN_ON")
                    time.sleep(2)
                    publishMQTT("GREEN_OFF")
                    bad_attempts_code = 0
                    log_influx("code_entry", "CORRECT")
                    current_state = STATE_CHECK_CARD
                    lcd.clear()
                    lcd.message("Scan your card", 1)
                    lcd_text = "Scan your card"
                    publishMQTT("READ_CARD")
                else:
                    bad_attempts_code += 1
                    lcd.clear()
                    lcd.message("Bad code", 1)
                    lcd_text = "Bad code"
                    publishMQTT("RED_ON")
                    time.sleep(2)
                    publishMQTT("RED_OFF")
                    log_influx("code_entry", "WRONG")
                    if bad_attempts_code >= max_attempts:
                        lockSystem(method="code")
                    else:
                        lcd.clear()
                        lcd.message("Try again", 1)
                        lcd_text = "Try again"

        elif current_state == STATE_CHECK_CARD:
            # Wait for RFID card result from ESP32 (handled in on_message)
            pass

        elif current_state == STATE_CHECK_FINGER:
            lcd.clear()
            lcd.message("Place Finger", 1)
            lcd_text = "Place Finger"
            time.sleep(1)
            if check_fingerprint():
                lcd.clear()
                lcd.message("Good fingerprint", 1)
                lcd_text = "Good fingerprint"
                publishMQTT("GREEN_ON")
                time.sleep(2)
                publishMQTT("GREEN_OFF")
                bad_attempts_finger = 0
                log_influx("fingerprint_event", "MATCHED")
                unlockSystem()
                log_influx("servo_action", "opened")
                servo.value = SERVO_INITIAL + SERVO_DELTA
                time.sleep(1)
                servo.value = None
                current_state = STATE_WAIT_DRAWER
                drawer_open_deadline = time.time() + drawer_open_countdown
            else:
                bad_attempts_finger += 1
                lcd.clear()
                lcd.message("Bad finger", 1)
                lcd_text = "Bad finger"
                publishMQTT("RED_ON")
                time.sleep(2)
                publishMQTT("RED_OFF")
                log_influx("fingerprint_event", "FAILED")
                if bad_attempts_finger >= max_attempts:
                    lockSystem(method="fingerprint")
                else:
                    lcd.clear()
                    lcd.message("Try again", 1)
                    lcd_text = "Try again"

        elif current_state == STATE_UNLOCKED:
            pass

        elif current_state == STATE_WAIT_DRAWER:
            dist_cm = ultra_sensor.distance * 100
            if dist_cm > 4.0:
                lcd.clear()
                lcd.message("Drawer opened", 1)
                lcd_text = "Drawer opened"
                log_influx("drawer_state", "opened")
                publishDrawerState("1")
                while True:
                    publishDrawerState("1")
                    dist2 = ultra_sensor.distance * 100
                    time.sleep(0.2)
                    if dist2 <= 4.0:
                        log_influx("drawer_state", "closed")
                        publishDrawerState("0")
                        lcd.clear()
                        lcd.message("Drawer closed", 1)
                        lcd_text = "Drawer closed"
                        time_start = time.time()
                        reOpened = False
                        while (time.time() - time_start) < 5.0:
                            dist3 = ultra_sensor.distance * 100
                            time.sleep(0.2)
                            if dist3 > 4.0:
                                reOpened = True
                                break
                        if reOpened:
                            lcd.clear()
                            lcd.message("Drawer reopened", 1)
                            lcd_text = "Drawer reopened"
                        else:
                            time.sleep(1)
                            servo.value = SERVO_INITIAL
                            time.sleep(1)
                            servo.value = None
                            log_influx("servo_action", "locked")
                            lcd.clear()
                            lcd.message("Lock closed", 1)
                            lcd_text = "Lock closed"
                            current_state = STATE_IDLE
                            break
                    if current_state != STATE_WAIT_DRAWER:
                        break
            else:
                if time.time() > drawer_open_deadline:
                    lcd.clear()
                    lcd.message("Time out. Locking", 1)
                    lcd_text = "Time out. Locking"
                    log_influx("drawer_timeout", "user_inactive")
                    time.sleep(1)
                    servo.value = SERVO_INITIAL
                    time.sleep(1)
                    servo.value = None
                    log_influx("servo_action", "locked")
                    lcd.clear()
                    lcd.message("Lock closed", 1)
                    lcd_text = "Lock closed"
                    current_state = STATE_IDLE
                else:
                    remain = int(drawer_open_deadline - time.time())
                    lcd.clear()
                    lcd.message(f"Open drawer: {remain}", 1)
                    lcd_text = f"Open drawer: {remain}"
                    time.sleep(1)

        elif current_state == STATE_LOCKED:
            pass

def main():
    global client, influxdb_client, finger, gpio_handle, servo

    # ---- InfluxDB Setup ----
    influxdb_client = InfluxDBClient(INFLUXDB_ADDRESS, 8086, INFLUXDB_USER, INFLUXDB_PASSWORD, None)
    dblist = influxdb_client.get_list_database()
    if not any(db['name'] == INFLUXDB_DATABASE for db in dblist):
        influxdb_client.create_database(INFLUXDB_DATABASE)
    influxdb_client.switch_database(INFLUXDB_DATABASE)

    # ---- MQTT Setup ----
    client_mqtt = mqtt.Client(client_id=MQTT_CLIENT_ID)  # No forced clean session
    client_mqtt.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    client_mqtt.on_connect = on_connect
    client_mqtt.on_message = on_message
    client_mqtt.connect(MQTT_ADDRESS, 1883, 60)
    client_mqtt.loop_start()
    global client
    client = client_mqtt

    # ---- Initialize Fingerprint Sensor ----
    try:
        s = serial.Serial(UART_DEVICE, UART_BAUD, timeout=1)
        f = adafruit_fingerprint.Adafruit_Fingerprint(s)
        if f.verify_password() == adafruit_fingerprint.OK:
            print("Fingerprint sensor verified.")
        else:
            print("Fingerprint sensor password verification failed.")
        global finger
        finger = f
    except Exception as e:
        print("Fingerprint sensor init error:", e)
        finger = None

    # ---- Initialize Keypad ----
    try:
        h = lgpio.gpiochip_open(0)
        for pin in ROW_PINS:
            lgpio.gpio_claim_input(h, pin)
        for pin in COL_PINS:
            lgpio.gpio_claim_output(h, pin, 1)
        global gpio_handle
        gpio_handle = h
    except Exception as e:
        print("Keypad init error:", e)
        sys.exit(1)

    # ---- Initialize Servo ----
    try:
        global servo
        servo = Servo(SERVO_GPIO, min_pulse_width=0.0005, max_pulse_width=0.0025)
        servo.value = SERVO_INITIAL
        time.sleep(1)
        servo.value = None
    except Exception as e:
        print("Servo init error:", e)
        sys.exit(1)

    # ---- Display Startup Message on LCD ----
    lcd.clear()
    lcd.message("System starting", 1)
    global lcd_text
    lcd_text = "System starting"
    time.sleep(2)

    try:
        main_loop()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        if gpio_handle:
            lgpio.gpiochip_close(gpio_handle)
        client_mqtt.loop_stop()
        client_mqtt.disconnect()
        lcd.clear()

if __name__ == "__main__":
    main()
