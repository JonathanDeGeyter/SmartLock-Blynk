# 🔐 Smart Drawer Lock – Raspberry Pi 5 + ESP32 Secure System

Welcome to the **Smart Drawer Lock Project**, a multi-layered physical security system built around a **Raspberry Pi 5** and **ESP32**, combining RFID, keypad, fingerprint authentication, ultrasonic sensing, remote control, data logging, and live dashboards. This was created as both a school project and a personal challenge to explore embedded hardware, software architecture, networked systems, and visual analytics.


[![Watch the video](https://img.youtube.com/vi/7pz2giw-tRE/maxresdefault.jpg)](https://youtu.be/7pz2giw-tRE)

### [📺 Watch the full demo video here](https://youtu.be/7pz2giw-tRE)

**NOTICE: ALTHOUGH CURRENTLY THIS A WORKING PROJECT, IT'S STILL IN PROGRESS AND WILL BE UPDATED IN THE FUTURE!**


---

## 🚀 Project Goals

- ✅ Secure a drawer with **multi-factor authentication**
- ✅ Provide **visual & audio feedback** for status
- ✅ Enable **remote control via Blynk app**
- ✅ Record all events to **InfluxDB**
- ✅ Visualize live system status in **Grafana**
- ✅ Maintain private remote access using **Tailscale VPN**
- ✅ Learn integration of hardware + cloud/local software

---

## 🧠 System Overview

### 🔹 Raspberry Pi 5 – Central Logic Controller
Handles all authentication steps and logic:
- User input via **Keypad**
- Biometric match via **Fingerprint sensor**
- Card verification via **NFC** (ISO15693)
- Controls **Servo motor**, reads **Ultrasonic sensor**
- Displays messages on **LCD screen**
- Sends logs to **InfluxDB**
- Publishes data over **MQTT** to ESP32

### 🔸 ESP32 – Feedback + Remote Interface
Listens to Raspberry Pi via MQTT and Blynk:
- Activates **Red/Green LEDs**
- Plays alarm or confirmation sound via **DFPlayer Mini**
- Sends Wi-Fi RSSI & health logs
- Handles **push button** for manual reset
- Connects to **local Blynk server** running on Raspberry Pi

---

## 🔩 Hardware Components

### Core
| Component                  | Platform       | Description                             |
|---------------------------|----------------|-----------------------------------------|
| Raspberry Pi 5            | Central Hub    | Handles logic, sensors, display, etc.   |
| ESP32 MiniKit             | Microcontroller| Controls servo, DFPlayer, LEDs          |
| 3D-Printed Servo Lock     | Locking mech.  | Optional – replaced with cardboard      |

### Sensors / Interfaces
| Sensor/Module             | Function                                      |
|--------------------------|-----------------------------------------------|
| 3x4 Keypad               | Code entry (e.g. 1234)                        |
| PN5180 NFC Reader        | Card validation (ISO15693 cards)             |
| AS608 Fingerprint Reader | Biometric auth                                |
| HC-SR04 Ultrasonic       | Drawer open/close detection                   |
| LCD 16x2 I2C             | Status display                                |
| DFPlayer Mini + Speaker  | Plays alerts/alarm sounds                    |
| Push Button              | Manual reset (lock mode → idle)              |
| LEDs (Red/Green)         | Visual status feedback                       |

---

## 🌍 Remote Access with Tailscale

To make the system **globally accessible**, a **Tailscale VPN** is used. This allows devices (laptop, phone) to securely connect to the Raspberry Pi from anywhere in the world.

---

## 📱 Blynk Interface (Self-Hosted Server)

### Server Setup
- Local Blynk server running on Raspberry Pi 5 (`port 8080`)
- Admin UI available at `https://<PI_IP>:9443`
- ESP32 connects using Blynk auth token

### Widgets
| Widget Type     | Function                                   |
|----------------|--------------------------------------------|
| Terminal (V9)   | Send `reset` or `unlock` commands          |
| SuperChart (V10)| System State – `0 = idle`, `1 = locked`    |
| SuperChart (V11)| Drawer State – `0 = closed`, `1 = open`    |
| LCD (V5)        | Shows remote copy of physical LCD          |

### Example Remote Use:
- Turn off Wi-Fi on phone
- Enable mobile data (4G/5G)
- Connect to tailscale
- Open Blynk app → send `unlock`
- Drawer unlocks from anywhere in the world 🌍

---

## 📊 Grafana Dashboards (via InfluxDB)

The Raspberry Pi publishes logs (via MQTT callbacks) to **InfluxDB**. Grafana reads from it and shows everything:

| Panel                     | What It Shows                                            |
|--------------------------|-----------------------------------------------------------|
| 🟢 System State          | idle/locked/reset state                                 |
| 📦 Drawer State          | opened / closed events + alerts                         |
| 📏 Ultrasonic Distance   | Live cm readings to track drawer movement               |
| 🆔 Card Events           | Table of GOOD / BAD RFID card scans                     |
| 🧬 Fingerprint Events    | Matched / Failed results + icon/colors                  |
| 🔢 Code Attempts         | Success/failure of PIN code entries                     |
| 🔐 Security Events       | Lockouts due to failed attempts                         |
| 💤 Drawer Timeouts       | Drawer wasn't opened in time after unlock               |
| 🔁 Servo Actions         | Open / close logs from motor                             |
| 🔘 Button Events         | When ESP32 reset button is pressed                      |
| 📶 ESP32 Wi-Fi RSSI      | Wi-Fi strength graph                                     |
| 💬 ESP32 Health Logs     | ONLINE RSSI logs                                         |

### Email Alerts (SMTP)
- Drawer opened
- Drawer closed
- System locked (after 3 failed attempts)

---

## 📽️ Demo Flow (Scenario Walkthrough)

### ✅ Normal Flow
1. Enter code `1234` on keypad
2. Green LED blinks, LCD: "Good Code"
3. Scan valid NFC card
4. Green LED blinks again, LCD: "Card OK"
5. Fingerprint scan — match confirmed
6. Green LED + LCD: "Fingerprint OK"
7. Servo unlocks drawer
8. Drawer opened — ultrasonic sensor detects it
9. Drawer closed → reopens → reclosed → servo locks

### 🔐 Intruder Scenarios
- 3x wrong code → lock mode → alarm sound + red LED
- 3x invalid RFID cards → lock mode → alarm sound + red LED
- 3x failed fingerprints → lock mode → alarm sound + red LED
- Unlock but never open drawer → timeout → auto-lock
- Reset system manually (button or Blynk `reset`)

### Limitations
- Ultrasonic sensor sometimes misses drawer state if half-closed
- Speaker distortion at max volume
- No facial or BLE support yet

---

## 🔧 Installation Guide

### 1. Raspberry Pi Setup
```bash
sudo raspi-config   # Enable I2C, Serial
sudo apt update && sudo apt install python3-pip
pip install paho-mqtt influxdb adafruit-circuitpython-fingerprint gpiozero
```

### 2. InfluxDB
```bash
sudo apt install influxdb
influx
> CREATE DATABASE smart_lock
```

### 3. Grafana
- Install from https://grafana.com/docs/

### 4. Blynk Server
server-0.41.16.jar

Access Blynk admin panel:
`https://<pi-ip>:9443`

### 5. Tailscale VPN
```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
```

### 6. ESP32
- Open `smartlock_esp32.ino` in Arduino IDE
- Install `Blynk`, `PubSubClient`, `SoftwareSerial`, etc.
- Flash code

---

## 📁 File Structure
```
smart-lock-drawer/
├── raspberry_pi/
│   ├── rpi_main.py
│   ├── LCD.py
│   ├── enroll_finger.py
├── esp32/
│   └── smartlock_esp32.ino
├── blynk/
│   └── server.jar
├── docs/
│   └── images/(Grafana, Blynk and Wiring Diagram)           
└── README.md
```

---

## ✨ Future Work
- [ ] Secure code/fingerprint files (not hardcoded)
- [ ] Add BLE unlock (via smartphone)
- [ ] Add web dashboard via Flask
- [ ] Use OLED or touchscreen for better UI
- [ ] Fine-tune ultrasonic drawer sensing logic
- [ ] Fix DFPlayer distortion + loop bug
- [ ] Combine Pi + ESP32 into a compact box
- [ ] Add real-time mobile alerts via Telegram or Signal
- [ ] Using WiFi camera
- [ ] More functionalities in Blynk

---

## 📜 License
MIT License – feel free to use, modify, and share!

---

## 📬 Contact
Made with 💡 by **Jonathan** – 2025
> Part of a school project and personal challenge to combine hardware, software, networking, and visualization into a full IoT security system.

Feel free to reach out if you have any questions!

---

