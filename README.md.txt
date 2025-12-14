# 🚨 AI-Powered IoT Search & Rescue Robot

A semi-autonomous **Search & Rescue (SAR)** robot built using a dual-ESP32 distributed architecture for real-time sensing, wireless audio transmission, and remote control. Designed for deployment in disaster zones where human access is risky, this robot provides two-way audio, environmental sensing, camera support, and obstacle navigation.

---

## 🔥 Key Features
- Dual **ESP32** microcontrollers (control + communication)
- **Full-duplex audio** using I2S + raw TCP sockets
- Python-based **DSP pipeline** with band-pass filtering
- Onboard motors for navigation
- Wireless remote control over WiFi
- Modular structure for improvement and upgrades

---

## 🛠️ Hardware Used
- ESP32 Dev Module × 2  
- MAX9814 / INMP441 microphone  
- LM386 + 8Ω speaker  
- L298N / BTS7960 motor driver  
- Li-ion battery pack  
- Optional: Camera module (ESP32-CAM)

---

## 📂 Project Structure
/rescuebot
|- control_unit/
|- audio_unit/
|- python_dsp/
|- docs/
|- README.md
|- LICENSE
|- .gitignore


▶️ How to Run

Flash control ESP32 with control_unit firmware

Flash audio ESP32 with audio_unit firmware

Start Python DSP script on laptop:

python dsp_server.py

Robot will auto‑connect to WiFi and server for audio streaming

📡 Communication Flow

ESP32 (mic) → TCP → Laptop DSP → TCP → ESP32 (speaker)

🤝 Contributions

Pull requests are welcome! Please follow proper commit messages.