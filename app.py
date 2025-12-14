import cv2
import requests 
import threading 
import pyaudio 
import socket
import time
import numpy as np
from scipy.signal import butter, lfilter
from flask import Flask, Response, render_template, jsonify
from flask_cors import CORS
from ultralytics import YOLO

app = Flask(__name__)
CORS(app)

# ================= 1. CONFIGURATION =================
# FIREBASE URL
FIREBASE_URL = "https://rescuebot-9f045-default-rtdb.firebaseio.com/sensors.json"

# CAMERA URL (From ESP32-CAM Serial Monitor)
CAM_URL = "http://10.227.91.20:81/stream" 

# AUDIO SETTINGS
ESP_AUDIO_IP = "10.109.218.65"   # <--- Confirm this matches your ESP32 #2 IP
ESP_AUDIO_PORT = 88
SAMPLE_RATE = 16000
CHUNK_SIZE = 512    # Low latency chunk size
NOISE_THRESHOLD = 500 

# AI SETTINGS
model = YOLO("yolo11n.pt") 
TARGET_CLASSES = [0, 16] # Person, Dog

# ================= 2. AUDIO FILTERING LOGIC (Preserved) =================
def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a

def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y

# ================= 3. TWO-WAY AUDIO MANAGER =================
def audio_manager():
    """
    Handles Full Duplex Audio:
    1. Sends Laptop Mic/Stereo Mix -> Bot Speaker
    2. Receives Bot Mic -> Laptop Speaker (Filtered)
    """
    while True:
        s = None
        try:
            print(f"🔌 AUDIO: Connecting to {ESP_AUDIO_IP}:{ESP_AUDIO_PORT}...")
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5)
            s.connect((ESP_AUDIO_IP, ESP_AUDIO_PORT))
            s.settimeout(None)
            print("✅ AUDIO CONNECTED! Two-Way Link Active.")
            
            p = pyaudio.PyAudio()
            
            # Stream 1: INPUT (Laptop Mic -> Bot Speaker)
            # Uses Windows Default Device (Stereo Mix or Mic Array)
            mic_stream = p.open(format=pyaudio.paInt16, 
                                channels=1, 
                                rate=SAMPLE_RATE, 
                                input=True, 
                                frames_per_buffer=CHUNK_SIZE)
            
            # Stream 2: OUTPUT (Bot Mic -> Laptop Speakers)
            speaker_stream = p.open(format=pyaudio.paInt16, 
                                    channels=1, 
                                    rate=SAMPLE_RATE, 
                                    output=True, 
                                    frames_per_buffer=CHUNK_SIZE)
            
            # --- THREAD: RECEIVE FROM BOT (With Filter) ---
            def receive_thread():
                while True:
                    try:
                        data = s.recv(CHUNK_SIZE * 2)
                        if not data: break
                        
                        audio_data = np.frombuffer(data, dtype=np.int16)
                        
                        # Apply Bandpass Filter & Noise Gate
                        if np.max(np.abs(audio_data)) > NOISE_THRESHOLD:
                            filtered = butter_bandpass_filter(audio_data, 300, 3400, SAMPLE_RATE, order=3)
                            # Volume Boost (x2) and Clip
                            loud = np.clip(filtered * 2.0, -32767, 32767)
                            speaker_stream.write(loud.astype(np.int16).tobytes())
                        else:
                            # Silence static
                            speaker_stream.write(np.zeros_like(audio_data).tobytes())
                    except:
                        break

            # --- THREAD: SEND TO BOT (Raw Audio) ---
            def send_thread():
                while True:
                    try:
                        # Read Laptop Audio
                        data = mic_stream.read(CHUNK_SIZE, exception_on_overflow=False)
                        s.sendall(data)
                    except:
                        break

            # Start Threads
            t1 = threading.Thread(target=receive_thread); t1.daemon=True; t1.start()
            t2 = threading.Thread(target=send_thread); t2.daemon=True; t2.start()
            
            t1.join() # Wait for connection loss
            print("⚠️ Audio Disconnected. Retrying in 3s...")
            s.close()
            
        except Exception as e:
            print(f"❌ Audio Connection Failed: {e}")
            if s: s.close()
            time.sleep(3)

# Start Audio System in Background
t_audio = threading.Thread(target=audio_manager)
t_audio.daemon = True
t_audio.start()

# ================= 4. VIDEO & SENSORS =================
def generate_frames():
    cap = cv2.VideoCapture(CAM_URL)
    while True:
        success, frame = cap.read()
        if not success:
            cap.release(); time.sleep(1); cap = cv2.VideoCapture(CAM_URL); continue
        
        frame = cv2.resize(frame, (640, 480))
        results = model.predict(frame, classes=TARGET_CLASSES, conf=0.4, verbose=False)
        annotated_frame = results[0].plot()
        ret, buffer = cv2.imencode('.jpg', annotated_frame)
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def index(): return render_template('index.html')

@app.route('/video_feed')
def video_feed(): return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/sensor_data')
def sensor_data():
    try:
        r = requests.get(FIREBASE_URL, timeout=3)
        if r.status_code == 200: return jsonify(r.json())
        return jsonify({"mq2": 0})
    except: return jsonify({"mq2": 0})

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, debug=True)