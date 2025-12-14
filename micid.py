import pyaudio

p = pyaudio.PyAudio()

print("----------------------")
print(" LIST OF AUDIO DEVICES")
print("----------------------")

# List all devices
for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    # We only care about microphones (input devices)
    if info['maxInputChannels'] > 0:
        print(f"ID {i}: {info['name']}")

p.terminate()