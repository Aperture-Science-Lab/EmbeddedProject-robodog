import os
import sys
import time
import json
import requests
import numpy as np
import sounddevice as sd
import queue
import wave
import threading
from faster_whisper import WhisperModel
import winsound  # Specific to Windows for simple beeps

# Configuration
SERVER_URL = "http://localhost:8000/api/voice-command"
MODEL_SIZE = "tiny.en" 
DEVICE = "cpu" # Fallback to CPU due to missing CuDNN DLLs on host
COMPUTE_TYPE = "int8"

# Audio Settings
SAMPLE_RATE = 16000
BLOCK_SIZE = 4000  # Process in chunks
THRESHOLD = 0.5   # Default high threshold, will be overwritten by calibration
SILENCE_DURATION = 0.8  # Reduced silence wait
MAX_RECORD_SECONDS = 3.0 # Strict limit for short commands

# Commands Mapping
# Dictionary mapping keywords to Command IDs
COMMAND_MAP = {
    # Add common hallucinations that might mean "Walk"
    "walk": "WALK", "work": "WALK", "woke": "WALK",
    "move forward": "WALK",
    "forward": "WALK",
    "go": "WALK",
    "stop": "STOP",
    "halt": "STOP",
    "back": "BACK",
    "backward": "BACK",
    "retreat": "BACK",
    "left": "LEFT",
    "turn left": "LEFT",
    "right": "RIGHT",
    "turn right": "RIGHT",
    "neutral": "NEUTRAL",
    "stand": "NEUTRAL",
    "sit": "NEUTRAL", # Assuming sit is neutral/stand for now
    "save": "SAVE",
    "calibrate": "CALIBRATE" # Contextual, might need more args
}

# Global State
audio_queue = queue.Queue()
is_recording = False
last_speech_time = 0
buffer = []

def calibrate_noise():
    """Measure ambient noise to set threshold"""
    print("\n[CALIBRATION] Measuring background noise... (Please be silent)")
    duration = 1.5 # seconds
    recordings = []
    
    def cal_callback(indata, frames, time, status):
        recordings.append(np.linalg.norm(indata) * 10)
        
    try:
        with sd.InputStream(samplerate=SAMPLE_RATE, blocksize=BLOCK_SIZE, callback=cal_callback):
            sd.sleep(int(duration * 1000))
            
        avg_noise = np.mean(recordings)
        max_noise = np.max(recordings)
        
        # Set threshold significantly above noise floor
        new_threshold = max_noise * 2.5 # Increased from 1.5 to avoid fan noise
        new_threshold = max(new_threshold, 0.4) # Higher safety floor
        
        print(f"[CALIBRATION] Noise Lvl: {avg_noise:.2f} | Max: {max_noise:.2f}")
        print(f"[CALIBRATION] Set THRESHOLD to: {new_threshold:.2f}")
        return new_threshold
    except Exception as e:
        print(f"[ERROR] Calibration failed: {e}")
        return 0.3

def play_beep(freq=1000, duration=200):
    """Play a system beep"""
    try:
        winsound.Beep(freq, duration)
    except:
        pass

def play_success():
    """Play a success sound"""
    play_beep(1500, 150)
    play_beep(2000, 150)

def speak(text):
    """Speak text using Piper TTS (or fallback)"""
    print(f"[TTS] Speaking: {text}")
    
    # Check for Piper
    # Assumes 'piper' is in PATH and a model exists. 
    # Current Default: Print only.
    # To enable: Download piper and a model (e.g., en_US-lessac-medium.onnx)
    
    # Simple check if we can run piper (requires configuration)
    # piper_cmd = f"echo {text} | piper --model en_US-lessac-medium.onnx --output_file output.wav"
    # if os.system(piper_cmd) == 0:
    #     winsound.PlaySound("output.wav", winsound.SND_FILENAME)


def audio_callback(indata, frames, time, status):
    """Callback for sounddevice"""
    if status:
        print(status, file=sys.stderr)
    audio_queue.put(indata.copy())

def process_audio():
    global is_recording, last_speech_time, buffer
    
    # Run Calibration First
    global THRESHOLD
    THRESHOLD = calibrate_noise()
    
    print(f"[INIT] Loading Faster-Whisper Model ({MODEL_SIZE} on {DEVICE})...")
    try:
        model = WhisperModel(MODEL_SIZE, device=DEVICE, compute_type=COMPUTE_TYPE)
        print("[INIT] Model Loaded Successfully.")
    except Exception as e:
        print(f"[ERROR] Failed to load model: {e}")
        return

    # Start stream
    with sd.InputStream(samplerate=SAMPLE_RATE, blocksize=BLOCK_SIZE, channels=1, callback=audio_callback):
        print("[READY] Listening... (Speak now)")
        
        while True:
            # 1. Read from queue
            try:
                data = audio_queue.get(timeout=0.1)
            except queue.Empty:
                continue
                
            # 2. Simple VAD (Voice Activity Detection)
            # Calculate volume (RMS)
            volume = np.linalg.norm(data) * 10
            
            # Check for max duration
            current_duration_sec = (len(buffer) * BLOCK_SIZE) / SAMPLE_RATE
            
            if volume > THRESHOLD:
                if not is_recording:
                    print("[Mic] Voice Detected...")
                    is_recording = True
                    play_beep(800, 100) # Start beep
                    buffer = [] # Clear buffer
                
                last_speech_time = time.time()
                buffer.append(data)
            
            elif is_recording:
                # We are recording but volume is low -> Silence
                buffer.append(data) # Keep recording silence briefly
                
                # Stop if silence OR if we exceeded max duration
                if (time.time() - last_speech_time > SILENCE_DURATION) or (current_duration_sec > MAX_RECORD_SECONDS):
                    # Speech ended
                    reason = "Max duration" if current_duration_sec > MAX_RECORD_SECONDS else "Silence"
                    print(f"[Mic] {reason} limit reached. Transcribing...")
                    is_recording = False
                    
                    # Flatten buffer
                    audio_data = np.concatenate(buffer, axis=0)
                    
                    # Convert to float32 expected by whisper
                    audio_data = audio_data.flatten().astype(np.float32)
                    
                    # Transcribe
                    print(f"[Debug] Buffer size: {len(audio_data)} samples. Calling model.transcribe...")
                    start_t = time.time()
                    segments, info = model.transcribe(
                        audio_data, 
                        beam_size=1, 
                        condition_on_previous_text=False,
                        temperature=0.0,
                        initial_prompt="Walk. Stop. Back. Left. Right." # Bias towards commands
                    ) 
                    
                    full_text = " ".join([segment.text for segment in segments]).strip()
                    print(f"[Debug] Transcription took {time.time()-start_t:.2f}s")
                    print(f"[Transcription] '{full_text}' (Prob: {info.language_probability:.2f})")
                    
                    # Hallucination Filter: Ignore if too long or no command matched
                    word_count = len(full_text.split())
                    text_lower = full_text.lower()
                    
                    # PRIORITY: Always process if it contains "stop" or "halt"
                    critical_stop = "stop" in text_lower or "halt" in text_lower
                    
                    if word_count > 10 and not critical_stop:
                        print(f"[Ignored] Text too long ({word_count} words) - likely noise hallucination.")
                    elif len(full_text) > 1:
                        handle_command(full_text)
                    else:
                        print("[Ignored] Too short.")
                    
                    print("\n[READY] Listening...")

def handle_command(text):
    text_lower = text.lower()
    
    # 0. Safety Priority: Check for STOP first
    if "stop" in text_lower or "halt" in text_lower:
        print(f"[Intent] SAFETY STOP TRIGGERED")
        send_to_server("STOP", text)
        play_success()
        speak("Stopping")
        return

    # 1. Match Intent
    matched_cmd = None
    for key, cmd in COMMAND_MAP.items():
        if key in text_lower:
            matched_cmd = cmd
            break
            
    if matched_cmd:
        print(f"[Intent] Matched: {matched_cmd}")
        send_to_server(matched_cmd, text)
        play_success()
        speak(f"Executing {matched_cmd}")
    else:
        print("[Intent] No matching command found.")
        # speak("I didn't understand that command.")

def send_to_server(cmd, text):
    try:
        payload = {"cmd": cmd, "text": text}
        res = requests.post(SERVER_URL, json=payload, timeout=2)
        if res.status_code == 200:
            print(f"[Server] Success: {res.json()}")
        else:
            print(f"[Server] Error {res.status_code}: {res.text}")
    except Exception as e:
        print(f"[Server] Connection Failed: {e}")

if __name__ == "__main__":
    try:
        process_audio()
    except KeyboardInterrupt:
        print("\n[STOP] Stopping...")
