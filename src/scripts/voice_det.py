#!/usr/bin/env python

import sounddevice as sd
import numpy as np
import wave
from faster_whisper import WhisperModel
import queue
import threading
import os
import time
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import scipy.signal as signal

audio_queue = queue.Queue()

def apply_noise_filter(audio_data, sample_rate=16000):

    if len(audio_data) < 100:  # Minimum length requirement
        return audio_data
        
    # Bandpass filter (human voice typically between 100Hz and 3000Hz)
    nyquist = sample_rate / 2
    low = 100 / nyquist
    high = 3000 / nyquist
    # Use a lower order filter for shorter segments
    order = min(4, len(audio_data) // 10)
    b, a = signal.butter(order, [low, high], btype='band')
    
    try:
        filtered_audio = signal.filtfilt(b, a, audio_data)
        return filtered_audio
    except ValueError:
        return audio_data

def is_voice_detected(indata, threshold=0.01, sample_rate=16000):
    if len(indata.flatten()) < 100:
        return np.max(np.abs(indata)) > threshold
    
    try:
        # Apply bandpass filter
        filtered = apply_noise_filter(indata.flatten(), sample_rate)        
        freqs = np.fft.fftfreq(len(filtered), 1/sample_rate)
        spectrum = np.abs(np.fft.fft(filtered))
        
        # Focus on voice frequency range (100-3000 Hz)
        voice_mask = (freqs >= 100) & (freqs <= 3000)
        voice_energy = np.sum(spectrum[voice_mask])
        total_energy = np.sum(spectrum)
        
        voice_ratio = voice_energy / total_energy if total_energy > 0 else 0
        
        return voice_ratio > 0.3 and np.max(np.abs(filtered)) > threshold
    
    except Exception as e:
        # Fallback to simple amplitude threshold
        return np.max(np.abs(indata)) > threshold

def record_audio(duration=5, sample_rate=16000):
    print("Listening for voice...")

    audio_buffer = []
    recording = False
    silence_threshold = 0.5  # seconds of silence to stop recording
    silence_counter = 0
    
    def audio_callback(indata, frames, time, status):
        nonlocal recording, silence_counter
        
        if status:
            print(status)

        if is_voice_detected(indata, threshold=0.01, sample_rate=sample_rate):
            if not recording:
                print("Voice detected! Recording...")
                recording = True
            silence_counter = 0

            filtered_audio = apply_noise_filter(indata.flatten())
            audio_buffer.extend(filtered_audio)
        elif recording:
            silence_counter += frames/sample_rate
            if silence_counter >= silence_threshold:
                raise sd.CallbackStop()

            filtered_audio = apply_noise_filter(indata.flatten())
            audio_buffer.extend(filtered_audio)
    
    with sd.InputStream(samplerate=sample_rate,
                       channels=1,
                       dtype=np.float32,
                       callback=audio_callback):
        try:
            sd.sleep(int(duration * 1000)) 
        except sd.CallbackStop:
            pass
    
    if not audio_buffer:
        return None

    audio_data = np.array(audio_buffer) * 32767
    audio_data = audio_data.astype(np.int16)
    
    print("Recording finished!")
    return audio_data

def save_audio(audio_data, sample_rate=16000):
    audio_dir = "src/audio_files"
    os.makedirs(audio_dir, exist_ok=True)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{audio_dir}/recording_{timestamp}.wav"
    
    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(audio_data.tobytes())
    
    return filename

def transcribe_audio(audio_file):
    model = WhisperModel("base", device="cpu", compute_type="int8")
    segments, _ = model.transcribe(audio_file,language="en")
    text = " ".join([segment.text for segment in segments])
    return text.strip()

def analyze_command(text):
    tools = {
        'screwdriver',
        'wrench',
        'hammer',
        'screw',
        'nail',
        'pliers',
        'drill'
    }

    pick_words = {'grab', 'give', 'hand', 'pass', 'pick'}
    place_words = {'take', 'put', 'place', 'set', 'down'}
    
    text = text.lower()
    words = text.split()

    found_tools = [t for t in tools if t in text]
    
    is_pick = any(word in pick_words for word in words)
    is_place = any(word in place_words for word in words)
    
    return (found_tools, is_pick, is_place)

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.publisher = self.create_publisher(String, 'cmdpickplace', 10)

def process_audio_queue(node):
    while True:
        try:
            audio_file = audio_queue.get(timeout=1)
            
            try:
                print(f"Processing {audio_file}...")
                transcribed_text = transcribe_audio(audio_file)
                print(f"Transcribed text: {transcribed_text}")

                tools, is_pick, is_place = analyze_command(transcribed_text)
                
                if tools:
                    command_type = []
                    if is_pick:
                        command_type.append("PICK")
                    if is_place:
                        command_type.append("PLACE")
                    
                    if command_type:
                        command_str = ' & '.join(command_type)
                        for tool in tools:
                            print(f"Command: {command_str} - Tool: {tool}")
                            # Publish command to ROS2 topic
                            msg = String()
                            msg.data = f"{command_str}:{tool}"
                            node.publisher.publish(msg)
                            node.get_logger().info(f"Published command: {msg.data}")
                    else:
                        for tool in tools:
                            print(f"Tool mentioned: {tool} (no pick/place command)")
                else:
                    print("No tool mentioned in the text")
                
                os.remove(audio_file)
                print(f"Deleted {audio_file}")
                
            except Exception as e:
                print(f"Error processing {audio_file}: {str(e)}")
                if os.path.exists(audio_file):
                    os.remove(audio_file)
            
            finally:
                audio_queue.task_done()
                
        except queue.Empty:
            time.sleep(0.1)
            continue
        except Exception as e:
            print(f"Queue processing error: {str(e)}")
            time.sleep(1)

def main():
    rclpy.init()
    node = VoiceCommandNode()
    
    processor_thread = threading.Thread(
        target=process_audio_queue,
        args=(node,),
        daemon=True
    )
    processor_thread.start()
    
    try:
        while True:
            audio_data = record_audio()
            if audio_data is not None:
                audio_file = save_audio(audio_data)
                audio_queue.put(audio_file)
            time.sleep(0.1) 
            rclpy.spin_once(node, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        print("\nStopping recording...")
        audio_queue.join()
        print("All files processed. Exiting.")
        rclpy.shutdown()

if __name__ == "__main__":
    main()