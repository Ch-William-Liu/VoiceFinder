# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import sys
import time
import csv
import queue
import threading
import subprocess
from datetime import datetime
import numpy as np
import soundfile as sf
import sounddevice as sd
from scipy import signal as sig
import tkinter as tk
from tkinter.scrolledtext import ScrolledText

# ====================================
# Global setting
# ====================================
fs = 96000
cal_fs = fs
cal_soundspeed = 340

RECORD_SECONDS = 5
RECORD_CHANNELS = 8
DEVICE_NAME = "audioinjector-octo-soundcard"

SAVE_ROOT = "./record_output"
REF_WHISTLE_PATH = "210909_bandpass_specsub.wav"

# ====================================
# Array setting
# ====================================
micSpacing = {
    (0 , 1) : 0.2475,
    (2 , 4) : 0.2475,
    (1 , 3) : 0.2475,
    (3 , 4) : 0.2475
}

micPositions = {
    1 : (0 , 1),
    2 : (1 , 1),
    3 : (0 , 0),
    4 : (1 , 0)
}

verticalPairs = {(1 , 3) , (2 , 4)}
horziontalPairs = {(1 , 2) , (3 , 4)}
neighborPairs = [(1 , 2) , (2 , 4) , (1 , 3) , (3 , 4)]

# ====================================
# Record part
# ====================================
def checkSoundCard():
    '''
    Check soundcard everytime program starts.
    '''
    print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Checking AudioInjector Octo soundcard...")
    checkResult = subprocess.run(["arecord" , "-l"] , capture_output = True , text = True)
    stderr = checkResult.stderr.strip()
    stdout = checkResult.stdout.strip()

    if "no soundcards found" in stderr.lower():
        print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} No soundcard is found. Syste, would reboot soon.")
        os.system("sudo reboot")
        sys.exit(1)

def recordSetup(device_name: str = DEVICE_NAME):
    '''
    Setup function for audioinjector soundcard
    '''
    try:
        sd.default.device = device_name
        print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Setting default record to Audioinjector octo.")
    except Exception as e:
        raise RuntimeError(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Cannot set device to {device_name} , {e}")
    
    print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Warm-up recording...")
    sd.rec(int(5 * fs) , samplerate=fs , channels=RECORD_CHANNELS , dtype="float32")
    sd.wait()
    time.sleep(0.5)
    print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Warm-up done.")

def multiRecord(channel: int = RECORD_CHANNELS , duration: float=RECORD_SECONDS):
    '''
    Record multi-channel audio from AudioInjector Octo.
    '''
    print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Start recording for {duration:.1f} seconds.")
    audio = sd.rec(int(duration * fs) , samplerate=fs , channels=channel , dtype="float32")
    sd.wait()
    print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Recording done.")
    return audio

# ====================================
# Angle calculation
# ====================================

def calculateAngle(delta_t , d , C , use_cos = False):
    val = (delta_t * C) / d
    val = np.clip(val , -1.0 , 1.0)
    phiRad = math.acos(val) if use_cos else math.asin(val)
    return math.degrees(phiRad) , val

def detectWhistle(signal_data):
    ref_whistle , ref_whistle_fs = sf.read(REF_WHISTLE_PATH)
    corr = sig.correlate(signal_data , ref_whistle , mode="valid")
    peakIdx = np.argmax(np.abs(corr))
    return peakIdx

def getSignedDelay(sig_a , sig_b):
    corr = sig.correlate(sig_a , sig_b , mode="full")
    delays = np.arange(-len(sig_a) + 1 , len(sig_a))
    delay_sample = delays[np.argmax(np.abs(corr))]
    delay_time = delay_sample / cal_fs
    return delay_sample , delay_time

def processFile(audio):
    read_data = audio
    read_fs = fs

    if read_fs != cal_fs:
        raise ValueError(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Sample rate mismatch: excepted {cal_fs} Hz , got {read_fs} Hz.")
    if read_data.shape[1] < 4:
        raise ValueError(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Insufficient channels in the audio. At least 4 channels are required.")
    
    dataChannel = {
        1 : read_data[int(0.3 * cal_fs): , 0],
        2 : read_data[int(0.3 * cal_fs): , 1],
        3 : read_data[int(0.3 * cal_fs): , 2],
        4 : read_data[int(0.3 * cal_fs): , 3]
    }

    b , a = sig.ellip(3 , 3, 50 , [5000 / (0.5 * cal_fs) , 25000 / (0.5 * cal_fs)] , btype = "bandpass")

    for c in dataChannel:
        dataChannel[c] = sig.filtfilt(b , a , dataChannel[c])

    arrival_sample = {k : detectWhistle(v) for k , v in dataChannel.items()}
    sortedMic = sorted(arrival_sample.items() , key = lambda x: x[1])
    arrival_order = [f"Mic{m} ({s / cal_fs:.4f} s)" for m, s in sortedMic]

    print("Arrival order:")
    print(" -> ".join(arrival_order))

    peak = min(arrival_sample.values())
    startIdx = max(0 , peak)
    endIdx = min(len(dataChannel[1]) , startIdx + (2 * cal_fs))

    for c in dataChannel:
        dataChannel[c] = dataChannel[c][startIdx:endIdx]

    d_h = micSpacing[(1 , 2)]
    d_v = micSpacing[(1 , 3)]

    delay12_samp , tau12 = getSignedDelay(dataChannel[1] , dataChannel[2])          # top row
    delay34_samp , tau34 = getSignedDelay(dataChannel[3] , dataChannel[4])          # bottom row
    delay13_samp , tau13 = getSignedDelay(dataChannel[1] , dataChannel[3])          # left col
    delay24_samp , tau24 = getSignedDelay(dataChannel[2] , dataChannel[4])          # rignt col

    print(f"(1 , 2): {delay12_samp} samples , {tau12:.4f} s")
    print(f"(3 , 4): {delay34_samp} samples , {tau34:.4f} s")
    print(f"(1 , 3): {delay13_samp} samples , {tau13:.4f} s")
    print(f"(2 , 4): {delay24_samp} samples , {tau24:.4f} s")

    tau_x = (tau12 + tau34) / 2.0
    tau_y = (tau13 + tau24) / 2.0

    print(f"tau_x = {tau_x:.8f} s")
    print(f"tau_y = {tau_y:.8f} s")

    ux = -(cal_soundspeed * tau_x) / d_h
    uy = -(cal_soundspeed * tau_y) / d_v

    ux = np.clip(ux, -1.0, 1.0)
    uy = np.clip(uy, -1.0, 1.0)

    print(f"ux = {ux:.4f}")
    print(f"uy = {uy:.4f}")

    # 0° = up, clockwise positive
    mean_angle = (math.degrees(math.atan2(ux, uy)) + 360) % 360

    corrected_angles = {
        (1, 2): math.degrees(math.asin(np.clip(-(cal_soundspeed * tau12) / d_h, -1.0, 1.0))),
        (3, 4): math.degrees(math.asin(np.clip(-(cal_soundspeed * tau34) / d_h, -1.0, 1.0))),
        (1, 3): math.degrees(math.asin(np.clip(-(cal_soundspeed * tau13) / d_v, -1.0, 1.0))),
        (2, 4): math.degrees(math.asin(np.clip(-(cal_soundspeed * tau24) / d_v, -1.0, 1.0))),
    }

    print("Pair angles:")
    for pair, ang in corrected_angles.items():
        print(f"  {pair}: {ang:.2f} deg")

    print(f"Mean angle = {mean_angle:.2f} deg")

    return corrected_angles, startIdx / cal_fs, arrival_order, mean_angle

# ====================================
# Save Helpers
# ====================================
def ensure_save_folder(root_dir=SAVE_ROOT):
    os.makedirs(root_dir, exist_ok=True)
    return root_dir

def make_timestamp_name():
    return datetime.now().strftime("%Y%m%d_%H%M%S")

def save_audio_file(audio, root_dir=SAVE_ROOT):
    ensure_save_folder(root_dir)
    stamp = make_timestamp_name()
    wav_path = os.path.join(root_dir, f"{stamp}.wav")
    sf.write(wav_path, audio, fs)
    print(f"[Save] Audio saved: {wav_path}")
    return wav_path, stamp

def append_result_csv(stamp, mean_angle, start_sec, arrival_order, root_dir=SAVE_ROOT):
    ensure_save_folder(root_dir)
    csv_path = os.path.join(root_dir, "Finder_result.csv")
    file_exists = os.path.exists(csv_path)

    with open(csv_path, mode="a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(["Timestamp", "Mean_angle_deg", "Start_sec", "Arrival_order"])
        writer.writerow([stamp, f"{mean_angle:.2f}", f"{start_sec:.4f}", " | ".join(arrival_order)])

    print(f"[Save] Result appended to: {csv_path}")

# ====================================
# GUI Log Redirect
# ====================================
class TextRedirector:
    def __init__(self , log_queue):
        self.log_queue = log_queue

    def write(self , text):
        if text:
            self.log_queue.put(text)

    def flush(self):
        pass

# ====================================
# GUI
# ====================================
class FinderGUI:
    def __init__(self , root):
        self.root = root
        self.root.title("Finder User GUI")
        self.root.geometry("1100x650")
        self.root.minsize(950 , 600)

        self.running = False
        self.working_thread = None
        self.current_angle = 0.0
        self.is_ready = False

        self.log_queue = queue.Queue()
        self.original_stdout = sys.stdout
        self.original_stderr = sys.stderr
        sys.stdout = TextRedirector(self.log_queue)
        sys.stderr = TextRedirector(self.log_queue)

        self.build_layout()
        self.root.after(100 , self.poll_log_queue)
        self.root.protocol("WM_DELETE_WINDOW" , self.on_close)

        self.initialize_system_async()

    def build_layout(self):
        top_frame = tk.Frame(self.root , bd=2 , relief="solid")
        top_frame.pack(side="top" , fill="x")

        self.btn_single = tk.Button(
            top_frame , text="Run Single Cycle" , width=14,
            command=self.run_single
        )
        self.btn_single.pack(side="left" , padx=40 , pady=10)

        self.btn_loop = tk.Button(
            top_frame , text="Start Continuous Mode" , width=20,
            command=self.start_loop
        )
        self.btn_loop.pack(side="left" , padx=140, pady=10)

        self.btn_stop = tk.Button(
            top_frame , text="Stop", width=14,
            command=self.stop_loop
        )
        self.btn_stop.pack(side="right" , padx=40 , pady=10)

        main_frame = tk.Frame(self.root , bd=2 , relief="solid")
        main_frame.pack(fill="both" , expand=True)

        left_frame = tk.Frame(main_frame , bd = 1 , relief="solid")
        left_frame.pack(side="left", fill="both", expand=True)

        right_frame = tk.Frame(main_frame , bd = 1 , relief="solid")
        right_frame.pack(side="right" , fill="both" , expand=True)

        self.left_hint = tk.Label(
            left_frame,
            text="Real-time Angle Display",
            fg="red",
            justify="left",
            font=("Microsoft JhengHei", 14)
        )

        self.angle_var = tk.StringVar(value="Current Angle: 0.00°")
        self.angle_label = tk.Label(
            left_frame,
            textvariable=self.angle_var,
        )

        self.canvas = tk.Canvas(left_frame , bg="white" , highlightthickness=0)
        self.canvas.pack(fill="both" , expand=True , padx=20 , pady=20)
        self.canvas.bind("<Configure>",self.on_canvas_resize)

        self.right_hint = tk.Label(
            right_frame,
            text="Processing Logs",
            fg="red",
            justify="left",
            font=("Microsoft JhengHei", 14)
        )
        self.right_hint.pack(anchor="nw", padx=10, pady=10)

        self.log_text = ScrolledText(
            right_frame,
            wrap="word",
            font=("Consolas", 11)
        )
        self.log_text.pack(fill="both", expand=True, padx=10, pady=10)

        self.draw_dial()
        self.update_dolphin_icon(self.current_angle)

    def initialize_system_async(self):
        def worker():
            try:
                print("[System] Initialization started.")
                ensure_save_folder(SAVE_ROOT)
                checkSoundCard()
                recordSetup()
                self.is_ready = True
                print("[System] Initialization finished.")
            except Exception as e:
                print(f"[ERROR] Initialization failed: {e}")
                self.is_ready = False

        threading.Thread(target=worker, daemon=True).start()

    def on_canvas_resize(self, event):
        self.draw_dial()
        self.update_dolphin_icon(self.current_angle)

    def draw_dial(self):
        self.canvas.delete("all")

        w = max(self.canvas.winfo_width(), 300)
        h = max(self.canvas.winfo_height(), 300)
        size = min(w, h) * 0.75

        self.cx = w / 2
        self.cy = h / 2
        self.r = size / 2

        self.canvas.create_oval(
            self.cx - self.r, self.cy - self.r,
            self.cx + self.r, self.cy + self.r,
            width=2
        )

        self.canvas.create_line(self.cx, self.cy - self.r, self.cx, self.cy + self.r,
                                dash=(4, 2), fill="gray")
        self.canvas.create_line(self.cx - self.r, self.cy, self.cx + self.r, self.cy,
                                dash=(4, 2), fill="gray")

        self.canvas.create_text(self.cx, self.cy - self.r - 15, text="0°", font=("Consolas", 12, "bold"))
        self.canvas.create_text(self.cx + self.r + 20, self.cy, text="90°", font=("Consolas", 12, "bold"))
        self.canvas.create_text(self.cx, self.cy + self.r + 15, text="180°", font=("Consolas", 12, "bold"))
        self.canvas.create_text(self.cx - self.r - 22, self.cy, text="270°", font=("Consolas", 12, "bold"))

        self.canvas.create_oval(self.cx - 4, self.cy - 4, self.cx + 4, self.cy + 4, fill="black")

        self.angle_line = self.canvas.create_line(
            self.cx, self.cy, self.cx, self.cy - self.r + 25,
            width=2, fill="blue"
        )
        self.dolphin_text = self.canvas.create_text(
            self.cx, self.cy - self.r + 30,
            text="🐬",
            font=("Segoe UI Emoji", 22)
        )

    def angle_to_xy(self, angle_deg, radius_offset=30):
        theta = math.radians(angle_deg)
        x = self.cx + (self.r - radius_offset) * math.sin(theta)
        y = self.cy - (self.r - radius_offset) * math.cos(theta)
        return x, y
    
    def update_dolphin_icon(self, angle_deg):
        self.current_angle = angle_deg % 360
        self.angle_var.set(f"Current Angle: {self.current_angle:.2f}°")

        if not hasattr(self, "cx"):
            return

        x, y = self.angle_to_xy(self.current_angle)
        self.canvas.coords(self.angle_line, self.cx, self.cy, x, y)
        self.canvas.coords(self.dolphin_text, x, y)

    def append_log(self, text):
        self.log_text.insert("end", text)
        self.log_text.see("end")

    def poll_log_queue(self):
        while not self.log_queue.empty():
            text = self.log_queue.get_nowait()
            self.append_log(text)
        self.root.after(100, self.poll_log_queue)

    def one_cycle(self):
        if not self.is_ready:
            print("[WARN] System is not ready yet.")
            return

        print("=" * 60)
        print(f"[Flow] {datetime.now().strftime('%H:%M:%S')} Start one cycle.")

        audio = multiRecord(channel=RECORD_CHANNELS, duration=RECORD_SECONDS)

        corrected_angles, startSec, arrival_order, mean_angle = processFile(audio)

        self.root.after(0, self.update_dolphin_icon, mean_angle)

        wav_path, stamp = save_audio_file(audio, SAVE_ROOT)
        append_result_csv(stamp, mean_angle, startSec, arrival_order, SAVE_ROOT)

        print(f"[Flow] Display angle: {mean_angle:.2f} deg")
        print(f"[Flow] Saved wav: {wav_path}")
        print(f"[Flow] {datetime.now().strftime('%H:%M:%S')} One cycle finished.")
        print("=" * 60)

    def run_single(self):
        def worker():
            try:
                self.one_cycle()
            except Exception as e:
                print(f"[ERROR] Single run failed: {e}")

        threading.Thread(target=worker, daemon=True).start()

    def start_loop(self):
        if self.running:
            print("[INFO] Loop is already running.")
            return

        if not self.is_ready:
            print("[WARN] System is not ready yet.")
            return

        self.running = True
        print("[INFO] Infinite loop started.")

        self.worker_thread = threading.Thread(target=self.loop_worker, daemon=True)
        self.worker_thread.start()

    def stop_loop(self):
        if not self.running:
            print("[INFO] Loop is not running.")
            return

        self.running = False
        print("[INFO] Stop signal sent.")

    def loop_worker(self):
        start_time = time.time()

        while self.running:
            try:
                self.one_cycle()
            except Exception as e:
                print(f"[ERROR] Loop cycle failed: {e}")
        print("[INFO] Infinite loop stopped.")

    def on_close(self):
        self.running = False
        sys.stdout = self.original_stdout
        sys.stderr = self.original_stderr
        self.root.destroy()


def main():
    root = tk.Tk()
    FinderGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()