#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import csv
from collections import deque
import os
from datetime import datetime
import time

class ESPHeadingReader:
    HEADER = [
        "timestamp" , "mx_raw" , "my_raw" , "mz_raw" , 
        "roll_deg" , "pitch_deg" , "heading_raw_deg" , "heading_comp_deg" , 
        "ax_raw" , "ay_raw" , "az_raw" , 
        "gx_raw" , "gy_raw" , "gz_raw" , "temp"]
    
    def __init__(self , ser , logpath : str = "Finder_raw.csv" , maxlen : int = 200):
        self.ser = ser
        self.log_path = logpath
        self.buffer = deque(maxlen = maxlen)

        self.stop_event = threading.Event()
        self.lock = threading.Lock()

        self._ensure_log_header()
        self.thread = threading.Thread(target = self._reader_loop , daemon = True)
        self.thread.start()

    def _ensure_log_header(self):
        need_header = (not os.path.exists(self.log_path)) or (os.path.getsize(self.log_path) == 0)
        if need_header:
            with open(self.log_path , "a" , newline = "") as  f:
                w = csv.writer(f)
                w.writerow(self.HEADER)

    def _parse_csv_line(self , line : str):
        if not line:
            print(f"[ESP32] {datetime.now().strftime('%H:%M:%S')} Receive Error")
            return None
        
        if ("ESP32" in line) or ("MPU-6050" in line) or ("HMC-5883L" in line) or ("DS3231" in line):
            print(f"[ESP32] {datetime.now().strftime('%H:%M:%S')} Initialize message: " , line)
            return None
        
        line = line.replace('\r', '').strip()
        parts = [p.strip() for p in line.split(",")]

        if len(parts) < 15:
            print(f"[ESP32] {datetime.now().strftime('%H:%M:%S')} Incomplete message: " , line)
            return None
        
        # ts = parts[0]

        now_ts = datetime.now().strftime("%y/%m/%d %H:%M:%S")

        try:
            nums = [float(x) for x in parts[1 : 15]]
            
        except ValueError as e:
            print(f"[ESP32 Parsing Error] {datetime.now().strftime('%H:%M:%S')}: " , e)

        row = [now_ts] + nums
        return row
    
    def _append_log(self , row):
        with open(self.log_path , "a" , newline = "") as f:
            w = csv.writer(f)
            w.writerow(row)

    def _reader_loop(self):
        print(f"[ESP32] {datetime.now().strftime('%H:%M:%S')} Reader thread start...")
        while not self.stop_event.is_set():
            try:
                raw = self.ser.readline()

                if not raw:
                    continue

                text = raw.decode("utf-8" , errors = "ignore").strip()
                row = self._parse_csv_line(text)
                if row is None:
                    continue

                # write full row to csv
                self._append_log(row)

                # store heading_comp in buffer
                heading_comp = float(row[7])
                with self.lock:
                    self.buffer.append(heading_comp)

            except Exception as e:
                print(f"[ESP32 Reader Error] {datetime.now().strftime('%H:%M:%S')}: " , e)
                time.sleep(0.2)

        print(f"[ESP32] {datetime.now().strftime('%H:%M:%S')} Reader thread stop")

    def get_mean(self , n : int = 50):
        with self.lock:
            if not self.buffer:
                print(f"[ESP32] {datetime.now().strftime('%H:%M:%S')} There is nothing in buffer.")
                return -180
            data = list(self.buffer)[-n :]
        return sum(data) / len(data)
    
    def get_latest(self):
        with self.lock:
            if not self.buffer:
                print(f"[ESP32] {datetime.now().strftime('%H:%M:%S')} There is nothing in buffer.")
                return -180
            return self.buffer[-1]
        
    def close(self):
        self.stop_event.set()
        self.thread.join(timeout = 1.0)
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print(f"[ESP32] {datetime.now().strftime('%H:%M:%S')} Reader close.")
        except Exception:
            pass