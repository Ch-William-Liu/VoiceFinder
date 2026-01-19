# !/usr/bin/env python3
# -*- utf-8 -*-

import subprocess
import math
import numpy as np
import soundfile as sf
import sounddevice as sd
import os
from pywmm import WMMv2
from scipy import signal as sig
import time
from datetime import datetime
import csv
import RPi.GPIO as GPIO
import sys
import serial
import st6100_send_msg
import read_ESP32

# ====================================
# Global Setting
# ====================================
fs = 96000
GPIO.setwarnings(False)
serST6100 = serial.Serial(port = "/dev/ttyUSB0" , baudrate = 9600 , timeout = 1)
serESP = serial.Serial(port = "/dev/serial0" , baudrate = 115200 , timeout = 1)

DONE_GPIO = 27                                      # goes to PIN34 on ESP32
RUN_DURATION = 525                                  # take 60 second as test
RED_LED = 17                                        # RED LED pin indicate that Raspberry is booted

GPIO.setmode(GPIO.BCM)
GPIO.setup(DONE_GPIO , GPIO.OUT , initial = GPIO.LOW)
GPIO.setup(RED_LED , GPIO.OUT , initial = GPIO.HIGH)

# ====================================
# Recording
# ====================================
def checkSoundcard():
    '''
    check soundcard every time program start
    '''
    print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Checking Audioinjector Octo soundcard...")
    checkResult = subprocess.run(["arecord" , "-l"] , capture_output = True , text = True)
    stderr = checkResult.stderr.strip()
    stdout = checkResult.stdout.strip()

    if "no soundcards found" in stderr.lower():
        print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} No soundcard is found. System would reboot.")
        os.system("sudo reboot")
        sys.exit(1)

def recordSetup(device_name : str = "audioinjector-octo-soundcard"):
    '''
    set up function for audioinjector soundcard
    :param device_name: soundcard name
    :type device_name: str
    '''
    try:
        sd.default.device = device_name
        print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Setting default record to Audioinjector Octo.")
    except Exception as e:
        raise RuntimeError(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Cannot set device to {device_name} , {e}")
    print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Warm-up recording.")
    sd.rec(int(5 * fs) , samplerate = fs , channels = 8)
    sd.wait()
    time.sleep(0.5)
    print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Dummy recording done.")

def multiRecord(channel : int = 8 , duration : float = 10.0):
    '''
    use audioinjector octo to record audio
    :param channel: input channel , should equal to 8
    :type channel: int
    :param duration: duration of audio file
    :type duration: float
    '''
    print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Start recording for {duration} second ...")
    audio = sd.rec(int(duration * fs) , samplerate = fs , channels = channel , dtype = "float32")
    sd.wait()
    print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Recorded.")

    return audio

# ====================================
# Declination 
# ====================================
def getLocalDeclination(lat : float , lon : float):
    '''
    use GPS coordinate from ST6100
    :param lat: coodrinate from st6100 , ddmm.mmmm
    :type lat: float
    :param lon: coodrinate from st6100 , ddmm.mmmm
    :type lon: float
    '''
    print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Getting declination from current coorinate.")
    lat_d = int(lat // 100)
    lat_m = lat % 100
    lon_d = int(lon // 100)
    lon_m = lon % 100

    lat_format = lat_d + (lat_m / 60)
    lon_format = lon_d + (lon_m / 60)
    alt_km = 0
    yearNow = datetime.now().year + (datetime.now().timetuple().tm_yday) / 365.0
    wmm = WMMv2()
    dec = wmm.get_declination(lat_format , lon_format , yearNow , alt_km)

    return dec

# ====================================
# Acoustic array calculate
# ====================================
cal_fs = fs
cal_soundspeed = 1520
# spacing of hyddrophone array
micSpacing = {
    (1 , 2) : 0.325,
    (2 , 4) : 1.25, 
    (1 , 3) : 1.25,
    (3 , 4) : 0.261
}

# actual hydrophone positions
micPositions = {
    1 : (0 , 1),        # upper-left
    2 : (1 , 1),        # upper-right
    3 : (0 , 0),        # lower-left
    4 : (1 , 0)         # lower-right
}

# make the horizontal or vertical mic pair list
verticalPairs = {(1 , 3) , (2 , 4)}
horizontalPairs = {(1 , 2) , (3 , 4)}
neighborPairs = [(1 , 2) , (2 , 4) , (1 , 3) , (3 , 4)]


# function to calculate angle from TDOA
def calculateAngle(delta_t , d , C , use_cos = False):
    val = (delta_t * C) / d                 # calculate the value of trigonometric function
    val = np.clip(val , -1.0 , 1.0)        # clip the value to avoid error
    phiRad = math.acos(val) if use_cos else math.asin(val)    # calculate the angle in radian
    return math.degrees(phiRad) , val

# generate chitp to detect whenever signal arrive
def generateChirp(f_low = 3000 , f_high = 7000 , chirp_duration = 0.5):
    t= np.linspace(0 , chirp_duration , int(cal_fs * chirp_duration) , endpoint = False)
    return sig.chirp(t , f0 = f_low , f1 = f_high , t1 = chirp_duration , method = "linear")

# detect the chirp
def detectChirp(signal , template):
    corr = sig.correlate(signal , template , mode = "valid")
    peadIdx = np.argmax(np.abs(corr))
    return peadIdx

# main calculation function
def processFile(audio):
    read_data = audio
    read_fs = fs
    if read_fs != cal_fs:
        raise ValueError(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Sample rate mismatch: expected {cal_fs} Hz, got {read_fs} Hz")
    if read_data.shape[1] < 4:
        raise ValueError(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Insufficient channels in the audio file. At least 4 channels are required.")
    
    dataChannel = {
        1 : read_data[int(0.3 * cal_fs) : , 0],
        2 : read_data[int(0.3 * cal_fs) : , 1],
        3 : read_data[int(0.3 * cal_fs) : , 2],
        4 : read_data[int(0.3 * cal_fs) : , 3]
    }

    # bandpass filter design
    b , a = sig.ellip(3, 3 , 50 , [3000 / (0.5 * cal_fs) , 7000 / (0.5 * cal_fs)] , btype = "bandpass")
    for c in dataChannel:
        dataChannel[c] = sig.filtfilt(b , a , dataChannel[c])

    # detect chirp arrival time
    template_chirp = generateChirp()
    arrivals_sample = {k : detectChirp(v , template_chirp) for k , v in dataChannel.items()}
    sortedMic = sorted(arrivals_sample.items() , key = lambda x: x[1])
    arrival_order = [[f"Mic{m} ({s / cal_fs : .4f} s)" for m , s in sortedMic]]

    # get chitp section
    peak = min(arrivals_sample.values())
    startIdx = max(0 , peak)
    endIdx = min(len(dataChannel[1]) , startIdx + cal_fs)

    for c in dataChannel:
        dataChannel[c] = dataChannel[c][startIdx : endIdx]

    # detect signal arrive which mic first
    # use mic1 as reference
    delay_idx = []
    delay_tmp = np.arange(-len(dataChannel[1]) + 1 , len(dataChannel[1]))
    for c in dataChannel:
        arrTime = sig.correlate(dataChannel[c] , dataChannel[1] , mode = "full")
        arrIdx = delay_tmp[np.argmax(np.abs(arrTime))]
        delay_idx.append(arrIdx)
    earliest_mic = np.argmin(delay_idx) + 1
    no_rotate = len(set(delay_idx)) == 1

    # only consider the case of neighboring hydrophone pair
    corrected_angles = {}
    vectors = []
    formulas = []
    print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Calculating angle...")
    for mic_a , mic_b in neighborPairs:
        corr = sig.correlate(dataChannel[mic_a] , dataChannel[mic_b] , mode = "full")
        delays = np.arange(-len(dataChannel[mic_a]) + 1 , len(dataChannel[mic_a]))
        delay = delays[np.argmax(np.abs(corr))]
        dt = np.abs(delay / cal_fs)
        spacing = micSpacing[(mic_a , mic_b)]
        use_cos = ((mic_a , mic_b) in verticalPairs) or ((mic_b , mic_a) in verticalPairs)

        angle ,  val = calculateAngle(dt , spacing , cal_soundspeed , use_cos)

        # rotate angle
        if not no_rotate:
            match earliest_mic:
                case 1: corrected_angle = (360 - angle) % 360
                case 2: corrected_angle = angle
                case 3: corrected_angle = (180 + angle) % 360
                case 4: corrected_angle = (90 + angle) % 360
                case _: corrected_angle = angle
        else:
            corrected_angle = angle
        
        corrected_angles[(mic_a , mic_b)] = corrected_angle

        trig_func = "cos" if use_cos else "sin"
        formula =  f"{trig_func}(Φ) = Δt × C₀ / d = {dt : .6f} × {cal_soundspeed} / {spacing} = {val:.4f} → Φ = {corrected_angle:.2f}°"
        formulas.append(f"Mic{mic_a}→Mic{mic_b}: {formula}")

        theta_rad = math.radians(corrected_angle)
        vec = np.array([math.cos(theta_rad) , math.sin(theta_rad)]) if not use_cos else np.array([math.sin(theta_rad) , math.cos(theta_rad)])
        vectors.append(vec)

    # average angle and return
    avg_vec = np.mean(vectors , axis = 0)
    mean_angle = math.degrees(math.atan2(avg_vec[1] , avg_vec[0])) % 360

    return corrected_angles , startIdx / cal_fs , arrival_order , mean_angle , formulas

# ====================================
# log
# ====================================
resultLogName = "Finder_result.csv"
def logCsv(timestamp , cal_ang , sensor_ang , true_ang):
    file_exist = os.path.exists(resultLogName)
    with open(resultLogName , mode = "a" , newline = "") as f:
        writer = csv.writer(f)
        if not file_exist:
            writer.writerow(["timestamp" , "cal_ang_deg" , "sensor_ang_deg" , "true_ang_deg"])

        writer.writerow([f"{timestamp}" , f"{cal_ang : 3.2f}" , f"{sensor_ang : 3.2f}" , f"{true_ang : 3.2f}"])

# ====================================
# main
# ====================================
def main():
    print(f"[Pi] {datetime.now().strftime('%H:%M:%S')}  ==== VoiceFinder system start. ====")
    checkSoundcard()
    recordSetup()

    esp = read_ESP32.ESPHeadingReader(ser = serESP)
    start_time = time.time()

    closeFlag = False
    msgcount = 700
    try:
        while not closeFlag:
            print("\n")
            print("=" * 40)
            timeNow = time.strftime("%Y%m%d%H%M%S" ,  time.localtime())
            audioFilename = timeNow + ".wav"

            currentCoordinate = st6100_send_msg.get_gps_info(serST6100 , retries = 20 , wait_time = 120 , stale_secs = 60)
            localLat = float(currentCoordinate[0]) * 0.01
            localLon = float(currentCoordinate[2]) * 0.01

            dec = getLocalDeclination(localLat , localLon)

            myAudio = multiRecord()

            print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Proccessing audio file...")
            _ , _ , _ , cal_mean_angle , _ = processFile(myAudio)
            sensor_angle = esp.get_mean()
            true_angle = (cal_mean_angle + sensor_angle + dec) % 360
            print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Processed.")
            print(f"\nCal: {cal_mean_angle : 3.2f}\tSensor: {sensor_angle : 3.2f}\tTrue: {true_angle : 3.2f}")
            logCsv(timeNow , cal_mean_angle , sensor_angle , true_angle)
            print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Saving audio file as {audioFilename}")
            sf.write(audioFilename , myAudio , fs , subtype = "PCM_16")
            del(myAudio)

            # msg_to_send = f"VF,{cal_mean_angle:3.2f},{sensor_angle:3.2f},{true_angle:3.2f}"
            # msg_to_send = f"{cal_mean_angle:3.2f},{sensor_angle:3.2f},{true_angle:3.2f}"
            buffer.append(true_angle)

            if len(buffer) >= 10:
                msg_to_send = ",".join(f"{v:.2f}" for v in buffer)
                st6100_send_msg.st6100_send_msg(msg_id=msgcount , msg = msg_to_send)
                print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Sending message from Pi to satellite.")
                msgcount += 1
                buffer = []
            
            print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Current buffer length: {len(buffer)} , wait buffer length = 10 to send message.")
            print("=" * 50)
            elasped_time = time.time() - start_time
            if elasped_time >= RUN_DURATION:
                closeFlag = True
            
            if msgcount > 710:
                msgcount = 700

    except KeyboardInterrupt:
        print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Interrupt by user.")
    except Exception as e:
        print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Fatal Error occured: {e}")
    finally:
        print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Reach max run time ({RUN_DURATION}). Send DONE to ESP32.")
        print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} DONE signal sent. Program will shutdown soon.")
        print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Program exiting normally")
        print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} ==== Finder System program ended ====")
        esp.close()
        GPIO.output(DONE_GPIO , GPIO.HIGH)
        GPIO.output(RED_LED , GPIO.LOW)
        time.sleep(0.7)
        GPIO.output(DONE_GPIO , GPIO.LOW)

if __name__ == "__main__":
    main()
