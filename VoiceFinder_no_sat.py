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
import sys
import serial
import read_ESP32


# ====================================
# Global Setting
# ====================================
fs = 96000
serESP = serial.Serial(port = "/dev/serial0" , baudrate = 115200 , timeout = 1)
RUN_DURATION = 30 * 60 + 120

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
# cal_soundspeed = 1520
cal_soundspeed = 340
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

# used already known whistle to test 
def detectWhistle(signal):
    ref_whistle , ref_whistle_fs= sf.read("210909_bandpass_specsub.wav")
    corr = sig.correlate(signal , ref_whistle , mode="valid")
    peadIdx = np.argmax(np.abs(corr))
    return peadIdx

# detect the chirp
def detectChirp(signal , template):
    corr = sig.correlate(signal , template , mode = "valid")
    peadIdx = np.argmax(np.abs(corr))
    return peadIdx

def getSignedDelay(sig_a , sig_b):
    corr = sig.correlate(sig_a , sig_b , mode="full")
    delays = np.arange(-len(sig_a) + 1 , len(sig_a))
    delay_sample = delays[np.argmax(np.abs(corr))]
    delay_time = delay_sample / cal_fs
    
    return delay_sample , delay_time

# main calculation function
def processFile(audio):
    read_data = audio
    read_fs = fs
    if read_fs != cal_fs:
        raise ValueError(f"Sample rate mismatch: except: {cal_fs} Hz, got {read_fs} Hz.")
    if read_data.shape[1] < 4:
        raise ValueError("Insufficient channels in the audio. At least 4 channels are required.")
    
    dataChannel = {
        1 : read_data[int(0.3 * cal_fs) : , 0],
        2 : read_data[int(0.3 * cal_fs) : , 1],
        3 : read_data[int(0.3 * cal_fs) : , 2],
        4 : read_data[int(0.3 * cal_fs) : , 3],
    }

    # bandpass filter design
    b , a = sig.ellip(3 , 3 , 50, [5000/(0.5*cal_fs), 25000/(0.5*cal_fs)], btype="bandpass")
    for c in dataChannel:
        dataChannel[c] = sig.filtfilt(b , a , dataChannel[c])

    # detect chirp arrival time
    arrivals_sample = {k: detectWhistle(v) for k , v in dataChannel.items()}
    sortedMic = sorted(arrivals_sample.items(), key=lambda x: x[1])
    arrival_order = [[f"Mic{m} ({s / cal_fs : .4f} s)" for m , s in sortedMic]]

    # get signal section
    peak = min(arrivals_sample.values())
    startIdx = max(0 , peak)
    endIdx = min(len(dataChannel[1]) , startIdx + (2 * cal_fs))

    for c in dataChannel:
        dataChannel[c] = dataChannel[c][startIdx:endIdx]

    # estimate x/y components directly from signed TDOA
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

    # average horziontal and vertical TDOA
    tau_x = (tau12 + tau34) / 2.0
    tau_y = (tau13 + tau24) / 2.0

    print(f"tau_x = {tau_x:.4f} s")
    print(f"tau_y = {tau_y:.4f} s")

    # convert TDOA to directional components
    ux = -(cal_soundspeed * tau_x) / d_h
    uy = -(cal_soundspeed * tau_y) / d_v

    ux = np.clip(ux , -1.0 , 1.0)
    uy = np.clip(uy , -1.0 , 1.0)

    print(f"ux = {ux:.4f}")
    print(f"uy = {uy:.4f}")

    mean_angle = (math.degrees(math.atan2(ux , uy)) + 360) % 360

    corrected_angles = {
        (1 , 2): math.degrees(math.asin(np.clip(-(cal_soundspeed * tau12) / d_h , -1.0 , 1.0))),
        (3 , 4): math.degrees(math.asin(np.clip(-(cal_soundspeed * tau34) / d_h , -1.0 , 1.0))),
        (1 , 3): math.degrees(math.asin(np.clip(-(cal_soundspeed * tau13) / d_v , -1.0 , 1.0))),
        (2 , 4): math.degrees(math.asin(np.clip(-(cal_soundspeed * tau24) / d_v , -1.0 , 1.0)))
    }

    return corrected_angles , startIdx / cal_fs , arrival_order , mean_angle

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
    buffer = []
    try:
        while not closeFlag:
            print("\n")
            print("=" * 40)
            timeNow = time.strftime("%Y%m%d%H%M%S" ,  time.localtime())
            audioFilename = timeNow + ".wav"

            localLat = 22.6319
            localLon = 120.2614

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

            print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Buffer: ", *buffer)

            if len(buffer) >= 10:
                msg_to_send = ",".join(f"{v:.2f}" for v in buffer)
                print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} message:{msg_to_send}")
                # st6100_send_msg.st6100_send_msg(msg_id=msgcount , msg = msg_to_send)
                print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Sending message from Pi to satellite.")
                msgcount += 1
                buffer = []

            else:
                print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Current buffer length: {len(buffer)} , wait buffer length = 10 to send message.")
            
            elasped_time = time.time() - start_time
            if elasped_time >= RUN_DURATION:
                closeFlag = True
                if buffer:
                    msg_to_send = ",".join(f"{float(v):.2f}" for v in buffer)
                    # st6100_send_msg.st6100_send_msg(msg_id=msgcount , msg = msg_to_send)
                    print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Reach max runtime. Sending angles from buffer.")
                    print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Sending message from Pi to satellite.")
                    msgcount += 1
                    buffer = []
            print("=" * 40)
            if msgcount > 707:
                msgcount = 700

    except KeyboardInterrupt:
        print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Interrupt by user.")
    except Exception as e:
        print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Fatal Error occured: {e}")
    finally:
        print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} DONE signal sent. Program will shutdown soon.")
        print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} Program exiting normally")
        print(f"[Pi] {datetime.now().strftime('%H:%M:%S')} ==== Finder System program ended ====")
        esp.close()


if __name__ == "__main__":
    main()
