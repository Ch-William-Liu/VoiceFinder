import serial
import st6100_send_msg
import read_ESP32

# serST6100 = serial.Serial(port = "/dev/ttyUSB0" , baudrate = 9600 , timeout = 1)

# GPSinfo = st6100_send_msg.get_gps_info(ser = serST6100 , retries = 20 , wait_time = 120 , stale_secs = 60)

# print(GPSinfo , sep = ",")

serESP32 = serial.Serial(port= "/dev/serial0" , baudrate = 115200 , timeout = 1)

esp = read_ESP32.ESPHeadingReader(ser = serESP32)

while True:
    print(esp.get_mean(10))