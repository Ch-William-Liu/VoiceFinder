import serial
import st6100_send_msg
import read_ESP32

serST6100 = serial.Serial(port = "/dev/ttyUSB0" , baudrate = 9600 , timeout = 1)

GPSinfo = st6100_send_msg.get_gps_info(ser = serST6100 , retries = 20 , wait_time = 120 , stale_secs = 60)

print(GPSinfo , sep = ",")
