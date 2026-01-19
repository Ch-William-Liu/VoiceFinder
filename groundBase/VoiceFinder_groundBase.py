# !/usr/bin/env python3
# -*- utf-8 -*-

'''
Created on Thu Jan 16 10:39:46 2026

@author: William

Excepted line format:
    OK,<server_time>,<lat_dmm>,<lon_dmm>,<cal_angle>,<sensor_angle>,<true_angle>

Behavior:
- Each worker thread polls the server with AT+GRMG
- If any thread receives the line "NULL" , it triggers a global stop and the program exits cleanly.
'''

# Rewritten from the VoiceSeeker ground base app; for VoiceFinder only.

import os
import json
import time
import socket
import threading
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Dict, Any, List

# ====================================
# Basic Setting
# ====================================
HOST = "www.skybees.net"
PORT = 9779

POLL_INTERVAL_SEC = 10.0            # wait before sending the next AT command
RECV_TIMEOUT = 5.0                  # socket recv() timeout (seconds)
CONNECT_TIMEOUT = 10.0              # socket connect timeout (seconds)
BASE_BACKOFF = 1.0                  # initial backoff after an error (seconds)
MAX_BACKOFF = 60.0                  # maximum backoff (seconds)

# Output directory (default to current working directory)
ANGLES_DIR = os.path.join(os.getcwd() , "angle_out")
os.makedirs(ANGLES_DIR , exist_ok = True)

# ====================================
# Credentials
# ====================================
@dataclass
class Credential:
    user : str
    passwd : str
    type_rec : int  # 1: Text, 2: ASC_HEX, 3: Base64

DEVICE_ID_LIST : List[Credential] = [
    Credential("echoing_ocean_05" , "echoing_ocean_05" , 1)
]

# ====================================
# Safe printing
# ====================================
_print_lock = threading.Lock()
def safe_print(*args , **kwargs):
    '''Thread-safe print to message from multiple workers do not interleave.'''
    with _print_lock:
        print(*args , **kwargs)

# ====================================
# File helpers
# ====================================
def path_raw(user: str) -> str:
    """NDJSON file for raw records (append-only)."""
    return os.path.join(ANGLES_DIR, f"{user}_raw.ndjson")

def path_angles_ndjson(user: str) -> str:
    """NDJSON file for parsed angle records (append-only)."""
    return os.path.join(ANGLES_DIR, f"{user}_angles.ndjson")

def append_ndjson(filepath: str, data: Dict[str, Any]) -> None:
    """Append a single JSON line (NDJSON) to a file."""
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with open(filepath, "a", encoding="utf-8") as f:
        f.write(json.dumps(data, ensure_ascii=False) + "\n")

# ====================================
# Parsing helpers
# ====================================
def gga_to_decimal(coord : float) -> float:
    '''
    Convert ddmm.mmmm / ddmm.mmmm to decimal degrees.
    Preserves sign: negative => south/west.
    '''
    try:
        deg = int(coord / 100)
        minute = coord - deg * 100
        return deg + minute / 60
    except Exception:
        return coord
    
def split_recv_lines(data_bytes : bytes) -> List[str]:
    '''Normalize CR/LF and split the received bytes into non-empty lines.'''
    s = data_bytes.decode("utf-8" , errors = "ignore")
    s = s.replace("\r\n" , "\n").replace("\r" , "\n")
    return [ln.strip() for ln in s.split("\n") if ln.strip()]

def parse_ok_angles_line(line: str) -> Optional[Dict[str, Any]]:
    """
    Parse lines like:
      OK,<server_time>,<lat_dmm>,<lon_dmm>,<angle_1>,...,<angle_10>
    Returns a dict with an 'angles' list (variable length, empties ignored),
    or None if the line is not parseable.
    """
    if not line.startswith("OK,"):
        return None

    parts = [p.strip() for p in line.split(",")]
    if len(parts) < 5:
        return None

    # first four tokens must exist
    _, server_time, lat_s, lon_s, *rest = parts
    try:
        lat_dec = gga_to_decimal(float(lat_s))
        lon_dec = gga_to_decimal(float(lon_s))
    except ValueError:
        return None

    # parse angles: ignore empty tokens (handles ",,")
    angles: List[float] = []
    for tok in rest:
        if tok == "":
            continue
        try:
            angles.append(float(tok))
        except ValueError:
            # ignore non-numeric tail if any
            pass

    # keep at most 10 angles (as per your format), but allow fewer
    if len(angles) == 0:
        return None
    angles = angles[:10]

    return {
        "category": "angles",
        "server_time": server_time,
        "lat": lat_dec,
        "lon": lon_dec,
        "angles": angles,  # e.g., [angle_1, angle_2, ...]
        "received_at": datetime.utcnow().isoformat() + "Z",
        "raw": line,
    }
# ====================================
# Work thread
# ====================================
def recv_msg_task(device: Credential, stop_event: threading.Event):
    """
    Per-device worker:
    - Connects to the server.
    - Periodically sends AT+GRMG=<user>,<passwd>,<type>.
    - Parses responses and writes to files (NDJSON append-only).
    - When the line 'NULL' is received, DO NOT exit; just print a tip.
    """
    backoff = BASE_BACKOFF
    while not stop_event.is_set():
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(CONNECT_TIMEOUT)
            sock.connect((HOST, PORT))
            sock.settimeout(RECV_TIMEOUT)
            safe_print(f"[{device.user}] connected to {HOST}:{PORT}")
            backoff = BASE_BACKOFF

            while not stop_event.is_set():
                cmd = f"AT+GRMG={device.user},{device.passwd},{device.type_rec}\r"
                sock.sendall(cmd.encode("utf-8"))

                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    safe_print(f"[{device.user}] recv timeout.")
                    data = b""
                except socket.error as e:
                    safe_print(f"[{device.user}] recv error: {e}")
                    raise

                if not data:
                    time.sleep(POLL_INTERVAL_SEC)
                    continue

                for line in split_recv_lines(data):
                    if line == "NULL":
                        safe_print(f"[{device.user}] end of batch (NULL). "
                                   f"Tip: press Ctrl+C to stop the program.")
                        # do NOT stop; just finish this batch
                        continue

                    if line.startswith("ER"):
                        safe_print(f"[{device.user}] server error: {line}")
                        continue

                    rec = parse_ok_angles_line(line)
                    if rec:
                        # append to raw & angles NDJSON (stacking)
                        append_ndjson(path_raw(device.user), rec)
                        append_ndjson(path_angles_ndjson(device.user), rec)
                        safe_print(f"[{device.user}] saved angles: {rec['angles']}")
                    else:
                        safe_print(f"[{device.user}] skip: {line}")

                time.sleep(POLL_INTERVAL_SEC)

        except Exception as e:
            safe_print(f"[{device.user}] error: {e} (backoff {backoff}s)")
            time.sleep(backoff)
            backoff = min(backoff * 2, MAX_BACKOFF)
        finally:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
    safe_print(f"[{device.user}] worker stopped.")
# ====================================
# main
# ====================================
def main():
    stop_event = threading.Event()
    threads: List[threading.Thread] = []
    for dev in DEVICE_ID_LIST:
        t = threading.Thread(target=recv_msg_task, args=(dev, stop_event), daemon=True)
        t.start()
        threads.append(t)

    safe_print("Angle receivers started. Press Ctrl+C to stop.")
    try:
        while not stop_event.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        safe_print("Stopping...")
        stop_event.set()

    for t in threads:
        t.join(timeout=5.0)
    safe_print("Exited.")

if __name__ == "__main__":
    main()