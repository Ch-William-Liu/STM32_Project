import serial
import time
from datetime import datetime

PORT = "COM6"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=2)

# wait STM32 ready
time.sleep(2)

now = datetime.now()

cmd = f"SETTIME,{now.year},{now.month},{now.day},{now.hour},{now.minute},{now.second}\r\n"

print("Send:", cmd.strip())
ser.write(cmd.encode("ascii"))

time.sleep(3)

while ser.in_waiting:
    line = ser.readline().decode(errors="ignore").strip()
    print("STM32:", line)

ser.close()