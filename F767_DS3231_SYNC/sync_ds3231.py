from __future__ import annotations

import time
from datetime import datetime

import serial
import serial.tools.list_ports

BAUDRATE  = 115200

def show_ports()->list[serial.tools.list_ports.LisPortInfo]:
  ports = list(serial.tools.list_ports.comports())

  if not ports:
    print("No COM Port found.")
    return []
  
  print("Usable COM PORT: ")
  
  for index, port in enumerate(ports):
    print(f"[{index}] {port.device} - {port.description}")

  return ports

def main() -> None:
  ports = show_ports()

  if not ports:
    return
  
  try:
    selected_index = int(input("Enter COM Port number:"))
    selected_port = ports[selected_index].device

  except (ValueError, IndexError):
    print("Invalid Index.")
    return
  
  try:
    with serial.Serial(port=selected_port, baudrate=BAUDRATE, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,timeout=3) as serial_port:
      time.sleep(1)
      serial_port.reset_input_buffer()

      now = datetime.now()

      message = now.strftime("%Y,%m,%d,%H,%M,%S\r\n")

      print(f"PC time: {now:%Y-%m-%d %H:%M:%S.%f}")
      print(f"Tx Message:{message.strip()}")

      serial_port.write(message.encode("ascii"))

      serial_port.flush()

      response = serial_port.readline().decode("ascii" , errors="repalce").strip()

      if response:
        print(f"Response: {response}")
      else:
        print("NO Response")

  except serial.SerialException as error:
    print(f"Serial Error: {error}")

if __name__ == "__main__":
  main()