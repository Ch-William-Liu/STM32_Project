# !usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import csv
import re
from datetime import datetime

import serial
from PyQt5.QtWidgets import QApplication , QWidget , QVBoxLayout , QLabel
import pyqtgraph as pg
from PyQt5.QtCore import QTimer

PORT = "COM6"
BAUD = 115200
CSV_FILE = "sensor_log.csv"

color1 = "#FFFFFF"
color2 = "#EE2737"
color3 = "#005E88"
color4 = "#FF8DA1"

class SensorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 Sensor Monitor")

        self.ser = serial.Serial(PORT , BAUD , timeout = 0.05)

        self.index = []
        self.ax , self.ay , self.az = [] , [] , []
        self.gx , self.gy , self.gz = [] , [] , []
        self.temp_bmp180 = []
        self.temp_sbe = []

        self.current_mpu = None
        self.pending_sbe_time = None

        self.label = QLabel("Waiting for data...")
        
        self.plot_acc = pg.PlotWidget(title = "Fig.1 Acceleration")
        self.plot_gyro = pg.PlotWidget(title = "Fig.2 Gyroscope")
        self.plot_temp = pg.PlotWidget(title = "Fig.3 Temperature")

        self.plot_acc.addLegend()
        self.plot_gyro.addLegend()
        self.plot_temp.addLegend()

        self.curve_ax = self.plot_acc.plot(name = "ax" , pen = pg.mkPen(color1 , width=2))
        self.curve_ay = self.plot_acc.plot(name = "ay" , pen = pg.mkPen(color2 , width=2))
        self.curve_az = self.plot_acc.plot(name = "az" , pen = pg.mkPen(color3 , width=2))

        self.curve_gx = self.plot_gyro.plot(name = "gx" , pen = pg.mkPen(color1 , width=2))
        self.curve_gy = self.plot_gyro.plot(name = "gy" , pen = pg.mkPen(color2 , width=2))
        self.curve_gz = self.plot_gyro.plot(name = "gz" , pen = pg.mkPen(color3 , width=2))

        self.curve_temp_bmp180 = self.plot_temp.plot(name = "temp_BMP180" , pen = pg.mkPen(color1 , width=2))
        self.curve_temp_sbe39 = self.plot_temp.plot(name = "temp_SBE39" , pen = pg.mkPen(color4 , width=2))

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.plot_acc)
        layout.addWidget(self.plot_gyro)
        layout.addWidget(self.plot_temp)
        self.setLayout(layout)

        self.csv_file = open(CSV_FILE , "a" , newline="" , encoding="utf-8")
        self.writer = csv.writer(self.csv_file)

        self.writer.writerow(
            [
                "pc_time",
                "index",
                "stm32_time_ms",
                "ax" , "ay" , "az",
                "gx" , "gy" , "gz",
                "temp_BMP180" , "temp_SBE39",
                "SBE_pressure" , "SBE_raw" 
            ]
        )

        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial)
        self.timer.start(50)

        self.waiting_sbe_value = False

    def read_serial(self):
        try:
            line = self.ser.readline().decode(errors="ignore").strip()
        except Exception:
            return

        if not line:
            return

        self.label.setText(line)

        if line.startswith("MPU,"):
            self.parse_mpu(line)

        elif line.startswith("SBE,"):
            self.waiting_sbe_value = True

        elif self.waiting_sbe_value:
            if "<Executed" in line:
                return

            nums = re.findall(r"[-+]?\d+\.\d+", line)

            if len(nums) >= 2:
                temp_sbe = float(nums[0])
                pressure_sbe = float(nums[1])

                if len(self.temp_sbe) > 0:
                    self.temp_sbe[-1] = temp_sbe

                self.update_plots()

                if self.current_mpu is not None:
                    self.writer.writerow([
                        datetime.now().isoformat(),
                        self.current_mpu["idx"],
                        self.current_mpu["t_ms"],
                        self.current_mpu["ax"],
                        self.current_mpu["ay"],
                        self.current_mpu["az"],
                        self.current_mpu["gx"],
                        self.current_mpu["gy"],
                        self.current_mpu["gz"],
                        self.current_mpu["temp_bmp"],
                        temp_sbe,
                        pressure_sbe,
                        line
                    ])
                    self.csv_file.flush()

                self.waiting_sbe_value = False

    def parse_mpu(self , line):
        parts = line.split(",")

        if len(parts) < 9:
            return
        
        try:
            t_ms = int(parts[1])
            ax = float(parts[2])
            ay = float(parts[3])
            az = float(parts[4])
            gx = float(parts[5])
            gy = float(parts[6])
            gz = float(parts[7])
            temp_bmp = float(parts[8])
        except ValueError:
            return
        
        idx = len(self.index)

        self.index.append(idx)

        self.ax.append(ax)
        self.ay.append(ay)
        self.az.append(az)

        self.gx.append(gx)
        self.gy.append(gy)
        self.gz.append(gz)

        self.temp_bmp180.append(temp_bmp)
        self.temp_sbe.append(float("nan"))

        self.current_mpu = {
            "idx": idx,
            "t_ms": t_ms,
            "ax": ax,
            "ay": ay,
            "az": az,
            "gx": gx,
            "gy": gy,
            "gz": gz,
            "temp_bmp": temp_bmp
        }

        self.update_plots()

    def update_plots(self):
        self.curve_ax.setData(self.index , self.ax)
        self.curve_ay.setData(self.index , self.ay)
        self.curve_az.setData(self.index , self.az)

        self.curve_gx.setData(self.index , self.gx)
        self.curve_gy.setData(self.index , self.gy)
        self.curve_gz.setData(self.index , self.gz)

        self.curve_temp_bmp180.setData(self.index , self.temp_bmp180)
        self.curve_temp_sbe39.setData(self.index , self.temp_sbe)

    def closeEvent(self , event):
        self.ser.close()
        self.csv_file.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = SensorGUI()
    win.resize(1000 , 800)
    win.show()
    sys.exit(app.exec_())

