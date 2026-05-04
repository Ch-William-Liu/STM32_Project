# !/usr/bin/env python3
# -*- utf-8 -*-

import sys
import csv
import time
from collections import deque

import numpy as np
import serial
import serial.tools.list_ports

from PySide6 import QtCore , QtWidgets
from PySide6.QtGui import QTextCursor
import pyqtgraph as pg

SYNC1 = 0xAA
SYNC2 = 0x55

FS = 64000
FFT_SIZE = 4096
FFT_BINS = FFT_SIZE // 2
TREND_LEN = 200
MA_WINDOW = 10

class SerialReader(QtCore.QThread):
    packet_received = QtCore.Signal(int , object , str , bool , float , int)
    status_msg = QtCore.Signal(str)

    def __init__(self, port , baudrate):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.ser = None

    def stop(self):
        self.running = False

    def read_exact(self , n):
        data = b''
        while self.running and len(data) < n:
            chunk = self.ser.read(n - len(data))
            if not chunk:
                return None
            data += chunk
        return data
    
    def find_sync(self):
        while self.running:
            b = self.ser.read(1)
            if not b:
                continue
            if b[0] == SYNC1:
                b2 = self.ser.read(2)
                if not b2:
                    continue
                if b2[0] == SYNC2:
                    return True
        return False
    
    def run(self):
        buf = bytearray()

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.running = True
            self.status_msg.emit(f"Connected: {self.port} @ {self.baudrate}")

            while self.running:
                data = self.ser.read(4096)
                if not data:
                    continue

                buf.extend(data)

                while True:
                    if len(buf) < 6:
                        break

                    if buf[0] != 0xAA or buf[1] != 0x55:
                        buf.pop(0)
                        continue

                    msg_id = int.from_bytes(buf[2:4], 'little')
                    fft_len = int.from_bytes(buf[4:6], 'little')

                    if fft_len != FFT_BINS:
                        buf.pop(0)
                        continue

                    packet_len = 2 + 2 + 2 + fft_len * 2 + 2

                    if len(buf) < packet_len:
                        break

                    payload = bytes(buf[6:6 + fft_len * 2])
                    crc_bytes = bytes(buf[6 + fft_len * 2:6 + fft_len * 2 + 2])

                    values = np.frombuffer(payload, dtype=np.uint16).copy()

                    recv_crc = int.from_bytes(crc_bytes, 'little')
                    calc_crc = int(np.sum(values) & 0xFFFF)
                    crc_ok = (recv_crc == calc_crc)

                    raw_hex = " ".join(f"{b:02X}" for b in buf[:min(packet_len, 64)])
                    if packet_len > 64:
                        raw_hex += " ..."

                    peak_bin = int(np.argmax(values))
                    peak_freq = peak_bin * FS / FFT_SIZE
                    peak_mag = int(np.max(values))

                    self.packet_received.emit(
                        msg_id,
                        values,
                        raw_hex,
                        crc_ok,
                        peak_freq,
                        peak_mag
                    )

                    del buf[:packet_len]

        except Exception as e:
            self.status_msg.emit(f"Serial error: {e}")

        finally:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
            self.status_msg.emit("Disconnected")

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 FFT UART Monitor")
        self.resize(1400 , 900)

        self.reader = None
        self.csv_file = None
        self.csv_writer = None
        self.logging_enabled = False

        self.frame_history = deque(maxlen=TREND_LEN)
        self.peak_freq_history = deque(maxlen=TREND_LEN)
        self.peak_mag_history = deque(maxlen=TREND_LEN)

        self.freq_axis =  np.arange(FFT_BINS) * FS / FFT_SIZE

        self.build_ui()
        self.refresh_ports()

    def build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        main_layout = QtWidgets.QVBoxLayout(central)

        top_layout = QtWidgets.QHBoxLayout()
        self.port_combo = QtWidgets.QComboBox()
        self.refresh_btn = QtWidgets.QPushButton("Refresh Ports")
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems(["115200" , "230400" , "460800" , "921600" , "1152000"])
        self.baud_combo.setCurrentText("921600")
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.disconnect_btn = QtWidgets.QPushButton("Disconnect")
        self.start_log_btn = QtWidgets.QPushButton("Start Log")
        self.stop_log_btn = QtWidgets.QPushButton("Stop Log")
        self.clear_btn = QtWidgets.QPushButton("Clear Log")

        top_layout.addWidget(QtWidgets.QLabel("Port"))
        top_layout.addWidget(self.port_combo)
        top_layout.addWidget(self.refresh_btn)
        top_layout.addSpacing(20)
        top_layout.addWidget(QtWidgets.QLabel("Baud"))
        top_layout.addWidget(self.baud_combo)
        top_layout.addSpacing(20)
        top_layout.addWidget(self.connect_btn)
        top_layout.addWidget(self.disconnect_btn)
        top_layout.addSpacing(20)
        top_layout.addWidget(self.start_log_btn)
        top_layout.addWidget(self.stop_log_btn)
        top_layout.addWidget(self.clear_btn)
        top_layout.addStretch()

        main_layout.addLayout(top_layout)

        status_layout = QtWidgets.QHBoxLayout()
        self.status_label = QtWidgets.QLabel("Status: Dissconnected")
        self.frame_label =  QtWidgets.QLabel("Frame: -")
        self.crc_label = QtWidgets.QLabel("CRC: -")
        self.peak_freq_label = QtWidgets.QLabel("Peak Freq: -")
        self.peak_mag_label = QtWidgets.QLabel("Peak Mag: -")

        status_layout.addWidget(self.status_label)
        status_layout.addSpacing(20)
        status_layout.addWidget(self.frame_label)
        status_layout.addSpacing(20)
        status_layout.addWidget(self.crc_label)
        status_layout.addSpacing(20)
        status_layout.addWidget(self.peak_freq_label)
        status_layout.addSpacing(20)
        status_layout.addWidget(self.peak_mag_label)
        status_layout.addStretch()

        main_layout.addLayout(status_layout)

        pg.setConfigOptions(antialias = True)

        self.fft_plot = pg.PlotWidget(title = "FFT Magnitude")
        self.fft_plot.setLabel('left' , 'Magnitude')
        self.fft_plot.setLabel('bottom' , 'Frequency' , units='Hz')
        self.fft_curve = self.fft_plot.plot(pen = 'y')

        self.freq_trend_plot = pg.PlotWidget(title = "Peak Frequency Trend")
        self.freq_trend_plot.setLabel('left' , 'Peak Frequency' , units='Hz')
        self.freq_trend_plot.setLabel('bottom' , 'Frame')
        self.freq_trend_curve = self.freq_trend_plot.plot(pen = 'c')
        self.freq_ma_curve = self.freq_trend_plot.plot(pen=pg.mkPen(width=3))

        self.mag_trend_plot = pg.PlotWidget(title = "Peak Magnitude Trend")
        self.mag_trend_plot.setLabel('left' , 'Peak Magnitude')
        self.mag_trend_plot.setLabel('bottom' , 'Frame')
        self.mag_trend_curve = self.mag_trend_plot.plot(pen='m')
        self.mag_ma_curve = self.mag_trend_plot.plot(pen=pg.mkPen(width=3))

        main_layout.addWidget(self.fft_plot , stretch=3)
        main_layout.addWidget(self.freq_trend_plot , stretch=2)
        main_layout.addWidget(self.mag_trend_plot , stretch=2)

        bottom_splitter = QtWidgets.QSplitter()
        bottom_splitter.setOrientation(QtCore.Qt.Horizontal)

        self.parsed_text = QtWidgets.QPlainTextEdit()
        self.parsed_text.setReadOnly(True)
        self.parsed_text.setPlaceholderText("Parsed UART summary")

        self.raw_text = QtWidgets.QPlainTextEdit()
        self.raw_text.setReadOnly(True)
        self.raw_text.setPlaceholderText("UART raw hex")

        bottom_splitter.addWidget(self.parsed_text)
        bottom_splitter.addWidget(self.raw_text)
        bottom_splitter.setSizes([700 , 700])

        main_layout.addWidget(bottom_splitter , stretch=2)

        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.connect_serial)
        self.disconnect_btn.clicked.connect(self.disconnect_serial)
        self.start_log_btn.clicked.connect(self.start_logging)
        self.stop_log_btn.clicked.connect(self.stop_logging)
        self.clear_btn.clicked.connect(self.clear_logs)

    def refresh_ports(self):
        self.port_combo.clear()
        for p in serial.tools.list_ports.comports():
            self.port_combo.addItem(p.device)

    def connect_serial(self):
        if self.reader is not None and self.reader.isRunning():
            return
        
        port = self.port_combo.currentText()
        if not port:
            self.status_label.setText("Status: No COM port selected")
            return
        
        baud = int(self.baud_combo.currentText())
        self.reader = SerialReader(port , baud)
        self.reader.packet_received.connect(self.handle_packet)
        self.reader.status_msg.connect(self.handle_status)
        self.reader.start()

    def disconnect_serial(self):
        if self.reader is not None:
            self.reader.stop()
            self.reader.wait()
            self.reader = None

    def handle_status(self , msg):
        self.status_label.setText(f"Status: {msg}")
        self.append_text(self.parsed_text , msg , limit = 200)
    
    def start_logging(self):
        if self.logging_enabled:
            return
        
        filename = time.strftime("fft_log_%Y%m%d_%H%M%S.csv")
        self.csv_file = open(filename , "w" , newline="" , encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        header = ["timestamp","msg_id","peak_freq_hz","peak_mag"] + [f"bin_{i}" for i in range(FFT_BINS)]
        self.csv_writer.writerow(header)
        self.csv_file.flush()
        self.logging_enabled = True
        self.handle_status(f"Logging started: {filename}")

    def stop_logging(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        self.logging_enabled = False
        self.handle_status("Logging stopped")

    def clear_logs(self):
        self.parsed_text.clear()
        self.raw_text.clear()
        self.frame_history.clear()
        self.peak_freq_history.clear()
        self.peak_mag_history.clear()

        self.fft_curve.setData([] , [])
        self.freq_trend_curve.setData([] , [])
        self.freq_ma_curve.setData([] , [])
        self.mag_trend_curve.setData([] , [])
        self.mag_ma_curve.setData([] , [])

        self.frame_label.setText("Frame: -")
        self.crc_label.setText("CRC: -")
        self.peak_freq_label.setText("Peak Freq: -")
        self.peak_mag_label.setText("Pead Mag: -")

    def moving_average(self , data , window):
        data = np.array(data , dtype=float)
        if len(data) == 0:
            return np.array([])
        if len(data) < window:
            return data.copy()
        
        kernel = np.ones(window) / window
        ma = np.convolve(data , kernel , mode='valid')
        pad = np.full(window - 1, np.nan)
        return np.concatenate([pad , ma])
    
    def handle_packet(self , msg_id , values , raw_hex , crc_ok , peak_freq , peak_mag):
        print("=" * 60)
        print(f"msg_id={msg_id}, len(values)={len(values)}, expected={FFT_BINS}, crc_ok={crc_ok}")
        print("First 16 values:", values[:16])

        self.frame_label.setText(f"Frame: {msg_id}")
        self.crc_label.setText(f"CRC: {'OK' if crc_ok else 'ERROR'}")
        self.peak_freq_label.setText(f"Peak Freq: {peak_freq:.2f} Hz")
        self.peak_mag_label.setText(f"Peak Mag: {peak_mag}")

        n = min(len(self.freq_axis), len(values))
        self.fft_curve.setData(self.freq_axis[:n], values[:n])

        self.frame_history.append(msg_id)
        self.peak_freq_history.append(peak_freq)
        self.peak_mag_history.append(peak_mag)

        frames = np.array(self.frame_history)
        peak_freqs = np.array(self.peak_freq_history)
        peak_mags = np.array(self.peak_mag_history)

        freq_ma = self.moving_average(peak_freqs , MA_WINDOW)
        mag_ma = self.moving_average(peak_mags , MA_WINDOW)

        self.freq_trend_curve.setData(frames , peak_freqs)
        self.freq_ma_curve.setData(frames , freq_ma)
        self.mag_trend_curve.setData(frames , peak_mags)
        self.mag_ma_curve.setData(frames , mag_ma)

        summary = (
            f"Frame {msg_id} | CRC {'OK' if crc_ok else 'ERROR'} | "
            f"Peak Freq {peak_freq:.2f} Hz | Peak Mag {peak_mag}"
        )

        self.append_text(self.parsed_text , summary , limit = 200)
        self.append_text(self.raw_text , raw_hex , limit = 200)

        if self.logging_enabled and self.csv_writer is not None:
            row = [time.time() , msg_id , peak_freq , peak_mag] + values.tolist()
            self.csv_writer.writerow(row)
            self.csv_file.flush()
        print(f"msg_id={msg_id}, len(values)={len(values)}, expected={len(self.freq_axis)}, crc_ok={crc_ok}")
        if len(values) != len(self.freq_axis):
            print("Length mismatch!")
            print("First 16 values:", values[:16])

    def append_text(self , widget , text , limit=200):
        widget.appendPlainText(text)
        lines = widget.toPlainText().splitlines()
        if len(lines) > limit:
            widget.setPlainText("\n".join(lines[-limit:]))
            cursor = widget.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.End)
            widget.setTextCursor(cursor)

    def closeEvent(self, event):
        self.disconnect_serial()
        self.stop_logging()
        super().closeEvent(event)

def main():
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()