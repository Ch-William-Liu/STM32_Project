# !usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import csv
import time
from collections import deque

import numpy as np
import serial
import serial.tools.list_ports

from PySide6 import QtCore, QtWidgets
from PySide6.QtGui import QTextCursor
import pyqtgraph as pg

SYNC1 = 0xAA
SYNC2 = 0x55

DEFAULT_FS = 64000
DEFAULT_FFT_SIZE = 4096
DEFAULT_FFT_BINS = DEFAULT_FFT_SIZE // 2

TREND_LEN = 200
SPECGRAM_LEN = 200
MA_WINDOW = 10

class SerialReader(QtCore.QThread):
    packet_received = QtCore.Signal(int, object, str, bool, float, float, int, int)
    status_msg = QtCore.Signal(str)

    def __init__(self, port, baudrate):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.ser = None

    def stop(self):
        self.running = False

    def send_command(self, cmd):
        if self.ser is not None and self.ser.is_open:
            if not cmd.endswith("\n"):
                cmd+="\n"
            self.ser.write(cmd.encode("ascii"))

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
                    sync_pos = buf.find(bytes[SYNC1 , SYNC2])

                    if sync_pos < 0:
                        if len(buf) > 8192:
                            buf.clear()
                        self.status_msg.emit(f"RX {len(data)} bytes, sync_pos = -1")
                        break

                    if sync_pos > 0:
                        del buf[:sync_pos]

                    if len(buf) < 18:
                        break

                    frame_index = int.from_bytes(buf[2:6],"little")
                    fs = int.from_bytes(buf[6:10],"little")
                    fft_size = int.from_bytes(buf[10:14],"little")
                    bin_count = int.from_bytes(buf[14:18],"little")

                    if fs not in [16000, 32000, 64000, 100000]:
                        del buf[0]
                        continue

                    if fft_size not in [1024, 2048, 4096]:
                        del buf[0]
                        continue

                    if bin_count <= 0 or bin_count > 4096:
                        del buf[0]
                        continue

                    packet_len = 18 + bin_count * 2 + 2

                    if len(buf) < packet_len:
                        break

                    if buf[packet_len - 2] != 0x55 or buf[packet_len - 1] != 0xAA:
                        del buf[0]
                        continue

                    payload = bytes(buf[18 : 18 + bin_count * 2])
                    values = np.frombuffer(payload , dtype="<u2").copy()

                    raw_hex = " ".join(f"{b:02X}" for b in buf[:min(packet_len , 64)])
                    if packet_len > 64:
                        raw_hex += " ..."

                    del buf[:packet_len]

                    freq_axis = np.arange(bin_count) * fs / fft_size
                    mag_db = 20.0 * np.log10(values.astype(float) + 1e-12)

                    peak_bin = int(np.argmax(mag_db))
                    peak_freq = float(freq_axis[peak_bin])
                    peak_mag = float(mag_db[peak_bin])

                    self.packet_received.emit(
                        frame_index,
                        values,
                        raw_hex,
                        True,
                        peak_freq,
                        peak_mag,
                        fs,
                        fft_size
                    )


        except Exception as e:
            self.status_msg.emit(f"Serial error: {e}")

        finally:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
            self.status_msg.emit("Disconnected")

class MainConfig:
    current_fs = DEFAULT_FS
    current_fft_size = DEFAULT_FFT_SIZE

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("STM32F767 FFT Monitor and WAV Recorder")
        self.resize(1500, 900)

        self.reader = None
        self.csv_file = None
        self.csv_writer = None
        self.logging_enabled = False

        self.frame_history = deque(maxlen=TREND_LEN)
        self.peak_freq_history = deque(maxlen=TREND_LEN)
        self.peak_mag_history = deque(maxlen=TREND_LEN)

        self.specgram = None
        self.latest_values = None
        self.latest_mag_db = None
        self.freq_axis = np.arange(DEFAULT_FFT_BINS) * DEFAULT_FS / DEFAULT_FFT_SIZE

        self.build_ui()
        self.refresh_ports()

    def build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        main_layout = QtWidgets.QVBoxLayout(central)

        # Top control pannel
        top_layout = QtWidgets.QHBoxLayout()

        self.port_combo = QtWidgets.QComboBox()
        self.refresh_btn = QtWidgets.QPushButton("Refresh Ports")

        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems(["115200", "230400", "460800", "921600", "1152000"])
        self.baud_combo.setCurrentText("1152000")

        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.disconnect_btn = QtWidgets.QPushButton("Disconnect")

        self.start_log_btn = QtWidgets.QPushButton("Start Log")
        self.stop_log_btn = QtWidgets.QPushButton("Stop Log")
        self.clear_log_btn = QtWidgets.QPushButton("Clear Log")

        top_layout.addWidget(QtWidgets.QLabel("Port"))
        top_layout.addWidget(self.port_combo)
        top_layout.addWidget(self.refresh_btn)
        top_layout.addSpacing(20)

        top_layout.addWidget(QtWidgets.QLabel("Baud"))
        top_layout.addWidget(self.baud_combo)
        top_layout.addWidget(self.refresh_btn)
        top_layout.addSpacing(20)

        top_layout.addWidget(self.connect_btn)
        top_layout.addWidget(self.disconnect_btn)
        top_layout.addSpacing(20)

        top_layout.addWidget(self.start_log_btn)
        top_layout.addWidget(self.stop_log_btn)
        top_layout.addWidget(self.clear_log_btn)
        top_layout.addStretch()

        main_layout.addLayout(top_layout)

        # STM32 setting panel
        setting_layout = QtWidgets.QHBoxLayout()

        self.fs_combo = QtWidgets.QComboBox()
        self.fs_combo.addItems(["16000" , "32000" , "64000" , "100000"])
        self.fs_combo.setCurrentText(str(DEFAULT_FS))

        self.fft_combo = QtWidgets.QComboBox()
        self.fft_combo.addItems(["1024" , "2048" , "4096"])
        self.fft_combo.setCurrentText(str(DEFAULT_FFT_SIZE))

        self.apply_setting_btn = QtWidgets.QPushButton("Apply Sampling / FFT Setting")

        self.rec_duration_edit = QtWidgets.QLineEdit("30")
        self.rec_duration_edit.setFixedWidth(80)

        self.start_rec_btn = QtWidgets.QPushButton("Start Recording to SD card")
        self.stop_rec_btn = QtWidgets.QPushButton("Stop Recording")

        setting_layout.addWidget(QtWidgets.QLabel("Sample Rate"))
        setting_layout.addWidget(self.fs_combo)
        setting_layout.addSpacing(10)

        setting_layout.addWidget(QtWidgets.QLabel("FFT Size"))
        setting_layout.addWidget(self.fft_combo)
        setting_layout.addWidget(self.apply_setting_btn)
        setting_layout.addSpacing(30)

        setting_layout.addWidget(QtWidgets.QLabel("Record Duration (s)"))
        setting_layout.addWidget(self.rec_duration_edit)
        setting_layout.addWidget(self.start_rec_btn)
        setting_layout.addWidget(self.stop_rec_btn)
        setting_layout.addStretch()

        main_layout.addLayout(setting_layout)

        # Status Panel
        status_layout = QtWidgets.QHBoxLayout()

        self.status_label = QtWidgets.QLabel("Status: Disconnected")
        self.frame_label = QtWidgets.QLabel("Frame : -")
        self.crc_label = QtWidgets.QLabel("CRC: -")
        self.peak_freq_label = QtWidgets.QLabel("Peak Freq: -")
        self.peak_mag_label = QtWidgets.QLabel("Peak Mag: -")
        self.setting_label = QtWidgets.QLabel(f"fs: {DEFAULT_FS}, Hz | FFT_SIZE: {DEFAULT_FFT_SIZE}")

        status_layout.addWidget(self.status_label)
        status_layout.addSpacing(20)
        status_layout.addWidget(self.frame_label)
        status_layout.addSpacing(20)
        status_layout.addWidget(self.crc_label)
        status_layout.addSpacing(20)
        status_layout.addWidget(self.peak_freq_label)
        status_layout.addSpacing(20)
        status_layout.addWidget(self.peak_mag_label)
        status_layout.addSpacing(20)
        status_layout.addWidget(self.setting_label)
        status_layout.addStretch()

        main_layout.addLayout(status_layout)

        # Tabs
        pg.setConfigOptions(antialias = True)

        self.tabs = QtWidgets.QTabWidget()
        main_layout.addWidget(self.tabs, stretch=7)

        self.peak_tab = QtWidgets.QWidget()
        self.specgram_tab = QtWidgets.QWidget()
        self.trend_tab = QtWidgets.QWidget()
        self.log_tab = QtWidgets.QWidget()

        self.tabs.addTab(self.peak_tab , "Peak Analysis")
        self.tabs.addTab(self.specgram_tab , "Real-time Spectrogram")
        self.tabs.addTab(self.trend_tab , "Peak Trend")
        self.tabs.addTab(self.log_tab , "Log / Raw Data")

        self.build_peak_tab()
        self.build_specgram_tab()
        self.build_trend_tab()
        self.build_log_tab()

        # signals
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.connect_serial)
        self.disconnect_btn.clicked.connect(self.disconnect_serial)
        
        self.start_log_btn.clicked.connect(self.start_logging)
        self.stop_log_btn.clicked.connect(self.stop_logging)
        self.clear_log_btn.clicked.connect(self.clear_logs)

        self.apply_setting_btn.clicked.connect(self.apply_setting)
        self.start_rec_btn.clicked.connect(self.start_recording)
        self.stop_rec_btn.clicked.connect(self.stop_recording)

    def build_peak_tab(self):
        layout = QtWidgets.QVBoxLayout(self.peak_tab)

        self.fft_plot = pg.PlotWidget(title = "FFT Magnitude")
        self.fft_plot.setLabel("left" , "Frequency", units="Hz")
        self.fft_plot.setLabel("bottom" , "Magnidude", units="dB")
        self.fft_curve = self.fft_plot.plot(pen="y")

        layout.addWidget(self.fft_plot)

    def build_specgram_tab(self):
        layout = QtWidgets.QVBoxLayout(self.specgram_tab)

        self.specgram_plot = pg.PlotWidget(title="Real-time Spectrogram")
        self.specgram_plot.setLabel("bottom", "Frame")
        self.specgram_plot.setLabel("left" , "Frequency" , units="Hz")

        self.specgram_img = pg.ImageItem()
        self.specgram_plot.addItem(self.specgram_img)

        self.colorbar = pg.ColorBarItem(values=(-120, 80), colorMap=pg.colormap.get("viridis"))
        self.colorbar.setImageItem(self.specgram_img)

        layout.addWidget(self.specgram_plot)

    def build_trend_tab(self):
        layout = QtWidgets.QVBoxLayout(self.trend_tab)

        self.freq_trend_plot = pg.PlotWidget(title="Peak Frequency Trend")
        self.freq_trend_plot.setLabel("left","Peak Frequency",units="Hz")
        self.freq_trend_plot.setLabel("bottom","Frame")
        self.freq_trend_curve = self.freq_trend_plot.plot(pen="c")
        self.freq_ma_curve = self.freq_trend_plot.plot(pen=pg.mkPen(width=3))

        self.mag_trend_plot = pg.PlotWidget(title="Peak Magnitude Trend")
        self.mag_trend_plot.setLabel("left", "Peak Magnitude", units="dB")
        self.mag_trend_plot.setLabel("bottom", "Frame")
        self.mag_trend_curve = self.mag_trend_plot.plot(pen="m")
        self.mag_ma_curve = self.mag_trend_plot.plot(pen=pg.mkPen(width=3))

        layout.addWidget(self.freq_trend_plot)
        layout.addWidget(self.mag_trend_plot)

    def build_log_tab(self):
        layout = QtWidgets.QHBoxLayout(self.log_tab)

        self.parsed_text = QtWidgets.QPlainTextEdit()
        self.parsed_text.setReadOnly(True)
        self.parsed_text.setPlaceholderText("Parsed UART summary")

        self.raw_text = QtWidgets.QPlainTextEdit()
        self.raw_text.setReadOnly(True)
        self.raw_text.setPlaceholderText("UART raw hex")

        layout.addWidget(self.parsed_text)
        layout.addWidget(self.raw_text)

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

        self.reader = SerialReader(port, baud)
        self.reader.packet_received.connect(self.handle_packet)
        self.reader.status_msg.connect(self.handle_status)
        self.reader.start()
        
    def disconnect_serial(self):
        if self.reader is not None:
            self.reader.stop()
            self.reader.wait()
            self.reader = None

    def send_command(self, cmd):
        if self.reader is None or not self.reader.isRunning():
            self.handle_status("Serial is not connected")
            return
        
        self.reader.send_command(cmd)
        self.handle_status(f"Send command: {cmd}")

    def apply_setting(self):
        fs = int(self.fs_combo.currentText())
        fft_size = int(self.fft_combo.currentText())

        MainConfig.current_fs = fs
        MainConfig.current_fft_size =  fft_size

        self.setting_label.setText(f"fs: {fs} Hz | FFT_SIZE: {fft_size}")

        self.reset_runtime_buffers(fs , fft_size)
        cmd = f"SET FS={fs} FFT={fft_size}"
        self.send_command(cmd)

    def start_recording(self):
        duration_text = self.rec_duration_edit.text().strip()

        try:
            duration = int(duration_text)
            if duration <= 0:
                raise ValueError
        except ValueError:
            self.handle_status("Invalid recording duration")
            return
        
        cmd = f"REC {duration}"
        self.send_command(cmd)

    def stop_recording(self):
        self.send_command("STOP_REC")

    def reset_runtime_buffers(self, fs, fft_size):
        fft_bins = fft_size // 2

        self.freq_axis = np.arange(fft_bins) * fs / fft_size
        self.specgram = np.full((fft_bins , SPECGRAM_LEN) , -120.0)

        self.frame_history.clear()
        self.peak_freq_history.clear()
        self.peak_mag_history.clear()

        self.fft_curve.setData([],[])
        self.freq_trend_curve.setData([],[])
        self.freq_ma_curve.setData([],[])
        self.mag_trend_curve.setData([],[])
        self.mag_ma_curve.setData([],[])

        self.specgram_img.setImage(self.specgram.T, autoLevels=False)

    def handle_status(self, msg):
        self.status_label.setText(f"Status: {msg}")
        self.append_text(self.parsed_text, msg , limit=200)

    def start_logging(self):
        if self.logging_enabled:
            return
        
        fft_bins = MainConfig.current_fft_size // 2

        filename = time.strftime("fft_log_%Y%m%d_%H%M%S.csv")
        self.csv_file = open(filename, "w" , newline="" , encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)

        header = ["timestamp" , "msg_id" , "fs" , "fft_size" , "peak_freq_hz" , "peak_mag_db"]
        header += [f"bin{i}" for i in range(fft_bins)]

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
        self.handle_status("Logging Stopped")

    def clear_logs(self):
        self.parsed_text.clear()
        self.raw_text.clear()

        self.frame_history.clear()
        self.peak_freq_history.clear()
        self.peak_mag_history.clear()

        self.fft_curve.setData([],[])
        self.freq_trend_curve.setData([],[])
        self.freq_ma_curve.setData([],[])
        self.mag_trend_curve.setData([],[])
        self.mag_ma_curve.setData([],[])

        self.specgram = None
        self.specgram_img.clear()

        self.frame_label.setText("Frame: -")
        self.crc_label.setText("CRC: -")
        self.peak_freq_label.setText("Peak Freq: -")
        self.peak_mag_label.setText("Peak Mag: -")

    def moving_average(self, data, window):
        data = np.array(data, dtype=float)

        if len(data) == 0:
            return np.array([])
        
        if len(data) < window:
            return data.copy()
        
        kernel = np.ones(window) / window
        ma = np.convolve(data , kernel , mode="valid")
        pad = np.full(window - 1 ,np.nan)

        return np.concatenate([pad, ma])
    
    def handle_packet(self, msg_id , values, raw_hex, crc_ok, peak_freq, peak_mag, fs, fft_size):
        fft_bins = fft_size // 2

        if len(values) != fft_bins:
            self.handle_status(f"FFT length mismatch: got {len(values)}, expected {fft_bins}")
            return
        
        MainConfig.current_fs = fs
        MainConfig.current_fft_size = fft_size

        self.freq_axis = np.arange(fft_bins) * fs / fft_size
        mag_db = 20.0 * np.log10(values.astype(float) + 1e-12)

        self.latest_values = values
        self.latest_mag_db = mag_db

        self.frame_label.setText(f"Frame: {msg_id}")
        self.crc_label.setText(f"CRC: {'Ok' if crc_ok else 'ERROR'}")
        self.peak_freq_label.setText(f"Peak Freq: {peak_freq:.2f} Hz")
        self.peak_mag_label.setText(f"Peak Mag: {peak_mag:.2f} dB")

        # FFT display
        self.fft_curve.setData(mag_db , self.freq_axis)
        self.fft_plot.setYRange(0 , fs/2)

        x_min = float(np.nanmin(mag_db))
        x_max = float(np.nanmax(mag_db))
        if np.isfinite(x_min) and np.isfinite(x_max) and x_max > x_min:
            self.fft_plot.setXRange(x_min - 5, x_max + 5)
        
        # Specgram update
        if self.specgram is None or self.specgram.shape[0] != fft_bins:
            self.specgram = np.full((fft_bins , SPECGRAM_LEN) , -120.0)
        
        self.specgram = np.roll(self.specgram , -1 , axis=1)
        self.specgram[: , -1] = mag_db

        self.specgram_img.setImage(self.specgram.T, autoLevels=False)
        self.specgram_img.setRect(QtCore.QRectF(0,0,SPECGRAM_LEN,fs/2))

        vmin = np.percentile(self.specgram, 5)
        vmax = np.percentile(self.specgram, 95)
        if np.isfinite(vmin) and np.isfinite(vmax) and vmax > vmin:
            self.colorbar.setLevels((vmin, vmax))

        # Trend update
        self.frame_history.append(msg_id)
        self.peak_freq_history.append(peak_freq)
        self.peak_mag_history.append(peak_mag)

        frames = np.array(self.frame_history)
        peak_freqs = np.array(self.peak_freq_history)
        peak_mags = np.array(self.peak_mag_history)

        freq_ma = self.moving_average(peak_freqs, MA_WINDOW)
        mag_ma = self.moving_average(peak_mags, MA_WINDOW)

        self.freq_trend_curve.setData(frames, peak_freqs)
        self.freq_ma_curve.setData(frames , freq_ma)
        
        self.mag_trend_curve.setData(frames, peak_mags)
        self.mag_ma_curve.setData(frames, mag_ma)

        summary = (
            f"Frame {msg_id} | CRC {'OK' if crc_ok else 'ERROR'} | "
            f"fs {fs} Hz | FFT {fft_size} | "
            f"Peak Freq {peak_freq:.2f} Hz | Peak Mag {peak_mag:.2f} dB"
            )

        self.append_text(self.parsed_text, summary , limit=200)
        self.append_text(self.raw_text , raw_hex , limit = 200)

        if self.logging_enabled and self.csv_writer is not None:
            row = [time.time(), msg_id , fs , fft_size , peak_freq ,  peak_mag]
            row += values.tolist()
            self.csv_writer.writerow(row)
            self.csv_file.flush()

    def append_text(self, widget, text , limit=200):
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
