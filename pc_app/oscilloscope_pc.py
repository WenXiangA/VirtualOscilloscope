import sys
import os
import time
import json
import sqlite3
import numpy as np
import serial
import serial.tools.list_ports
from datetime import datetime

from PySide6.QtCore import Qt, QThread, Signal, Slot, QTimer
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QPushButton, QComboBox, QSlider,
    QSpinBox, QDoubleSpinBox, QCheckBox, QFileDialog, QMessageBox,
    QTableWidget, QTableWidgetItem, QHeaderView, QTabWidget
)
import pyqtgraph as pg

# ─── 数据通信协议常量 ───
HEADER_MCU_TO_PC = bytes([0xAA, 0x55])
HEADER_PC_TO_MCU = bytes([0x55, 0xAA])
FRAME_SIZE = 256
UPLOAD_FRAME_BYTES = 522  # 2 + 1 + 4 + 2 + 256*2 + 1
CONTROL_FRAME_BYTES = 12   # 2 + 1 + 2 + 2 + 2 + 2 + 1

# ─── 数据库路径 ───
DB_PATH = "oscilloscope_data.db"

# ─── 硬件安全边界常量 ───
SAFE_MIN = 0.1  # 安全下限 (V)
SAFE_MAX = 3.2  # 安全上限 (V)

# ─── 数据库初始化与操作 ───
def init_db():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS measurements (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TEXT,
            wave_type TEXT,
            target_freq REAL,
            target_ampl REAL,
            v_max REAL,
            v_min REAL,
            v_pp REAL,
            v_rms REAL,
            measured_freq REAL,
            waveform_data TEXT
        )
    """)
    # 自动升级：为旧数据库动态添加 sample_rate 字段
    try:
        cursor.execute("ALTER TABLE measurements ADD COLUMN sample_rate INTEGER DEFAULT 5000")
        conn.commit()
    except sqlite3.OperationalError:
        pass  # 字段已存在或表不存在时忽略错误
    conn.close()

def save_to_db(wave_type, target_freq, target_ampl, v_max, v_min, v_pp, v_rms, measured_freq, y_data, sample_rate):
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    waveform_json = json.dumps(y_data.tolist())
    cursor.execute("""
        INSERT INTO measurements (timestamp, wave_type, target_freq, target_ampl, v_max, v_min, v_pp, v_rms, measured_freq, waveform_data, sample_rate)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    """, (timestamp, wave_type, target_freq, target_ampl, v_max, v_min, v_pp, v_rms, measured_freq, waveform_json, sample_rate))
    conn.commit()
    conn.close()


# ─── 串口后台数据读取线程 ───
class SerialReaderThread(QThread):
    frame_received = Signal(int, int, np.ndarray)  # wave_type, sample_rate, voltages
    error_occurred = Signal(str)

    def __init__(self, port, baudrate=460800):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.ser.reset_input_buffer()
            self.running = True
        except Exception as e:
            self.error_occurred.emit(f"无法打开串口端口: {str(e)}")
            return

        buffer = bytearray()
        
        while self.running:
            try:
                # 读取当前串口内所有可用数据
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer.extend(data)
                else:
                    time.sleep(0.002)
                    continue

                # 循环解析缓冲区中的数据帧
                while len(buffer) >= UPLOAD_FRAME_BYTES:
                    # 查找帧头位置
                    head_idx = buffer.find(HEADER_MCU_TO_PC)
                    if head_idx == -1:
                        # 未找到帧头，保留缓冲区最后一个字节（防止半个帧头被裁切）
                        if len(buffer) > 0:
                            buffer = buffer[-1:]
                        break
                    
                    if head_idx > 0:
                        # 丢弃帧头前面的无效残留字节
                        buffer = buffer[head_idx:]
                        if len(buffer) < UPLOAD_FRAME_BYTES:
                            break

                    # 校验和验证
                    frame = buffer[:UPLOAD_FRAME_BYTES]
                    checksum = 0
                    for i in range(2, UPLOAD_FRAME_BYTES - 1):
                        checksum = (checksum + frame[i]) & 0xFF
                    
                    if checksum == frame[-1]:
                        # 校验成功，提取数据
                        wave_type = frame[2]
                        sample_rate = int.from_bytes(frame[3:7], byteorder='little')
                        n_points = int.from_bytes(frame[7:9], byteorder='little')
                        
                        # 提取 ADC 12位原始读数
                        raw_data = np.frombuffer(frame[9:9 + n_points * 2], dtype='<u2')
                        # 转换为对应电压值 (12位ADC，基准电压3.3V)
                        voltages = raw_data.astype(np.float32) * 3.3 / 4095.0
                        
                        self.frame_received.emit(wave_type, sample_rate, voltages)
                        # 移出已处理完成的帧数据
                        buffer = buffer[UPLOAD_FRAME_BYTES:]
                    else:
                        # 校验失败，跳过前2字节帧头继续检索对齐
                        buffer = buffer[2:]

            except Exception as e:
                if self.running:
                    self.error_occurred.emit(f"串口通信发生错误: {str(e)}")
                    break
        
        # 线程关闭清理
        if self.ser and self.ser.is_open:
            self.ser.close()

    def stop(self):
        self.running = False
        self.wait()


# ─── 主窗口 GUI 界面 ───
class OscilloscopeApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("简易虚拟示波器")
        self.resize(1200, 750)
        
        # 初始化数据库
        init_db()

        # 核心状态变量
        self.serial_thread = None
        self.is_connected = False
        self.frozen = False
        self.fps_last_time = time.time()
        self.fps_counter = 0
        self.current_fps = 0.0
        
        # 缓存最新收到的采样数据
        self.latest_voltages = None
        self.latest_sample_rate = 5000
        self.latest_wave_type = 0

        # UI 样式设计 (科技感暗黑界面)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #121212;
            }
            QWidget {
                color: #E0E0E0;
                font-family: 'Microsoft YaHei', Arial, sans-serif;
                font-size: 13px;
            }
            QGroupBox {
                background-color: #1E1E1E;
                border: 1px solid #2D2D2D;
                border-radius: 8px;
                margin-top: 15px;
                padding: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 10px;
                padding: 0 5px;
                color: #00ADB5;
            }
            QPushButton {
                background-color: #393E46;
                border: 1px solid #4E5561;
                border-radius: 4px;
                padding: 6px 12px;
                min-width: 70px;
            }
            QPushButton:hover {
                background-color: #4E5561;
            }
            QPushButton:pressed {
                background-color: #00ADB5;
                color: #121212;
            }
            QPushButton#connectBtn[connected="true"] {
                background-color: #D32F2F;
                border-color: #E53935;
                color: white;
            }
            QPushButton#connectBtn[connected="true"]:hover {
                background-color: #F44336;
            }
            QPushButton#runBtn[frozen="true"] {
                background-color: #388E3C;
                color: white;
            }
            QComboBox, QSpinBox, QDoubleSpinBox {
                background-color: #222831;
                border: 1px solid #393E46;
                border-radius: 4px;
                padding: 4px;
                color: #EEEEEE;
            }
            QComboBox::drop-down {
                border: none;
            }
            QSlider::groove:horizontal {
                border: 1px solid #393E46;
                height: 6px;
                background: #222831;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #00ADB5;
                width: 14px;
                height: 14px;
                margin: -4px 0;
                border-radius: 7px;
            }
            QSlider::handle:horizontal:hover {
                background: #00FFF5;
            }
            QTableWidget {
                background-color: #1E1E1E;
                gridline-color: #2D2D2D;
                border: 1px solid #2D2D2D;
                border-radius: 4px;
            }
            QHeaderView::section {
                background-color: #222831;
                color: #00ADB5;
                padding: 5px;
                border: 1px solid #2D2D2D;
            }
            QLabel#titleLabel {
                font-size: 20px;
                font-weight: bold;
                color: #00ADB5;
                margin-bottom: 5px;
            }
            QTabBar::tab {
                background-color: #1E1E1E;
                color: #888888;
                padding: 8px 20px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                margin-right: 2px;
            }
            QTabBar::tab:selected {
                background-color: #2D2D2D;
                color: #00ADB5;
                border-bottom: 2px solid #00ADB5;
            }
        """)

        # 主布局
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # ─── 左侧区域: 示波器网格显示与数值测量 ───
        left_layout = QVBoxLayout()
        main_layout.addLayout(left_layout, stretch=3)

        # 顶部标题与状态条
        top_bar = QHBoxLayout()
        title_lbl = QLabel("简易虚拟示波器")
        title_lbl.setObjectName("titleLabel")
        top_bar.addWidget(title_lbl)
        
        self.status_lbl = QLabel("串口未连接")
        self.status_lbl.setStyleSheet("color: #888888; font-weight: bold;")
        top_bar.addWidget(self.status_lbl, alignment=Qt.AlignRight | Qt.AlignVCenter)
        left_layout.addLayout(top_bar)

        # 选项卡组件
        self.tabs = QTabWidget()
        left_layout.addWidget(self.tabs)

        # 示波器显示选项卡
        osc_tab = QWidget()
        osc_layout = QVBoxLayout(osc_tab)
        osc_layout.setContentsMargins(0, 5, 0, 0)
        
        # PyQtGraph 波形网格视窗
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#1A1A1A')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel('left', '电压', units='V')
        self.plot_widget.setLabel('bottom', '时间', units='s')
        self.plot_widget.setYRange(0, 3.3)
        
        # 示波器波形画笔配置
        self.trace_pen = pg.mkPen(color='#00ADB5', width=2.5, style=Qt.SolidLine)
        self.curve = self.plot_widget.plot(pen=self.trace_pen)
        
        # 动态可拖拽的触发电平虚线 (橙色)
        self.trigger_line = pg.InfiniteLine(angle=0, movable=True, pen=pg.mkPen('#FF8A3B', width=1.5, style=Qt.DashLine))
        self.trigger_line.setValue(1.65)  # 默认设置在中点 1.65V
        self.plot_widget.addItem(self.trigger_line)
        self.trigger_line.sigPositionChanged.connect(self.on_trigger_line_dragged)
        
        osc_layout.addWidget(self.plot_widget, stretch=4)

        # 实时指标测量面板
        meas_group = QGroupBox("实时测量参数")
        meas_layout = QGridLayout(meas_group)
        meas_layout.setSpacing(10)
        
        self.val_vmax = QLabel("0.00 V")
        self.val_vmin = QLabel("0.00 V")
        self.val_vpp = QLabel("0.00 V")
        self.val_vrms = QLabel("0.00 V")
        self.val_freq = QLabel("0.00 Hz")
        self.val_vavg = QLabel("0.00 V")

        lbl_style = "font-weight: bold; font-size: 16px; color: #EEEEEE;"
        title_style = "color: #888888; font-size: 12px;"

        self.val_vmax.setStyleSheet(lbl_style)
        self.val_vmin.setStyleSheet(lbl_style)
        self.val_vpp.setStyleSheet(lbl_style)
        self.val_vrms.setStyleSheet(lbl_style)
        self.val_freq.setStyleSheet(lbl_style)
        self.val_vavg.setStyleSheet(lbl_style)

        # 布局排列
        # 第一排
        meas_layout.addWidget(QLabel("最大电压 (Vmax):", styleSheet=title_style), 0, 0)
        meas_layout.addWidget(self.val_vmax, 0, 1)
        meas_layout.addWidget(QLabel("最小电压 (Vmin):", styleSheet=title_style), 0, 2)
        meas_layout.addWidget(self.val_vmin, 0, 3)
        meas_layout.addWidget(QLabel("峰峰值 (Vpp):", styleSheet=title_style), 0, 4)
        meas_layout.addWidget(self.val_vpp, 0, 5)
        # 第二排
        meas_layout.addWidget(QLabel("有效值 (Vrms):", styleSheet=title_style), 1, 0)
        meas_layout.addWidget(self.val_vrms, 1, 1)
        meas_layout.addWidget(QLabel("信号频率:", styleSheet=title_style), 1, 2)
        meas_layout.addWidget(self.val_freq, 1, 3)
        meas_layout.addWidget(QLabel("平均电压 (Vavg):", styleSheet=title_style), 1, 4)
        meas_layout.addWidget(self.val_vavg, 1, 5)

        osc_layout.addWidget(meas_group, stretch=1)
        self.tabs.addTab(osc_tab, "示波器显示")

        # 数据库历史记录面板
        hist_tab = QWidget()
        hist_layout = QVBoxLayout(hist_tab)
        
        self.history_table = QTableWidget(0, 10)
        self.history_table.setHorizontalHeaderLabels([
            "序号", "时间戳", "波形类型", "设定频率 (Hz)", 
            "设定幅值 (V)", "最大电压 (V)", "最小电压 (V)", "峰峰值 (V)", "有效值 (V)", "测量频率 (Hz)"
        ])
        self.history_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.history_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.history_table.setSelectionMode(QTableWidget.SingleSelection)
        self.history_table.doubleClicked.connect(self.load_history_item)
        hist_layout.addWidget(self.history_table)

        btn_hist_layout = QHBoxLayout()
        refresh_hist_btn = QPushButton("刷新列表")
        refresh_hist_btn.clicked.connect(self.refresh_history)
        clear_hist_btn = QPushButton("清空数据库")
        clear_hist_btn.clicked.connect(self.clear_history)
        btn_hist_layout.addWidget(refresh_hist_btn)
        btn_hist_layout.addWidget(clear_hist_btn)
        btn_hist_layout.addStretch()
        hist_layout.addLayout(btn_hist_layout)
        
        self.tabs.addTab(hist_tab, "数据库历史记录")

        # ─── 右侧区域: 示波器控制器面板 ───
        right_panel = QVBoxLayout()
        main_layout.addLayout(right_panel, stretch=1)

        # 串口连接控制器
        conn_box = QGroupBox("串口连接设置")
        conn_grid = QGridLayout(conn_box)
        conn_grid.addWidget(QLabel("选择串口:"), 0, 0)
        self.port_combo = QComboBox()
        conn_grid.addWidget(self.port_combo, 0, 1)

        self.refresh_port_btn = QPushButton("刷新")
        self.refresh_port_btn.clicked.connect(self.scan_ports)
        conn_grid.addWidget(self.refresh_port_btn, 0, 2)

        self.connect_btn = QPushButton("连接串口")
        self.connect_btn.setObjectName("connectBtn")
        self.connect_btn.clicked.connect(self.toggle_connection)
        conn_grid.addWidget(self.connect_btn, 1, 0, 1, 3)

        right_panel.addWidget(conn_box)

        # 信号源控制面板 (下发指令)
        wave_box = QGroupBox("信号源参数控制")
        wave_grid = QGridLayout(wave_box)
        
        wave_grid.addWidget(QLabel("波形选择:"), 0, 0)
        self.wavetype_combo = QComboBox()
        self.wavetype_combo.addItems(["正弦波", "方波", "三角波"])
        self.wavetype_combo.currentIndexChanged.connect(self.send_controls)
        wave_grid.addWidget(self.wavetype_combo, 0, 1)

        wave_grid.addWidget(QLabel("输出频率 (10-500 Hz):"), 1, 0)
        self.freq_spin = QSpinBox()
        self.freq_spin.setRange(10, 500)
        self.freq_spin.setValue(100)
        self.freq_spin.setSuffix(" Hz")
        self.freq_spin.valueChanged.connect(self.send_controls)
        wave_grid.addWidget(self.freq_spin, 1, 1)

        wave_grid.addWidget(QLabel("输出幅值 (0-1.5V):"), 2, 0)
        self.ampl_slider = QSlider(Qt.Horizontal)
        self.ampl_slider.setRange(0, 150)  # 代表 0.00 到 1.50V (以 0.01V 为步进)
        self.ampl_slider.setValue(100)     # 默认幅值 1.00V
        self.ampl_slider.valueChanged.connect(self.on_ampl_slider_changed)
        wave_grid.addWidget(self.ampl_slider, 2, 1)
        
        self.ampl_lbl = QLabel("1.00 V (峰峰值: 2.00 V)")
        self.ampl_lbl.setStyleSheet("color: #00ADB5;")
        wave_grid.addWidget(self.ampl_lbl, 3, 0, 1, 2, alignment=Qt.AlignCenter)

        wave_grid.addWidget(QLabel("直流偏置 (1.0-2.0V):"), 4, 0)
        self.offset_slider = QSlider(Qt.Horizontal)
        self.offset_slider.setRange(100, 200)  # 代表 1.00V 到 2.00V
        self.offset_slider.setValue(165)       # 默认 1.65V
        self.offset_slider.valueChanged.connect(self.on_offset_slider_changed)
        wave_grid.addWidget(self.offset_slider, 4, 1)

        self.offset_lbl = QLabel("1.65 V")
        self.offset_lbl.setStyleSheet("color: #00ADB5;")
        wave_grid.addWidget(self.offset_lbl, 5, 0, 1, 2, alignment=Qt.AlignCenter)

        wave_grid.addWidget(QLabel("采集采样率:"), 6, 0)
        self.fs_combo = QComboBox()
        self.fs_combo.addItems(["5 kHz", "10 kHz", "20 kHz"])
        self.fs_combo.currentIndexChanged.connect(self.send_controls)
        wave_grid.addWidget(self.fs_combo, 6, 1)
        
        right_panel.addWidget(wave_box)

        # 触发器参数设置面板
        trig_box = QGroupBox("触发器参数设置")
        trig_grid = QGridLayout(trig_box)
        
        trig_grid.addWidget(QLabel("触发模式:"), 0, 0)
        self.trig_mode_combo = QComboBox()
        self.trig_mode_combo.addItems(["自动触发", "普通触发", "单次触发"])
        trig_grid.addWidget(self.trig_mode_combo, 0, 1)

        trig_grid.addWidget(QLabel("触发沿:"), 1, 0)
        self.trig_slope_combo = QComboBox()
        self.trig_slope_combo.addItems(["上升沿", "下降沿"])
        trig_grid.addWidget(self.trig_slope_combo, 1, 1)

        trig_grid.addWidget(QLabel("触发电平 (0-3.3V):"), 2, 0)
        self.trig_level_spin = QDoubleSpinBox()
        self.trig_level_spin.setRange(0.0, 3.3)
        self.trig_level_spin.setValue(1.65)
        self.trig_level_spin.setSingleStep(0.1)
        self.trig_level_spin.setSuffix(" V")
        self.trig_level_spin.valueChanged.connect(self.on_trig_level_spin_changed)
        trig_grid.addWidget(self.trig_level_spin, 2, 1)
        
        right_panel.addWidget(trig_box)

        # 波形显示配置面板
        disp_box = QGroupBox("波形显示设置")
        disp_grid = QGridLayout(disp_box)

        self.freeze_btn = QPushButton("暂停波形")
        self.freeze_btn.setObjectName("runBtn")
        self.freeze_btn.setProperty("frozen", False)
        self.freeze_btn.clicked.connect(self.toggle_freeze)
        disp_grid.addWidget(self.freeze_btn, 0, 0, 1, 2)

        disp_grid.addWidget(QLabel("水平时基 (X轴):"), 1, 0)
        self.time_div_combo = QComboBox()
        self.time_div_combo.addItems(["自动时基", "10 ms", "20 ms", "50 ms"])
        self.time_div_combo.currentIndexChanged.connect(self.update_axis_ranges)
        disp_grid.addWidget(self.time_div_combo, 1, 1)

        disp_grid.addWidget(QLabel("垂直分度 (Y轴):"), 2, 0)
        self.volt_div_combo = QComboBox()
        self.volt_div_combo.addItems(["0 - 3.3 V", "1.0 - 2.5 V", "自动量程"])
        self.volt_div_combo.currentIndexChanged.connect(self.update_axis_ranges)
        disp_grid.addWidget(self.volt_div_combo, 2, 1)

        right_panel.addWidget(disp_box)

        # 数据存储与导出面板
        data_box = QGroupBox("测量数据管理")
        data_layout = QVBoxLayout(data_box)
        
        save_db_btn = QPushButton("存入数据库")
        save_db_btn.clicked.connect(self.save_current_data_db)
        data_layout.addWidget(save_db_btn)

        export_csv_btn = QPushButton("导出至 CSV 文件")
        export_csv_btn.clicked.connect(self.export_csv)
        data_layout.addWidget(export_csv_btn)

        right_panel.addWidget(data_box)
        right_panel.addStretch()

        # 单次触发状态锁
        self.single_armed = False

        # 诊断定时器（用于监测串口物理数据流）
        self.diagnostic_timer = QTimer()
        self.diagnostic_timer.setInterval(1000)
        self.diagnostic_timer.timeout.connect(self.run_diagnostics)
        self.frames_received_since_last_sec = 0

        # 初始化调用与状态更新
        self.update_axis_ranges()
        self.refresh_history()
        self.scan_ports()

    # ─── 串口扫描功能 ───
    def scan_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_combo.addItem(f"{p.device} ({p.description})")
        
        if self.port_combo.count() == 0:
            self.port_combo.addItem("未检测到串口")
            self.connect_btn.setEnabled(False)
        else:
            self.connect_btn.setEnabled(True)

    def run_diagnostics(self):
        if not self.is_connected:
            return
            
        if self.frames_received_since_last_sec > 0:
            self.status_lbl.setText(f"已连接至 {self.serial_thread.port} | 接收正常 (帧率: {self.current_fps:.1f} FPS)")
            self.status_lbl.setStyleSheet("color: #00ADB5; font-weight: bold;")
        else:
            self.status_lbl.setText(f"已连接至 {self.serial_thread.port} | ⚠️ 无数据，请检查：1. 串口接线(TXD/RXD) 2. 单片机GND共地 3. 单片机LED是否闪烁")
            self.status_lbl.setStyleSheet("color: #FF8A3B; font-weight: bold;")
            
        self.frames_received_since_last_sec = 0

    # ─── 串口连接/断开逻辑 ───
    def toggle_connection(self):
        if not self.is_connected:
            # 执行连接
            port_text = self.port_combo.currentText()
            if not port_text or "未检测到串口" in port_text:
                QMessageBox.warning(self, "警告", "请选择一个有效的串口端口。")
                return
            
            port = port_text.split(" ")[0] # 提取 COM 端口名
            
            # 开启后台接收线程
            self.serial_thread = SerialReaderThread(port, 460800)
            self.serial_thread.frame_received.connect(self.on_frame_received)
            self.serial_thread.error_occurred.connect(self.on_serial_error)
            
            # 更新界面连接按钮样式
            self.connect_btn.setText("断开串口")
            self.connect_btn.setProperty("connected", True)
            self.connect_btn.setStyle(self.connect_btn.style()) # 强制使 QSS 生效
            self.is_connected = True
            
            self.status_lbl.setText(f"已连接至 {port}")
            self.status_lbl.setStyleSheet("color: #00ADB5; font-weight: bold;")
            
            self.frames_received_since_last_sec = 0
            self.diagnostic_timer.start()
            self.serial_thread.start()
            
            # 延迟 500ms 发送一次当前的滑块参数给单片机
            QTimer.singleShot(500, self.send_controls)
        else:
            # 执行断开
            self.disconnect_serial()

    def disconnect_serial(self):
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread = None
        
        self.connect_btn.setText("连接串口")
        self.connect_btn.setProperty("connected", False)
        self.connect_btn.setStyle(self.connect_btn.style())
        self.is_connected = False
        self.diagnostic_timer.stop()
        
        self.status_lbl.setText("串口已断开")
        self.status_lbl.setStyleSheet("color: #888888; font-weight: bold;")

    def on_serial_error(self, err_msg):
        QMessageBox.critical(self, "串口错误", err_msg)
        self.disconnect_serial()

    # ─── 下发控制参数 ───
    def send_controls(self):
        if not self.is_connected or not self.serial_thread or not self.serial_thread.ser:
            return
            
        wave_type = self.wavetype_combo.currentIndex()
        freq = self.freq_spin.value()
        
        # 获取幅值与偏置并转换为毫伏 (mV)
        ampl_mv = int(self.ampl_slider.value() * 10)  # 0~150 -> 0~1500 mV
        offset_mv = int(self.offset_slider.value() * 10) # 100~200 -> 1000~2000 mV
        
        # 采样率选项
        fs_options = [5000, 10000, 20000]
        sample_rate = fs_options[self.fs_combo.currentIndex()]
        
        # 组装 12 字节控制协议帧
        # 帧头(2) + 波形(1) + 频率(2) + 幅值(2) + 偏置(2) + 采样率(2) + 校验和(1)
        frame = bytearray(12)
        frame[0:2] = HEADER_PC_TO_MCU
        frame[2] = wave_type
        frame[3:5] = int(freq).to_bytes(2, byteorder='little')
        frame[5:7] = int(ampl_mv).to_bytes(2, byteorder='little')
        frame[7:9] = int(offset_mv).to_bytes(2, byteorder='little')
        frame[9:11] = int(sample_rate).to_bytes(2, byteorder='little')
        
        # 累加校验和
        cs = sum(frame[0:11]) & 0xFF
        frame[11] = cs
        
        try:
            self.serial_thread.ser.write(frame)
        except Exception as e:
            self.status_lbl.setText(f"下发参数失败: {str(e)}")

    def on_offset_slider_changed(self, val):
        offset = val / 100.0
        self.offset_lbl.setText(f"{offset:.2f} V")
        
        # 限制幅值滑块合法范围：amp <= offset - SAFE_MIN 且 amp <= SAFE_MAX - offset
        max_amp = min(offset - SAFE_MIN, SAFE_MAX - offset)
        max_amp_val = max(0, min(150, int(max_amp * 100)))
        
        if self.ampl_slider.value() > max_amp_val:
            self.ampl_slider.blockSignals(True)
            self.ampl_slider.setValue(max_amp_val)
            amp_volts = max_amp_val / 100.0
            self.ampl_lbl.setText(f"{amp_volts:.2f} V (峰峰值: {amp_volts*2:.2f} V)")
            self.ampl_slider.blockSignals(False)
            
        self.send_controls()

    def on_ampl_slider_changed(self, val):
        amp = val / 100.0
        self.ampl_lbl.setText(f"{amp:.2f} V (峰峰值: {amp*2:.2f} V)")
        
        # 限制偏置滑块范围：offset >= amp + SAFE_MIN 且 offset <= SAFE_MAX - amp
        min_offset_val = int((amp + SAFE_MIN) * 100)
        max_offset_val = int((SAFE_MAX - amp) * 100)
        
        current_offset_val = self.offset_slider.value()
        if current_offset_val < min_offset_val:
            self.offset_slider.blockSignals(True)
            self.offset_slider.setValue(min_offset_val)
            self.offset_lbl.setText(f"{(min_offset_val/100.0):.2f} V")
            self.offset_slider.blockSignals(False)
        elif current_offset_val > max_offset_val:
            self.offset_slider.blockSignals(True)
            self.offset_slider.setValue(max_offset_val)
            self.offset_lbl.setText(f"{(max_offset_val/100.0):.2f} V")
            self.offset_slider.blockSignals(False)
            
        self.send_controls()

    # ─── 拖拽触发电平虚线时的同步反馈 ───
    def on_trigger_line_dragged(self, line):
        val = line.value()
        # 钳位限制在 0.0V 到 3.3V 以内
        if val < 0: val = 0.0
        if val > 3.3: val = 3.3
        line.setValue(val)
        
        # 双向同步更新微调数值框
        self.trig_level_spin.blockSignals(True)
        self.trig_level_spin.setValue(val)
        self.trig_level_spin.blockSignals(False)

    def on_trig_level_spin_changed(self, val):
        # 调整数值框同步更新绘图虚线位置
        self.trigger_line.blockSignals(True)
        self.trigger_line.setValue(val)
        self.trigger_line.blockSignals(False)

    # ─── 数据帧解析与触发绘制核心逻辑 ───
    @Slot(int, int, np.ndarray)
    def on_frame_received(self, wave_type, sample_rate, voltages):
        self.frames_received_since_last_sec += 1
        if self.frozen:
            return

        self.latest_voltages = voltages
        self.latest_sample_rate = sample_rate
        self.latest_wave_type = wave_type
        
        # 同步下拉框的波形显示，屏蔽信号发送避免死循环
        if self.wavetype_combo.currentIndex() != wave_type:
            self.wavetype_combo.blockSignals(True)
            self.wavetype_combo.setCurrentIndex(wave_type)
            self.wavetype_combo.blockSignals(False)
        
        # 计算图形绘制刷新率
        now = time.time()
        self.fps_counter += 1
        if now - self.fps_last_time >= 1.0:
            self.current_fps = self.fps_counter / (now - self.fps_last_time)
            self.fps_counter = 0
            self.fps_last_time = now

        self.process_and_plot()

    # ─── 波形分析与显示对齐 ───
    def process_and_plot(self):
        if self.latest_voltages is None:
            return
        
        y_raw = self.latest_voltages
        fs = self.latest_sample_rate
        n_points = len(y_raw)
        
        # 解析时间轴宽度与分度值
        time_div = self.time_div_combo.currentText()
        if "自动时基" in time_div:
            width = n_points / fs
            is_auto_tb = True
        else:
            is_auto_tb = False
            width = 0.010
            if "20 ms" in time_div:
                width = 0.020
            elif "50 ms" in time_div:
                width = 0.050

        # 计算安全的触发搜索区域，确保触发点左右的数据能够完全填满波形窗口
        # 防止触发点太靠前或太靠后导致屏幕左右出现空白白边，从而引发画面跳跃/闪烁
        half_width_samples = int((width / 2.0) * fs)
        if is_auto_tb:
            start_search = 0
            end_search = n_points - 1
        else:
            # 限制边界：至少为左右留出 half_width_samples 个点。如果宽度过大，则使用最大可用边界
            margin = min(n_points // 2 - 10, half_width_samples)
            start_search = margin
            end_search = n_points - 1 - margin

        # 1. 软件触发系统 (使显示波形稳定不晃动)
        trig_mode = self.trig_mode_combo.currentText()
        trig_level = self.trig_level_spin.value()
        slope = self.trig_slope_combo.currentText()
        
        trig_idx = -1
        
        if "自动" in trig_mode or "普通" in trig_mode or "单次" in trig_mode:
            # 仅在安全搜索范围内查找触发交越点
            y_search = y_raw[start_search:end_search+1]
            if "上升沿" in slope:
                # 寻找第一个从触发电平下方穿透到上方的点 (y[i-1] < level <= y[i])
                crossings = np.where((y_search[:-1] < trig_level) & (y_search[1:] >= trig_level))[0]
            else:
                # 寻找第一个从触发电平上方穿透到下方的点 (y[i-1] > level >= y[i])
                crossings = np.where((y_search[:-1] > trig_level) & (y_search[1:] <= trig_level))[0]
                
            if len(crossings) > 0:
                trig_idx = crossings[0] + 1 + start_search
            else:
                # 未触发
                if "普通" in trig_mode or "单次" in trig_mode:
                    # 如果不是自动模式，未触发时不刷新波形界面
                    if "单次" in trig_mode and not self.single_armed:
                        return
                    return

        # 对于自动触发模式，若未找到触发点，则默认将其对齐到数据中点，使波形平稳过渡而不是在屏幕上跳跃
        if trig_idx == -1 and "自动" in trig_mode:
            trig_idx = n_points // 2
        
        # 捕获单次触发
        if "单次" in trig_mode:
            if not self.single_armed:
                return
            else:
                # 已捕获单次触发数据，立即暂停波形刷新
                self.single_armed = False
                self.frozen = True
                self.freeze_btn.setText("运行波形")
                self.freeze_btn.setProperty("frozen", True)
                self.freeze_btn.setStyle(self.freeze_btn.style())
        
        # 对齐 X 时间轴
        if trig_idx != -1:
            # 将触发位置（或默认中点）强制对齐至 X轴 of 0点 (T=0)
            t_axis = (np.arange(n_points) - trig_idx) / fs
            y_plot = y_raw
        else:
            # 备用方案（正常情况下自动模式会默认至中点而不走这里）
            t_axis = np.arange(n_points) / fs
            y_plot = y_raw

        # 更新波形图线
        self.curve.setData(t_axis, y_plot)

        # 限制时间轴量程，使波形完美贴合屏幕宽度而不会显示不全或留下多余白边
        if is_auto_tb:
            # 自动时基下，强制 X 轴范围与数据的实际时间范围完全贴合
            self.plot_widget.setXRange(t_axis[0], t_axis[-1], padding=0)
        else:
            # 手动时基下，无论是否真实触发，均将屏幕 X 轴窗口固定锁定在 [-width/2, width/2]
            # 彻底杜绝时间轴原点在 [0, width] 与 [-width/2, width/2] 间跳变引起的连续剧烈抖动
            self.plot_widget.setXRange(-width / 2.0, width / 2.0, padding=0)

        # 2. 测量计算指标
        v_max = float(np.max(y_raw))
        v_min = float(np.min(y_raw))
        v_pp = v_max - v_min
        
        # 计算交流加直流的真有效值 Vrms
        v_rms = float(np.sqrt(np.mean(y_raw ** 2)))
        # 计算平均电压 Vavg (信号的直流分量)
        v_avg = float(np.mean(y_raw))

        # 频率估算逻辑：计算过均值点之间的距离
        mean_v = np.mean(y_raw)
        raw_crossings = np.where(np.diff(y_raw > mean_v))[0]
        
        # 过滤虚假过零点：剔除由于高频噪声/毛刺引起的过密交越点（设定最小交越间隔为 3 个采样点）
        # 既能保证干净信号的绝对测量精度，又能有效过滤高频噪声的干扰
        filtered_crossings = []
        for c in raw_crossings:
            if not filtered_crossings or (c - filtered_crossings[-1]) >= 3:
                filtered_crossings.append(c)
        crossings = np.array(filtered_crossings)
        
        measured_freq = 0.0
        if len(crossings) >= 2:
            periods = np.diff(crossings)
            if len(periods) >= 1:
                # 均值上升与下降交叉，因此间距乘以2为周期
                avg_period_pts = np.mean(periods) * 2.0
                measured_freq = fs / avg_period_pts
            else:
                measured_freq = 0.0
        else:
            measured_freq = 0.0

        # 刷新界面指标参数
        self.val_vmax.setText(f"{v_max:.2f} V")
        self.val_vmin.setText(f"{v_min:.2f} V")
        self.val_vpp.setText(f"{v_pp:.2f} V")
        self.val_vrms.setText(f"{v_rms:.2f} V")
        self.val_vavg.setText(f"{v_avg:.2f} V")
        if measured_freq > 0 and v_pp > 0.1:
            self.val_freq.setText(f"{measured_freq:.1f} Hz")
        else:
            self.val_freq.setText("--- Hz")

    # ─── 运行/暂停波形动作 ───
    def toggle_freeze(self):
        trig_mode = self.trig_mode_combo.currentText()
        if "单次" in trig_mode:
            # 锁定单次触发等待下一次跃迁
            self.single_armed = True
            self.frozen = False
            self.freeze_btn.setText("等待触发...")
            self.freeze_btn.setProperty("frozen", True)
            self.freeze_btn.setStyle(self.freeze_btn.style())
        else:
            self.frozen = not self.frozen
            if self.frozen:
                self.freeze_btn.setText("运行波形")
                self.freeze_btn.setProperty("frozen", True)
            else:
                self.freeze_btn.setText("暂停波形")
                self.freeze_btn.setProperty("frozen", False)
            self.freeze_btn.setStyle(self.freeze_btn.style())

    # ─── 轴量程动态调整 ───
    def update_axis_ranges(self):
        # 调整 X 时间轴
        time_div = self.time_div_combo.currentText()
        if "自动" in time_div:
            # 256个点在 5kHz 下刚好是 51.2ms 完整范围，默认以 0 起步
            self.plot_widget.setXRange(0, 0.0512, padding=0)
        else:
            width = 0.010
            if "20 ms" in time_div:
                width = 0.020
            elif "50 ms" in time_div:
                width = 0.050
            # 默认设置为从 0 开始的宽度范围（未连接时的默认状态）
            self.plot_widget.setXRange(0, width, padding=0)

        # 调整 Y 电压轴
        volt_div = self.volt_div_combo.currentText()
        if "0 - 3.3 V" in volt_div:
            self.plot_widget.setYRange(0, 3.3, padding=0.05)
        elif "1.0 - 2.5 V" in volt_div:
            self.plot_widget.setYRange(1.0, 2.5, padding=0.05)
        elif "自动量程" in volt_div:
            self.plot_widget.enableAutoRange(axis=pg.ViewBox.YAxis)

    # ─── 数据导出为 CSV 文件 ───
    def export_csv(self):
        if self.latest_voltages is None:
            QMessageBox.warning(self, "保存错误", "没有可导出的波形数据。")
            return

        filename, _ = QFileDialog.getSaveFileName(self, "导出波形数据", "", "CSV 文件 (*.csv)")
        if not filename:
            return

        try:
            # 对齐并对准当前屏幕坐标轴时间点后导出
            trig_level = self.trig_level_spin.value()
            slope = self.trig_slope_combo.currentText()
            y_data = self.latest_voltages
            n_points = len(y_data)
            fs = self.latest_sample_rate
            
            if "上升沿" in slope:
                crossings = np.where((y_data[:-1] < trig_level) & (y_data[1:] >= trig_level))[0]
            else:
                crossings = np.where((y_data[:-1] > trig_level) & (y_data[1:] <= trig_level))[0]
            
            trig_idx = crossings[0] + 1 if len(crossings) > 0 else 0
            t_axis = (np.arange(n_points) - trig_idx) / fs

            with open(filename, "w", encoding="utf-8") as f:
                f.write("时间(秒),电压(伏特)\n")
                for t_val, v_val in zip(t_axis, y_data):
                    f.write(f"{t_val:.6f},{v_val:.4f}\n")
            
            QMessageBox.information(self, "成功", "波形数据已成功导出至 CSV！")
        except Exception as e:
            QMessageBox.critical(self, "导出错误", f"保存 CSV 文件失败：\n{str(e)}")

    # ─── 数据测量记录保存至本地 SQLite 数据库 ───
    def save_current_data_db(self):
        if self.latest_voltages is None:
            QMessageBox.warning(self, "保存错误", "没有可保存的波形数据。")
            return

        y_data = self.latest_voltages
        v_max = float(np.max(y_data))
        v_min = float(np.min(y_data))
        v_pp = v_max - v_min
        v_rms = float(np.sqrt(np.mean(y_data ** 2)))
        
        # 频率估测
        mean_v = np.mean(y_data)
        crossings = np.where(np.diff(y_data > mean_v))[0]
        measured_freq = 0.0
        if len(crossings) >= 2:
            periods = np.diff(crossings)
            if len(periods) >= 2:
                avg_period_pts = np.mean(periods) * 2.0
                measured_freq = self.latest_sample_rate / avg_period_pts

        wave_types = ["正弦波", "方波", "三角波"]
        wave_str = wave_types[self.latest_wave_type] if self.latest_wave_type < len(wave_types) else "未知波形"

        target_freq = self.freq_spin.value()
        target_ampl = self.ampl_slider.value() / 100.0

        try:
            save_to_db(wave_str, target_freq, target_ampl, v_max, v_min, v_pp, v_rms, measured_freq, y_data, self.latest_sample_rate)
            QMessageBox.information(self, "成功", "测量指标与波形数据已成功存入 SQLite 数据库！")
            self.refresh_history()
        except Exception as e:
            QMessageBox.critical(self, "数据库错误", f"保存至数据库失败：\n{str(e)}")

    # ─── 数据库列表刷新 ───
    def refresh_history(self):
        self.history_table.setRowCount(0)
        try:
            conn = sqlite3.connect(DB_PATH)
            cursor = conn.cursor()
            cursor.execute("SELECT id, timestamp, wave_type, target_freq, target_ampl, v_max, v_min, v_pp, v_rms, measured_freq FROM measurements ORDER BY id DESC")
            rows = cursor.fetchall()
            
            for row_idx, row_data in enumerate(rows):
                self.history_table.insertRow(row_idx)
                for col_idx, value in enumerate(row_data):
                    item = QTableWidgetItem()
                    if isinstance(value, float):
                        item.setText(f"{value:.2f}")
                    else:
                        item.setText(str(value))
                    item.setFlags(item.flags() ^ Qt.ItemIsEditable) # 只读属性
                    self.history_table.setItem(row_idx, col_idx, item)
            conn.close()
        except Exception as e:
            pass

    # ─── 双击历史数据行进行波形回放 ───
    def load_history_item(self, model_index):
        row = model_index.row()
        item_id = self.history_table.item(row, 0).text()
        
        try:
            conn = sqlite3.connect(DB_PATH)
            cursor = conn.cursor()
            cursor.execute("SELECT wave_type, measured_freq, waveform_data, sample_rate FROM measurements WHERE id=?", (item_id,))
            res = cursor.fetchone()
            conn.close()
            
            if res:
                wave_type_str, freq_val, waveform_json, sample_rate = res
                voltages = np.array(json.loads(waveform_json), dtype=np.float32)
                
                # 加载显示并自动挂起刷新
                self.frozen = True
                self.freeze_btn.setText("运行波形")
                self.freeze_btn.setProperty("frozen", True)
                self.freeze_btn.setStyle(self.freeze_btn.style())
                
                wave_map = {"Sine": 0, "Square": 1, "Triangle": 2, "正弦波": 0, "方波": 1, "三角波": 2}
                self.latest_wave_type = wave_map.get(wave_type_str, 0)
                self.latest_voltages = voltages
                self.latest_sample_rate = sample_rate if sample_rate else 5000
                
                # 切换至波形选项卡重新绘图
                self.tabs.setCurrentIndex(0)
                self.process_and_plot()
                
                QMessageBox.information(self, "波形已加载", f"成功从历史记录加载波形，ID: {item_id}。\n波形显示已暂停。")
        except Exception as e:
            QMessageBox.critical(self, "加载错误", f"加载历史记录失败：\n{str(e)}")

    # ─── 清空历史数据库 ───
    def clear_history(self):
        reply = QMessageBox.question(
            self, "清空数据库", "您确定要删除数据库中所有存储的历史测量记录吗？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            try:
                conn = sqlite3.connect(DB_PATH)
                cursor = conn.cursor()
                cursor.execute("DELETE FROM measurements")
                conn.commit()
                conn.close()
                self.refresh_history()
            except Exception as e:
                QMessageBox.critical(self, "数据库错误", f"清空数据库失败：\n{str(e)}")

    # ─── 窗口关闭清理事件 ───
    def closeEvent(self, event):
        try:
            self.disconnect_serial()
        except Exception:
            pass
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = OscilloscopeApp()
    window.show()
    sys.exit(app.exec())
