"""
receber_dados.py — Pipeline de Vibrações MPU6050 + ESP32
Baseado em udpplotscroll (damianjwilliams) usando pyqtgraph para máxima fluidez.
Instalar: pip install pyqtgraph PyQt5 numpy scipy PyWavelets
"""

import socket
import numpy as np
from scipy import signal
import pywt
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import csv
import time
import sys

# ─── Configurações ────────────────────────────────────────────────
UDP_IP   = "0.0.0.0"
UDP_PORT = 5005
FS       = 2000
N        = 1024       # janela de processamento
HIST     = 150        # pontos no scroll de RMS/curtose
SENSIT   = 16384.0    # MPU6050 ±2g

# ─── Filtro IIR Butterworth 10–800 Hz ─────────────────────────────
sos_iir = signal.butter(2, [10, 800], btype='bandpass', fs=FS, output='sos')

# ─── Socket UDP não bloqueante ────────────────────────────────────
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)
print(f"Aguardando ESP32 na porta {UDP_PORT}...")

# ─── CSV ──────────────────────────────────────────────────────────
csv_file = open("dados_mpu.csv", "w", newline="")
writer   = csv.writer(csv_file)
writer.writerow(["timestamp", "rms_g", "curtose", "fator_crista", "alerta"])

# ─── Buffers de dados ─────────────────────────────────────────────
buffer_udp   = []                        # acumula amostras brutas
sinal_raw    = np.zeros(N)               # última janela raw
sinal_filt   = np.zeros(N)              # última janela filtrada
psd_freqs    = np.linspace(0, FS/2, N//2+1)
psd_vals     = np.ones(N//2+1) * 1e-12
dwt_coef     = np.zeros(70)

# Arrays de scroll — estratégia do udpplotscroll
scroll_rms   = np.zeros(HIST)
scroll_kurt  = np.zeros(HIST) + 3.0

# Dispersão
disp_ok_x, disp_ok_y = [], []
disp_al_x, disp_al_y = [], []

# ─── DSP ──────────────────────────────────────────────────────────
def aplicar_iir(s):
    return signal.sosfilt(sos_iir, s)

def calcular_atributos(s):
    rms  = float(np.sqrt(np.mean(s**2)))
    std  = float(np.std(s))
    kurt = float(np.mean((s - np.mean(s))**4) / (std**4)) if std > 0 else 3.0
    pico = float(np.max(np.abs(s)))
    cfc  = pico / rms if rms > 0 else 0.0
    return rms, kurt, cfc

def classificar(rms, kurt):
    if kurt > 4.0: return "ALERTA — curtose alta", True
    if rms  > 0.1: return "ALERTA — vibracao alta", True
    return "OK", False

# ─── App Qt ───────────────────────────────────────────────────────
app = QtWidgets.QApplication(sys.argv)

# Tema escuro nativo do pyqtgraph
pg.setConfigOption('background', '#0d0d1a')
pg.setConfigOption('foreground', '#aaaacc')

win = pg.GraphicsLayoutWidget(title="Pipeline de Vibrações — MPU6050 + ESP32  |  IIR Butterworth 10–800 Hz")
win.resize(1400, 800)
win.show()

# Linha de status no topo
lbl_status = QtWidgets.QLabel("  Aguardando dados da ESP32...")
lbl_status.setStyleSheet("color: #aaaacc; background: #1a1a2e; font-family: monospace; font-size: 11px; padding: 4px;")

container = QtWidgets.QWidget()
layout    = QtWidgets.QVBoxLayout(container)
layout.setContentsMargins(0, 0, 0, 0)
layout.setSpacing(0)
layout.addWidget(lbl_status)
layout.addWidget(win)
container.setStyleSheet("background: #1a1a2e;")
container.resize(1400, 830)
container.setWindowTitle("Pipeline de Vibrações — MPU6050 + ESP32")
container.show()

# ─── Grade de plots ───────────────────────────────────────────────
def novo_plot(titulo, xlabel, ylabel, row, col):
    p = win.addPlot(row=row, col=col, title=titulo)
    p.setLabel('bottom', xlabel, color='#aaaacc', size='8pt')
    p.setLabel('left',   ylabel, color='#aaaacc', size='8pt')
    p.getAxis('bottom').setTextPen('#aaaacc')
    p.getAxis('left').setTextPen('#aaaacc')
    p.showGrid(x=True, y=True, alpha=0.25)
    p.titleLabel.setText(titulo, color='#ffffff', size='9pt')
    return p

p_sinal = novo_plot("Sinal eixo Z — raw vs IIR filtrado", "Tempo (ms)", "Aceleração (g)", 0, 0)
p_fft   = novo_plot("PSD — Welch",                         "Freq (Hz)",  "PSD (g²/Hz)",    0, 1)
p_dwt   = novo_plot("DWT db4 — detalhe nível 1",           "Amostras",   "Coef.",           0, 2)
p_rms   = novo_plot("RMS — scroll",                        "Janelas",    "RMS (g)",         1, 0)
p_kurt  = novo_plot("Curtose — scroll",                    "Janelas",    "Curtose",         1, 1)
p_disp  = novo_plot("Dispersão curtose × RMS",             "RMS (g)",    "Curtose",         1, 2)

t_ms = np.linspace(0, N/FS*1000, N)

# Curvas
c_raw  = p_sinal.plot(t_ms, sinal_raw,  pen=pg.mkPen('#4a9eff', width=1), name='Raw',         alpha=0.35)
c_filt = p_sinal.plot(t_ms, sinal_filt, pen=pg.mkPen('#00d4aa', width=1.5), name='IIR filtrado')
p_sinal.setXRange(0, N/FS*1000, padding=0)
p_sinal.setYRange(-0.05, 0.05, padding=0)
leg_sinal = p_sinal.addLegend(offset=(5, 5))
leg_sinal.setLabelTextColor('#ffffff')

c_fft = p_fft.plot(psd_freqs, psd_vals, pen=pg.mkPen('#ff9500', width=1))
p_fft.setLogMode(x=False, y=True)
p_fft.setXRange(0, FS/2, padding=0)
p_fft.setYRange(-14, -5, padding=0)
# Regiões removidas pelo filtro
reg1 = pg.LinearRegionItem([0, 10],   brush=pg.mkBrush(255, 60, 60, 30), movable=False)
reg2 = pg.LinearRegionItem([800, FS/2], brush=pg.mkBrush(255, 60, 60, 30), movable=False)
p_fft.addItem(reg1); p_fft.addItem(reg2)

c_dwt  = p_dwt.plot(np.arange(70), dwt_coef, pen=pg.mkPen('#bf5fff', width=1))

c_rms  = p_rms.plot(scroll_rms,  pen=pg.mkPen('#4a9eff', width=1.5))
lim_rms = pg.InfiniteLine(pos=0.1, angle=0, pen=pg.mkPen('#ff4444', width=1, style=QtCore.Qt.DashLine), label='limiar 0.1g', labelOpts={'color':'#ff4444','position':0.9})
p_rms.addItem(lim_rms)
p_rms.setXRange(0, HIST-1, padding=0)
p_rms.setYRange(0, 0.3, padding=0)

c_kurt = p_kurt.plot(scroll_kurt, pen=pg.mkPen('#ff9500', width=1.5))
lim_k4 = pg.InfiniteLine(pos=4.0, angle=0, pen=pg.mkPen('#ff4444', width=1, style=QtCore.Qt.DashLine), label='alerta > 4', labelOpts={'color':'#ff4444','position':0.9})
lim_k3 = pg.InfiniteLine(pos=3.0, angle=0, pen=pg.mkPen('#888888', width=1, style=QtCore.Qt.DotLine),  label='normal ≈ 3', labelOpts={'color':'#888888','position':0.1})
p_kurt.addItem(lim_k4); p_kurt.addItem(lim_k3)
p_kurt.setXRange(0, HIST-1, padding=0)
p_kurt.setYRange(0, 8, padding=0)

c_disp_ok = p_disp.plot([], [], pen=None, symbol='o', symbolSize=5, symbolBrush='#00d4aa', symbolPen=None, name='Saudável')
c_disp_al = p_disp.plot([], [], pen=None, symbol='o', symbolSize=5, symbolBrush='#ff4444', symbolPen=None, name='Alerta')
lim_dx = pg.InfiniteLine(pos=0.1,  angle=90, pen=pg.mkPen('#ff4444', width=1, style=QtCore.Qt.DashLine))
lim_dy = pg.InfiniteLine(pos=4.0,  angle=0,  pen=pg.mkPen('#ff4444', width=1, style=QtCore.Qt.DashLine))
p_disp.addItem(lim_dx); p_disp.addItem(lim_dy)
p_disp.setXRange(0, 0.3, padding=0)
p_disp.setYRange(0, 8, padding=0)
leg_disp = p_disp.addLegend(offset=(5, 5))
leg_disp.setLabelTextColor('#ffffff')

# ─── Update (chamado pelo QTimer) ─────────────────────────────────
def update():
    global buffer_udp, sinal_raw, sinal_filt, psd_freqs, psd_vals, dwt_coef
    global scroll_rms, scroll_kurt, disp_ok_x, disp_ok_y, disp_al_x, disp_al_y

    # Drena socket — máx 400 pacotes por tick
    for _ in range(400):
        try:
            dados, _ = sock.recvfrom(1024)
            if len(dados) == 2:
                buffer_udp.append(int(np.frombuffer(dados, dtype=np.int16)[0]))
        except BlockingIOError:
            break

    if len(buffer_udp) < N:
        return

    # Processa 1 janela
    raw          = np.array(buffer_udp[:N], dtype=float)
    buffer_udp   = buffer_udp[N//2:]   # sobreposição 50%

    sinal_raw    = raw / SENSIT
    sinal_filt   = aplicar_iir(sinal_raw)

    rms, kurt, cfc = calcular_atributos(sinal_filt)
    psd_freqs, psd_vals = signal.welch(sinal_filt, fs=FS, window='hamming', nperseg=N, noverlap=N//2)
    coefs        = pywt.wavedec(sinal_filt, 'db4', level=4)
    dwt_coef     = coefs[1]

    # Scroll — estratégia do udpplotscroll
    scroll_rms[:-1]  = scroll_rms[1:];  scroll_rms[-1]  = rms
    scroll_kurt[:-1] = scroll_kurt[1:]; scroll_kurt[-1] = kurt

    status, alerta = classificar(rms, kurt)
    ts = time.strftime("%H:%M:%S")
    writer.writerow([ts, f"{rms:.5f}", f"{kurt:.4f}", f"{cfc:.4f}", int(alerta)])
    csv_file.flush()

    if alerta:
        disp_al_x.append(rms); disp_al_y.append(kurt)
    else:
        disp_ok_x.append(rms); disp_ok_y.append(kurt)

    # ── Atualiza curvas ───────────────────────────────────────────
    c_raw.setData(t_ms,    sinal_raw)
    c_filt.setData(t_ms,   sinal_filt)
    c_fft.setData(psd_freqs, np.maximum(psd_vals, 1e-14))
    c_dwt.setData(np.arange(len(dwt_coef)), dwt_coef)
    c_rms.setData(scroll_rms)
    c_kurt.setData(scroll_kurt)

    MAX_DISP = 500
    if disp_ok_x: c_disp_ok.setData(disp_ok_x[-MAX_DISP:], disp_ok_y[-MAX_DISP:])
    if disp_al_x: c_disp_al.setData(disp_al_x[-MAX_DISP:], disp_al_y[-MAX_DISP:])

    cor = "#ff4444" if alerta else "#00d4aa"
    lbl_status.setText(
        f"  [{ts}]  RMS = {rms:.4f} g   |   Curtose = {kurt:.2f}   |   CFC = {cfc:.2f}   |   {status}"
    )
    lbl_status.setStyleSheet(
        f"color: {cor}; background: #1a1a2e; font-family: monospace; font-size: 11px; padding: 4px; font-weight: bold;"
    )

# QTimer a 50ms — mesmo intervalo do udpplotscroll
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

# ─── Encerramento limpo ───────────────────────────────────────────
def encerrar():
    csv_file.close()
    sock.close()
    print("Encerrado. Dados salvos em dados_mpu.csv")

app.aboutToQuit.connect(encerrar)

if __name__ == '__main__':
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        sys.exit(app.exec_())