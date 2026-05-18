import socket
import numpy as np
from scipy import signal
import pywt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import csv
import time

# ─── Configurações ────────────────────────────────────────────────
UDP_IP   = "0.0.0.0"
UDP_PORT = 5005
FS       = 2000       # Hz
N        = 1024       # amostras por janela
SENSIT   = 16384.0    # MPU6050 ±2g: 16384 LSB/g

# ─── Filtro IIR Butterworth passa-banda 10–800 Hz ─────────────────
# Calculado uma vez, aplicado em cada janela (SOS = segunda ordem)
sos_iir = signal.butter(
    N      = 2,
    Wn     = [10, 800],
    btype  = 'bandpass',
    fs     = FS,
    output = 'sos'
)

# ─── Acumulador e histórico ───────────────────────────────────────
buffer_local = []
HIST         = 60
rms_hist     = deque([0.0]*HIST, maxlen=HIST)
curtose_hist = deque([3.0]*HIST, maxlen=HIST)
janelas_ok   = []
janelas_alerta = []

# ─── CSV ──────────────────────────────────────────────────────────
csv_file = open("dados_mpu.csv", "w", newline="")
writer   = csv.writer(csv_file)
writer.writerow(["timestamp", "rms_g", "curtose", "fator_crista", "alerta"])

# ─── Socket ───────────────────────────────────────────────────────
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)
print(f"Aguardando ESP32 na porta {UDP_PORT}...")

# ─── Funções DSP ──────────────────────────────────────────────────
def aplicar_iir(sinal_g):
    """Butterworth passa-banda IIR 2ª ordem, 10–800 Hz"""
    return signal.sosfilt(sos_iir, sinal_g)

def calcular_atributos(sinal_g):
    rms  = np.sqrt(np.mean(sinal_g**2))
    std  = np.std(sinal_g)
    kurt = np.mean((sinal_g - np.mean(sinal_g))**4) / (std**4) if std > 0 else 3.0
    pico = np.max(np.abs(sinal_g))
    cfc  = pico / rms if rms > 0 else 0.0
    return rms, kurt, cfc

def calcular_welch(sinal_g):
    freqs, psd = signal.welch(
        sinal_g, fs=FS, window='hamming', nperseg=N, noverlap=N//2
    )
    return freqs, psd

def calcular_dwt(sinal_g):
    return pywt.wavedec(sinal_g, 'db4', level=4)

def classificar(rms, kurt):
    if kurt > 4.0:
        return "ALERTA — curtose alta", 1
    if rms > 0.1:   # limiar ajustado para sinal filtrado (sem DC)
        return "ALERTA — vibração alta", 1
    return "OK", 0

# ─── Estado dos gráficos ──────────────────────────────────────────
t_eixo  = np.linspace(0, N/FS*1000, N)
f_dummy = np.linspace(0, FS/2, N//2+1)

ultima_sinal_raw  = np.zeros(N)
ultima_sinal_filt = np.zeros(N)
ultima_freqs      = f_dummy
ultima_psd        = np.ones(len(f_dummy)) * 1e-12
ultimo_coef       = np.zeros(70)

# ─── Layout ───────────────────────────────────────────────────────
plt.style.use('dark_background')
fig = plt.figure(figsize=(15, 8), facecolor='#1a1a2e')
fig.suptitle("Pipeline de Vibrações — MPU6050 + ESP32  |  Filtro IIR Butterworth 10–800 Hz",
             fontsize=12, color='white', y=0.98)

gs = fig.add_gridspec(2, 3, hspace=0.45, wspace=0.35,
                      left=0.06, right=0.97, top=0.92, bottom=0.08)

ax_sinal = fig.add_subplot(gs[0, 0])
ax_fft   = fig.add_subplot(gs[0, 1])
ax_dwt   = fig.add_subplot(gs[0, 2])
ax_rms   = fig.add_subplot(gs[1, 0])
ax_kurt  = fig.add_subplot(gs[1, 1])
ax_disp  = fig.add_subplot(gs[1, 2])

COR_RAW  = '#4a9eff'
COR_FILT = '#00d4aa'
COR_FFT  = '#ff9500'
COR_DWT  = '#bf5fff'
COR_RMS  = '#4a9eff'
COR_KURT = '#ff9500'
COR_OK   = '#00d4aa'
COR_AL   = '#ff4444'
COR_GRID = '#2a2a4a'

def estilizar(ax, titulo, xlabel, ylabel):
    ax.set_facecolor('#0d0d1a')
    ax.set_title(titulo, fontsize=9, color='white', pad=4)
    ax.set_xlabel(xlabel, fontsize=8, color='#aaaacc')
    ax.set_ylabel(ylabel, fontsize=8, color='#aaaacc')
    ax.tick_params(colors='#aaaacc', labelsize=7)
    for spine in ax.spines.values():
        spine.set_edgecolor('#2a2a4a')
    ax.grid(True, color=COR_GRID, linewidth=0.5, alpha=0.7)

# Inicializa as linhas (update por set_data, muito mais rápido)
estilizar(ax_sinal, "Sinal eixo Z — raw vs filtrado IIR", "Tempo (ms)", "Aceleração (g)")
ln_raw,  = ax_sinal.plot(t_eixo, ultima_sinal_raw,  color=COR_RAW,  lw=0.6, alpha=0.4, label='Raw')
ln_filt, = ax_sinal.plot(t_eixo, ultima_sinal_filt, color=COR_FILT, lw=0.8, label='IIR filtrado')
ax_sinal.legend(fontsize=7, loc='upper right',
                facecolor='#1a1a2e', edgecolor='#2a2a4a', labelcolor='white')

estilizar(ax_fft, "PSD — Welch (sinal filtrado)", "Frequência (Hz)", "PSD (g²/Hz)")
ln_fft, = ax_fft.semilogy(ultima_freqs, ultima_psd, color=COR_FFT, lw=0.8)
ax_fft.set_xlim(0, FS/2)
ax_fft.axvspan(0,  10,  alpha=0.15, color='red',  label='< 10 Hz (removido)')
ax_fft.axvspan(800, FS/2, alpha=0.15, color='red', label='> 800 Hz (removido)')
ax_fft.legend(fontsize=6, loc='upper right',
              facecolor='#1a1a2e', edgecolor='#2a2a4a', labelcolor='white')

estilizar(ax_dwt, "DWT db4 — detalhe nível 1", "Amostras", "Coeficiente")
ln_dwt, = ax_dwt.plot(ultimo_coef, color=COR_DWT, lw=0.8)

estilizar(ax_rms, "RMS ao longo do tempo", "Janelas", "RMS (g)")
ln_rms, = ax_rms.plot(list(rms_hist), color=COR_RMS, lw=1.2)
ax_rms.axhline(0.1, color=COR_AL, lw=0.8, ls='--', label='limiar 0.1g')
ax_rms.legend(fontsize=7, facecolor='#1a1a2e', edgecolor='#2a2a4a', labelcolor='white')

estilizar(ax_kurt, "Curtose ao longo do tempo", "Janelas", "Curtose")
ln_kurt, = ax_kurt.plot(list(curtose_hist), color=COR_KURT, lw=1.2)
ax_kurt.axhline(4.0, color=COR_AL,    lw=0.8, ls='--', label='alerta > 4')
ax_kurt.axhline(3.0, color='#888888', lw=0.6, ls=':',  label='normal ≈ 3')
ax_kurt.legend(fontsize=7, facecolor='#1a1a2e', edgecolor='#2a2a4a', labelcolor='white')

estilizar(ax_disp, "Dispersão curtose × RMS", "RMS (g)", "Curtose")
sc_ok, = ax_disp.plot([], [], 'o', color=COR_OK, ms=4, alpha=0.7, label='Saudável')
sc_al, = ax_disp.plot([], [], 'o', color=COR_AL, ms=4, alpha=0.7, label='Alerta')
ax_disp.axhline(4.0, color=COR_AL, lw=0.7, ls='--')
ax_disp.axvline(0.1, color=COR_AL, lw=0.7, ls='--')
ax_disp.legend(fontsize=7, facecolor='#1a1a2e', edgecolor='#2a2a4a', labelcolor='white')

# Label de status no topo
status_txt = fig.text(0.5, 0.005, "Aguardando dados...",
                      ha='center', fontsize=10, color='#aaaacc',
                      fontfamily='monospace')

# ─── Função de atualização ────────────────────────────────────────
def atualizar(frame):
    global buffer_local, ultima_sinal_raw, ultima_sinal_filt
    global ultima_freqs, ultima_psd, ultimo_coef

    # Drena socket
    while True:
        try:
            dados, _ = sock.recvfrom(1024)
            if len(dados) == 2:
                amostra = np.frombuffer(dados, dtype=np.int16)[0]
                buffer_local.append(int(amostra))
        except BlockingIOError:
            break

    # Processa janelas completas
    processou = False
    while len(buffer_local) >= N:
        raw          = np.array(buffer_local[:N], dtype=float)
        buffer_local = buffer_local[N//2:]

        sinal_raw  = raw / SENSIT
        sinal_filt = aplicar_iir(sinal_raw)

        ultima_sinal_raw  = sinal_raw
        ultima_sinal_filt = sinal_filt

        rms, kurt, cfc     = calcular_atributos(sinal_filt)
        ultima_freqs, ultima_psd = calcular_welch(sinal_filt)
        coefs              = calcular_dwt(sinal_filt)
        ultimo_coef        = coefs[1]

        rms_hist.append(rms)
        curtose_hist.append(kurt)

        status, alerta = classificar(rms, kurt)
        ts = time.strftime("%H:%M:%S")
        writer.writerow([ts, f"{rms:.5f}", f"{kurt:.4f}", f"{cfc:.4f}", alerta])
        csv_file.flush()

        if alerta:
            janelas_alerta.append((rms, kurt))
        else:
            janelas_ok.append((rms, kurt))

        cor_status = COR_AL if alerta else COR_OK
        status_txt.set_text(f"[{ts}]  RMS={rms:.4f}g   Curtose={kurt:.2f}   CFC={cfc:.2f}   {status}")
        status_txt.set_color(cor_status)
        processou = True

    if not processou:
        return

    # Atualiza linhas (sem redesenhar eixos)
    ln_raw.set_ydata(ultima_sinal_raw)
    ln_filt.set_ydata(ultima_sinal_filt)
    ax_sinal.relim(); ax_sinal.autoscale_view()

    ln_fft.set_xdata(ultima_freqs)
    ln_fft.set_ydata(np.maximum(ultima_psd, 1e-14))
    ax_fft.relim(); ax_fft.autoscale_view()

    ln_dwt.set_xdata(np.arange(len(ultimo_coef)))
    ln_dwt.set_ydata(ultimo_coef)
    ax_dwt.relim(); ax_dwt.autoscale_view()

    ln_rms.set_ydata(list(rms_hist))
    ax_rms.relim(); ax_rms.autoscale_view()

    ln_kurt.set_ydata(list(curtose_hist))
    ax_kurt.relim(); ax_kurt.autoscale_view()

    if janelas_ok:
        rx, kx = zip(*janelas_ok[-300:])
        sc_ok.set_data(rx, kx)
    if janelas_alerta:
        ra, ka = zip(*janelas_alerta[-300:])
        sc_al.set_data(ra, ka)
    ax_disp.relim(); ax_disp.autoscale_view()

ani = animation.FuncAnimation(fig, atualizar, interval=500, cache_frame_data=False)

try:
    plt.show()
finally:
    csv_file.close()
    sock.close()
    print("Encerrado. Dados salvos em dados_mpu.csv")