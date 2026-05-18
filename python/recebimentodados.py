"""
recebimento.py — Pipeline de Análise de Vibrações

Recebe amostras UDP do ESP32 (MPU6050, eixo Z, 2 kHz),
aplica filtro IIR Butterworth 10–800 Hz, calcula atributos
estatísticos (RMS, curtose, fator de crista), PSD via Welch,
DWT db4 e classifica o estado da máquina em tempo real.
"""

import socket
import csv
import time
from collections import deque

import numpy as np
from scipy import signal
import pywt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ══════════════════════════════════════════════════════════════════
#  CONFIGURAÇÕES GERAIS
# ══════════════════════════════════════════════════════════════════

UDP_IP   = "0.0.0.0"
UDP_PORT = 5005

FS       = 2000     # Taxa de amostragem (Hz)
N        = 1024     # Amostras por janela de análise
OVERLAP  = N // 2   # Sobreposição entre janelas (50 %)
SENSIT   = 16384.0  # MPU6050 em ±2 g → 16384 LSB/g

LIMIAR_RMS   = 0.10  # g — acima disto → vibração excessiva
LIMIAR_KURT  = 4.0   # acima disto → impactos/falha incipiente

HIST_LEN = 60        # Janelas exibidas nos gráficos de histórico

CSV_PATH = "dados_vibracao.csv"

# ══════════════════════════════════════════════════════════════════
#  FILTRO IIR — Butterworth passa-banda 10–800 Hz (2ª ordem, SOS)
# ══════════════════════════════════════════════════════════════════

sos = signal.butter(
    N      = 2,
    Wn     = [10, 800],
    btype  = "bandpass",
    fs     = FS,
    output = "sos",
)

# ══════════════════════════════════════════════════════════════════
#  ESTADO DO PIPELINE
# ══════════════════════════════════════════════════════════════════

amostras_brutas: list[int] = []

rms_hist  = deque([0.0] * HIST_LEN, maxlen=HIST_LEN)
kurt_hist = deque([3.0] * HIST_LEN, maxlen=HIST_LEN)

pontos_ok:     list[tuple[float, float]] = []
pontos_alerta: list[tuple[float, float]] = []

# Últimos sinais processados (inicializados com zeros/ruído mínimo)
_t_ms      = np.linspace(0, N / FS * 1000, N)
_f_hz      = np.linspace(0, FS / 2, N // 2 + 1)
sig_raw    = np.zeros(N)
sig_filt   = np.zeros(N)
psd_freqs  = _f_hz.copy()
psd_vals   = np.full(len(_f_hz), 1e-12)
dwt_detail = np.zeros(70)   # coeficientes de detalhe nível 1 (db4)

# ══════════════════════════════════════════════════════════════════
#  FUNÇÕES DSP
# ══════════════════════════════════════════════════════════════════

def filtrar(sinal_g: np.ndarray) -> np.ndarray:
    """Aplica o filtro IIR Butterworth passa-banda."""
    return signal.sosfilt(sos, sinal_g)


def calcular_atributos(sinal_g: np.ndarray) -> tuple[float, float, float]:
    """Retorna (RMS, curtose, fator de crista) do sinal."""
    rms  = float(np.sqrt(np.mean(sinal_g ** 2)))
    std  = float(np.std(sinal_g))
    kurt = float(
        np.mean((sinal_g - sinal_g.mean()) ** 4) / std ** 4
        if std > 0 else 3.0
    )
    pico = float(np.max(np.abs(sinal_g)))
    cfc  = pico / rms if rms > 0 else 0.0
    return rms, kurt, cfc


def calcular_welch(sinal_g: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """PSD via método de Welch com janela Hamming e sobreposição de 50 %."""
    freqs, psd = signal.welch(
        sinal_g,
        fs       = FS,
        window   = "hamming",
        nperseg  = N,
        noverlap = OVERLAP,
    )
    return freqs, psd


def calcular_dwt(sinal_g: np.ndarray) -> list[np.ndarray]:
    """Decomposição Wavelet Discreta com wavelet db4 em 4 níveis."""
    return pywt.wavedec(sinal_g, "db4", level=4)


def classificar(rms: float, kurt: float) -> tuple[str, bool]:
    """
    Retorna (mensagem, alerta).
    Prioriza curtose sobre RMS, pois é mais sensível a falhas incipientes.
    """
    if kurt > LIMIAR_KURT:
        return "ALERTA — curtose elevada (impacto/falha incipiente)", True
    if rms > LIMIAR_RMS:
        return "ALERTA — vibração excessiva (RMS alto)", True
    return "OK — máquina saudável", False

# ══════════════════════════════════════════════════════════════════
#  SOCKET UDP
# ══════════════════════════════════════════════════════════════════

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)
print(f"[UDP] Aguardando dados do ESP32 na porta {UDP_PORT}…")

# ══════════════════════════════════════════════════════════════════
#  CSV
# ══════════════════════════════════════════════════════════════════

csv_file = open(CSV_PATH, "w", newline="", encoding="utf-8")
writer   = csv.writer(csv_file)
writer.writerow(["timestamp", "rms_g", "curtose", "fator_crista", "alerta"])

# ══════════════════════════════════════════════════════════════════
#  LAYOUT MATPLOTLIB
# ══════════════════════════════════════════════════════════════════

plt.style.use("dark_background")

PALETA = dict(
    raw    = "#4a9eff",
    filt   = "#00d4aa",
    psd    = "#ff9500",
    dwt    = "#bf5fff",
    rms    = "#4a9eff",
    kurt   = "#ff9500",
    ok     = "#00d4aa",
    alerta = "#ff4444",
    grade  = "#2a2a4a",
    fundo  = "#0d0d1a",
    painel = "#1a1a2e",
)

fig = plt.figure(figsize=(15, 8), facecolor=PALETA["painel"])
fig.suptitle(
    "Análise de Vibrações — MPU6050 + ESP32  |  IIR Butterworth 10–800 Hz  |  FAINOR 2026",
    fontsize=11, color="white", y=0.98,
)

gs = fig.add_gridspec(
    2, 3, hspace=0.45, wspace=0.35,
    left=0.06, right=0.97, top=0.92, bottom=0.08,
)

ax_sinal = fig.add_subplot(gs[0, 0])
ax_psd   = fig.add_subplot(gs[0, 1])
ax_dwt   = fig.add_subplot(gs[0, 2])
ax_rms   = fig.add_subplot(gs[1, 0])
ax_kurt  = fig.add_subplot(gs[1, 1])
ax_disp  = fig.add_subplot(gs[1, 2])


def _estilizar(ax, titulo: str, xlabel: str, ylabel: str) -> None:
    ax.set_facecolor(PALETA["fundo"])
    ax.set_title(titulo, fontsize=9, color="white", pad=4)
    ax.set_xlabel(xlabel, fontsize=8, color="#aaaacc")
    ax.set_ylabel(ylabel, fontsize=8, color="#aaaacc")
    ax.tick_params(colors="#aaaacc", labelsize=7)
    for spine in ax.spines.values():
        spine.set_edgecolor(PALETA["grade"])
    ax.grid(True, color=PALETA["grade"], linewidth=0.5, alpha=0.7)


_estilizar(ax_sinal, "Sinal eixo Z — Raw vs Filtrado IIR", "Tempo (ms)", "Aceleração (g)")
ln_raw,  = ax_sinal.plot(_t_ms, sig_raw,  color=PALETA["raw"],  lw=0.6, alpha=0.4, label="Raw")
ln_filt, = ax_sinal.plot(_t_ms, sig_filt, color=PALETA["filt"], lw=0.8, label="IIR filtrado")
ax_sinal.legend(fontsize=7, loc="upper right",
                facecolor=PALETA["painel"], edgecolor=PALETA["grade"], labelcolor="white")

_estilizar(ax_psd, "PSD — Welch (sinal filtrado)", "Frequência (Hz)", "PSD (g²/Hz)")
ln_psd, = ax_psd.semilogy(psd_freqs, psd_vals, color=PALETA["psd"], lw=0.8)
ax_psd.set_xlim(0, FS / 2)
ax_psd.axvspan(0,   10,      alpha=0.15, color="red", label="< 10 Hz removido")
ax_psd.axvspan(800, FS / 2,  alpha=0.15, color="red", label="> 800 Hz removido")
ax_psd.legend(fontsize=6, loc="upper right",
              facecolor=PALETA["painel"], edgecolor=PALETA["grade"], labelcolor="white")

_estilizar(ax_dwt, "DWT db4 — Detalhe nível 1", "Amostras", "Coeficiente")
ln_dwt, = ax_dwt.plot(dwt_detail, color=PALETA["dwt"], lw=0.8)

_estilizar(ax_rms, "RMS por janela", "Janelas", "RMS (g)")
ln_rms, = ax_rms.plot(list(rms_hist), color=PALETA["rms"], lw=1.2)
ax_rms.axhline(LIMIAR_RMS, color=PALETA["alerta"], lw=0.8, ls="--",
               label=f"Limiar {LIMIAR_RMS} g")
ax_rms.legend(fontsize=7, facecolor=PALETA["painel"],
              edgecolor=PALETA["grade"], labelcolor="white")

_estilizar(ax_kurt, "Curtose por janela", "Janelas", "Curtose")
ln_kurt, = ax_kurt.plot(list(kurt_hist), color=PALETA["kurt"], lw=1.2)
ax_kurt.axhline(LIMIAR_KURT, color=PALETA["alerta"],  lw=0.8, ls="--",
                label=f"Alerta > {LIMIAR_KURT}")
ax_kurt.axhline(3.0,          color="#888888", lw=0.6, ls=":",
                label="Normal ≈ 3")
ax_kurt.legend(fontsize=7, facecolor=PALETA["painel"],
               edgecolor=PALETA["grade"], labelcolor="white")

_estilizar(ax_disp, "Dispersão: RMS × Curtose", "RMS (g)", "Curtose")
sc_ok, = ax_disp.plot([], [], "o", color=PALETA["ok"],     ms=4, alpha=0.7, label="Saudável")
sc_al, = ax_disp.plot([], [], "o", color=PALETA["alerta"], ms=4, alpha=0.7, label="Alerta")
ax_disp.axhline(LIMIAR_KURT, color=PALETA["alerta"], lw=0.7, ls="--")
ax_disp.axvline(LIMIAR_RMS,  color=PALETA["alerta"], lw=0.7, ls="--")
ax_disp.legend(fontsize=7, facecolor=PALETA["painel"],
               edgecolor=PALETA["grade"], labelcolor="white")

txt_status = fig.text(
    0.5, 0.005, "Aguardando dados do ESP32…",
    ha="center", fontsize=10, color="#aaaacc", fontfamily="monospace",
)

# ══════════════════════════════════════════════════════════════════
#  CALLBACK DE ANIMAÇÃO
# ══════════════════════════════════════════════════════════════════

def atualizar(_frame):
    global amostras_brutas, sig_raw, sig_filt, psd_freqs, psd_vals, dwt_detail

    # 1. Drena o socket sem bloquear
    while True:
        try:
            dados, _ = sock.recvfrom(1024)
            if len(dados) == 2:
                valor = int(np.frombuffer(dados, dtype=np.int16)[0])
                amostras_brutas.append(valor)
        except BlockingIOError:
            break

    # 2. Processa todas as janelas disponíveis (com sobreposição de 50 %)
    processou = False
    while len(amostras_brutas) >= N:
        janela_raw     = np.array(amostras_brutas[:N], dtype=float)
        amostras_brutas = amostras_brutas[OVERLAP:]   # avança meia janela

        sinal_g  = janela_raw / SENSIT
        filtrado = filtrar(sinal_g)

        rms, kurt, cfc = calcular_atributos(filtrado)
        freqs, psd     = calcular_welch(filtrado)
        coefs          = calcular_dwt(filtrado)

        rms_hist.append(rms)
        kurt_hist.append(kurt)

        status, alerta = classificar(rms, kurt)
        ts = time.strftime("%H:%M:%S")
        writer.writerow([ts, f"{rms:.5f}", f"{kurt:.4f}", f"{cfc:.4f}", int(alerta)])
        csv_file.flush()

        (pontos_alerta if alerta else pontos_ok).append((rms, kurt))

        txt_status.set_text(
            f"[{ts}]  RMS={rms:.4f} g   Curtose={kurt:.2f}"
            f"   CFC={cfc:.2f}   {status}"
        )
        txt_status.set_color(PALETA["alerta"] if alerta else PALETA["ok"])

        # Guarda para redesenho
        sig_raw, sig_filt = sinal_g, filtrado
        psd_freqs, psd_vals = freqs, psd
        dwt_detail = coefs[1]   # detalhe de maior frequência
        processou = True

    if not processou:
        return

    # 3. Atualiza os artistas (sem recriar objetos)
    ln_raw.set_ydata(sig_raw)
    ln_filt.set_ydata(sig_filt)
    ax_sinal.relim(); ax_sinal.autoscale_view()

    ln_psd.set_xdata(psd_freqs)
    ln_psd.set_ydata(np.maximum(psd_vals, 1e-14))
    ax_psd.relim(); ax_psd.autoscale_view()

    ln_dwt.set_xdata(np.arange(len(dwt_detail)))
    ln_dwt.set_ydata(dwt_detail)
    ax_dwt.relim(); ax_dwt.autoscale_view()

    ln_rms.set_ydata(list(rms_hist))
    ax_rms.relim(); ax_rms.autoscale_view()

    ln_kurt.set_ydata(list(kurt_hist))
    ax_kurt.relim(); ax_kurt.autoscale_view()

    # Limita a 300 pontos no scatter para não sobrecarregar
    if pontos_ok:
        rx, kx = zip(*pontos_ok[-300:])
        sc_ok.set_data(rx, kx)
    if pontos_alerta:
        ra, ka = zip(*pontos_alerta[-300:])
        sc_al.set_data(ra, ka)
    ax_disp.relim(); ax_disp.autoscale_view()


# ══════════════════════════════════════════════════════════════════
#  ENTRADA PRINCIPAL
# ══════════════════════════════════════════════════════════════════

ani = animation.FuncAnimation(fig, atualizar, interval=500, cache_frame_data=False)

try:
    plt.show()
finally:
    csv_file.close()
    sock.close()
    print(f"[INFO] Encerrado. Dados salvos em '{CSV_PATH}'.")