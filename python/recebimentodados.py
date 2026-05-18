"""
recebimento.py — Pipeline de Análise de Vibrações | Telemetria Completa
FAINOR | Processamento Digital de Sinais | 2026

Recebe pacotes UDP do ESP32 (22 bytes, pacote TelemetryPacket):
  - Aceleração XYZ (MPU6050, ±2 g)
  - Giroscópio  XYZ (MPU6050, ±250 °/s)
  - Estado do motor: duty cycle (0–255) e sentido (1/-1/0)

O pipeline aplica filtro IIR Butterworth 10–800 Hz sobre o eixo Z
(principal eixo de vibração axial) e calcula RMS, curtose, fator de
crista, PSD via Welch e DWT db4 para classificação do estado da máquina.

O estado do motor é correlacionado com as vibrações em tempo real,
permitindo detectar anomalias vinculadas a condições operacionais específicas.
"""

import socket
import struct
import csv
import time
from collections import deque

import numpy as np
from scipy import signal
import pywt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ══════════════════════════════════════════════════════════════════
#  CONFIGURAÇÕES
# ══════════════════════════════════════════════════════════════════

UDP_IP   = "0.0.0.0"
UDP_PORT = 5005

FS      = 2000      # Hz — taxa de amostragem
N       = 1024      # amostras por janela de análise
OVERLAP = N // 2    # sobreposição 50 %

# Fatores de conversão MPU6050
ACCEL_SCALE = 16384.0   # LSB/g  (faixa ±2 g)
GYRO_SCALE  = 131.0     # LSB/(°/s) (faixa ±250 °/s)

# Limiares de classificação
LIMIAR_RMS  = 0.10   # g
LIMIAR_KURT = 4.0

# Histórico exibido nos gráficos de tendência
HIST_LEN = 60

CSV_PATH = "telemetria_vibracao.csv"

# ── Formato do pacote UDP (22 bytes, little-endian) ───────────────
# magic(H) ts(I) ax(h) ay(h) az(h) gx(h) gy(h) gz(h) duty(B) dir(b) csum(H)
PKT_FORMAT = "<HIhhhhhhBbH"   # Corrigido: exatamente 6 'h's e sem duplicar a linha
PKT_SIZE   = struct.calcsize(PKT_FORMAT)   
PKT_MAGIC  = 0xFA10

assert PKT_SIZE == 22, f"Tamanho esperado 22 bytes, obtido {PKT_SIZE}"

# ══════════════════════════════════════════════════════════════════
#  FILTRO IIR — Butterworth passa-banda 10–800 Hz
# ══════════════════════════════════════════════════════════════════

sos = signal.butter(
    N      = 2,
    Wn     = [10, 800],
    btype  = "bandpass",
    fs     = FS,
    output = "sos",
)

# ══════════════════════════════════════════════════════════════════
#  BUFFERS E ESTADO
# ══════════════════════════════════════════════════════════════════

# Buffers por eixo (acumulam amostras brutas em g ou °/s)
buf_az: list[float] = []   # eixo principal de vibração axial
buf_ax: list[float] = []
buf_ay: list[float] = []
buf_gx: list[float] = []
buf_gy: list[float] = []
buf_gz: list[float] = []

# Histórico de atributos para gráficos de tendência
rms_hist  = deque([0.0] * HIST_LEN, maxlen=HIST_LEN)
kurt_hist = deque([3.0] * HIST_LEN, maxlen=HIST_LEN)

# Histórico do estado do motor (duty % e sentido)
duty_hist = deque([0.0] * HIST_LEN, maxlen=HIST_LEN)

# Dispersão RMS × curtose
pontos_ok:     list[tuple] = []
pontos_alerta: list[tuple] = []

# Pacotes inválidos (checksum ou magic errado)
pkts_total   = 0
pkts_invalidos = 0

# Último estado do motor recebido
motor_duty_pct = 0.0
motor_dir_str  = "—"

# Últimos sinais calculados (para redesenho)
_t_ms      = np.linspace(0, N / FS * 1000, N)
_f_hz      = np.linspace(0, FS / 2, N // 2 + 1)
sig_raw    = np.zeros(N)
sig_filt   = np.zeros(N)
psd_freqs  = _f_hz.copy()
psd_vals   = np.full(len(_f_hz), 1e-12)
dwt_detail = np.zeros(70)

# ══════════════════════════════════════════════════════════════════
#  FUNÇÕES DE DECODIFICAÇÃO E DSP
# ══════════════════════════════════════════════════════════════════

def decodificar_pacote(raw: bytes) -> dict | None:
    """
    Desempacota e valida um pacote UDP de 22 bytes.
    Retorna dict com campos ou None se inválido.
    """
    if len(raw) != PKT_SIZE:
        return None

    magic, ts, ax, ay, az, gx, gy, gz, duty, direction, csum = struct.unpack(PKT_FORMAT, raw)

    if magic != PKT_MAGIC:
        return None

    # Valida checksum (XOR dos primeiros 20 bytes)
    xor_calc = 0
    for b in raw[:20]:
        xor_calc ^= b
    if xor_calc != csum:
        return None

    return {
        "timestamp_ms": ts,
        "accel_x":  ax / ACCEL_SCALE,   # g
        "accel_y":  ay / ACCEL_SCALE,
        "accel_z":  az / ACCEL_SCALE,
        "gyro_x":   gx / GYRO_SCALE,    # °/s
        "gyro_y":   gy / GYRO_SCALE,
        "gyro_z":   gz / GYRO_SCALE,
        "motor_duty": duty,              # 0–255
        "motor_dir":  direction,         # 1 | -1 | 0
    }


def filtrar(sinal_g: np.ndarray) -> np.ndarray:
    return signal.sosfilt(sos, sinal_g)


def calcular_atributos(sinal_g: np.ndarray) -> tuple[float, float, float]:
    rms  = float(np.sqrt(np.mean(sinal_g ** 2)))
    std  = float(np.std(sinal_g))
    kurt = float(
        np.mean((sinal_g - sinal_g.mean()) ** 4) / std ** 4 if std > 0 else 3.0
    )
    pico = float(np.max(np.abs(sinal_g)))
    cfc  = pico / rms if rms > 0 else 0.0
    return rms, kurt, cfc


def calcular_welch(sinal_g: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return signal.welch(sinal_g, fs=FS, window="hamming", nperseg=N, noverlap=OVERLAP)


def calcular_dwt(sinal_g: np.ndarray) -> list[np.ndarray]:
    return pywt.wavedec(sinal_g, "db4", level=4)


def classificar(rms: float, kurt: float) -> tuple[str, bool]:
    """
    Curtose tem prioridade: é mais sensível a falhas incipientes
    (impactos isolados que ainda não elevam o RMS médio).
    """
    if kurt > LIMIAR_KURT:
        return "ALERTA — curtose elevada (impacto/falha incipiente)", True
    if rms > LIMIAR_RMS:
        return "ALERTA — vibração excessiva (RMS alto)", True
    return "OK — máquina saudável", False


def dir_str(d: int) -> str:
    return {1: "horário", -1: "anti-horário", 0: "parado"}.get(d, "?")

# ══════════════════════════════════════════════════════════════════
#  SOCKET E CSV
# ══════════════════════════════════════════════════════════════════

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)
print(f"[UDP] Aguardando pacotes do ESP32 na porta {UDP_PORT} ({PKT_SIZE} bytes)…")

csv_file = open(CSV_PATH, "w", newline="", encoding="utf-8")
writer   = csv.writer(csv_file)
writer.writerow([
    "timestamp_pc", "timestamp_esp_ms",
    "rms_z_g", "curtose_z", "fator_crista_z",
    "accel_x_g", "accel_y_g",
    "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
    "motor_duty_pct", "motor_dir",
    "alerta", "pkts_invalidos",
])

# ══════════════════════════════════════════════════════════════════
#  LAYOUT MATPLOTLIB
# ══════════════════════════════════════════════════════════════════

plt.style.use("dark_background")

PAL = dict(
    raw    = "#4a9eff",
    filt   = "#00d4aa",
    psd    = "#ff9500",
    dwt    = "#bf5fff",
    rms    = "#4a9eff",
    kurt   = "#ff9500",
    duty   = "#ffffff",
    ok     = "#00d4aa",
    alerta = "#ff4444",
    grade  = "#2a2a4a",
    fundo  = "#0d0d1a",
    painel = "#1a1a2e",
)

fig = plt.figure(figsize=(17, 9), facecolor=PAL["painel"])
fig.suptitle(
    "Análise de Vibrações — MPU6050 + ESP32 + BTS7960  |  IIR 10–800 Hz  |  FAINOR 2026",
    fontsize=11, color="white", y=0.99,
)

# Grid 3×3: linha 0 = sinais, linha 1 = tendências, linha 2 = correlação motor
gs = fig.add_gridspec(
    3, 3, hspace=0.55, wspace=0.38,
    left=0.06, right=0.97, top=0.94, bottom=0.07,
)

ax_sinal = fig.add_subplot(gs[0, 0])
ax_psd   = fig.add_subplot(gs[0, 1])
ax_dwt   = fig.add_subplot(gs[0, 2])
ax_rms   = fig.add_subplot(gs[1, 0])
ax_kurt  = fig.add_subplot(gs[1, 1])
ax_duty  = fig.add_subplot(gs[1, 2])
ax_disp  = fig.add_subplot(gs[2, 0])
ax_xyz   = fig.add_subplot(gs[2, 1])
ax_gyro  = fig.add_subplot(gs[2, 2])


def _est(ax, titulo, xlabel, ylabel):
    ax.set_facecolor(PAL["fundo"])
    ax.set_title(titulo, fontsize=8, color="white", pad=3)
    ax.set_xlabel(xlabel, fontsize=7, color="#aaaacc")
    ax.set_ylabel(ylabel, fontsize=7, color="#aaaacc")
    ax.tick_params(colors="#aaaacc", labelsize=6)
    for sp in ax.spines.values():
        sp.set_edgecolor(PAL["grade"])
    ax.grid(True, color=PAL["grade"], lw=0.5, alpha=0.7)


# ── Linha 0: sinal, PSD, DWT ──────────────────────────────────────
_est(ax_sinal, "Sinal Eixo Z — Raw vs Filtrado IIR", "Tempo (ms)", "Acel. (g)")
ln_raw,  = ax_sinal.plot(_t_ms, sig_raw,  color=PAL["raw"],  lw=0.5, alpha=0.4, label="Raw")
ln_filt, = ax_sinal.plot(_t_ms, sig_filt, color=PAL["filt"], lw=0.8, label="IIR filtrado")
ax_sinal.legend(fontsize=6, loc="upper right",
                facecolor=PAL["painel"], edgecolor=PAL["grade"], labelcolor="white")

_est(ax_psd, "PSD — Welch (eixo Z filtrado)", "Frequência (Hz)", "PSD (g²/Hz)")
ln_psd, = ax_psd.semilogy(psd_freqs, psd_vals, color=PAL["psd"], lw=0.8)
ax_psd.set_xlim(0, FS / 2)
ax_psd.axvspan(0,   10,     alpha=0.12, color="red", label="< 10 Hz")
ax_psd.axvspan(800, FS / 2, alpha=0.12, color="red", label="> 800 Hz")
ax_psd.legend(fontsize=6, loc="upper right",
              facecolor=PAL["painel"], edgecolor=PAL["grade"], labelcolor="white")

_est(ax_dwt, "DWT db4 — Detalhe Nível 1 (Z)", "Amostras", "Coef.")
ln_dwt, = ax_dwt.plot(dwt_detail, color=PAL["dwt"], lw=0.8)

# ── Linha 1: RMS, Curtose, Duty Motor ─────────────────────────────
_est(ax_rms, "RMS (eixo Z) por janela", "Janelas", "RMS (g)")
ln_rms, = ax_rms.plot(list(rms_hist), color=PAL["rms"], lw=1.2)
ax_rms.axhline(LIMIAR_RMS, color=PAL["alerta"], lw=0.8, ls="--",
               label=f"Limiar {LIMIAR_RMS} g")
ax_rms.legend(fontsize=6, facecolor=PAL["painel"],
              edgecolor=PAL["grade"], labelcolor="white")

_est(ax_kurt, "Curtose (eixo Z) por janela", "Janelas", "Curtose")
ln_kurt, = ax_kurt.plot(list(kurt_hist), color=PAL["kurt"], lw=1.2)
ax_kurt.axhline(LIMIAR_KURT, color=PAL["alerta"], lw=0.8, ls="--",
                label=f"Alerta > {LIMIAR_KURT}")
ax_kurt.axhline(3.0, color="#888888", lw=0.6, ls=":", label="Normal ≈ 3")
ax_kurt.legend(fontsize=6, facecolor=PAL["painel"],
               edgecolor=PAL["grade"], labelcolor="white")

_est(ax_duty, "Duty Cycle do Motor (%)", "Janelas", "Duty (%)")
ln_duty, = ax_duty.plot(list(duty_hist), color=PAL["duty"], lw=1.2)
ax_duty.set_ylim(-5, 110)

# ── Linha 2: Dispersão, XY accel, Giroscópio ─────────────────────
_est(ax_disp, "Dispersão: RMS × Curtose", "RMS (g)", "Curtose")
sc_ok, = ax_disp.plot([], [], "o", color=PAL["ok"],     ms=3, alpha=0.7, label="Saudável")
sc_al, = ax_disp.plot([], [], "o", color=PAL["alerta"], ms=3, alpha=0.7, label="Alerta")
ax_disp.axhline(LIMIAR_KURT, color=PAL["alerta"], lw=0.7, ls="--")
ax_disp.axvline(LIMIAR_RMS,  color=PAL["alerta"], lw=0.7, ls="--")
ax_disp.legend(fontsize=6, facecolor=PAL["painel"],
               edgecolor=PAL["grade"], labelcolor="white")

_est(ax_xyz, "Aceleração XY — Plano radial", "Acel. X (g)", "Acel. Y (g)")
sc_xy, = ax_xyz.plot([], [], ".", color=PAL["filt"], ms=1.5, alpha=0.5)
ax_xyz.axhline(0, color="#444466", lw=0.5)
ax_xyz.axvline(0, color="#444466", lw=0.5)

_est(ax_gyro, "Giroscópio XYZ", "Janelas", "°/s")
ln_gx, = ax_gyro.plot([], [], color="#ff6b6b", lw=0.8, label="Gyro X")
ln_gy, = ax_gyro.plot([], [], color="#6bffb8", lw=0.8, label="Gyro Y")
ln_gz, = ax_gyro.plot([], [], color="#6bb8ff", lw=0.8, label="Gyro Z")
ax_gyro.legend(fontsize=6, facecolor=PAL["painel"],
               edgecolor=PAL["grade"], labelcolor="white")

# Buffers curtos para gráfico XY e giroscópio
xy_ax_buf = deque(maxlen=N)
xy_ay_buf = deque(maxlen=N)
gx_hist   = deque([0.0] * HIST_LEN, maxlen=HIST_LEN)
gy_hist   = deque([0.0] * HIST_LEN, maxlen=HIST_LEN)
gz_hist   = deque([0.0] * HIST_LEN, maxlen=HIST_LEN)

# Status inferior
txt_status = fig.text(
    0.5, 0.005,
    "Aguardando pacotes do ESP32…",
    ha="center", fontsize=9, color="#aaaacc", fontfamily="monospace",
)
txt_motor = fig.text(
    0.5, 0.025,
    "Motor: —",
    ha="center", fontsize=9, color="#ffffff", fontfamily="monospace",
)

# ══════════════════════════════════════════════════════════════════
#  CALLBACK DE ANIMAÇÃO
# ══════════════════════════════════════════════════════════════════

def atualizar(_frame):
    global sig_raw, sig_filt, psd_freqs, psd_vals, dwt_detail
    global pkts_total, pkts_invalidos
    global motor_duty_pct, motor_dir_str

    # ── 1. Drena socket ───────────────────────────────────────────
    while True:
        try:
            raw, _ = sock.recvfrom(64)
            pkts_total += 1
            pkt = decodificar_pacote(raw)
            if pkt is None:
                pkts_invalidos += 1
                continue

            # Converte duty para %
            d_pct = pkt["motor_duty"] / 255.0 * 100.0
            motor_duty_pct = d_pct
            motor_dir_str  = dir_str(pkt["motor_dir"])

            # Acumula amostras por eixo
            buf_az.append(pkt["accel_z"])
            buf_ax.append(pkt["accel_x"])
            buf_ay.append(pkt["accel_y"])
            buf_gx.append(pkt["gyro_x"])
            buf_gy.append(pkt["gyro_y"])
            buf_gz.append(pkt["gyro_z"])
            xy_ax_buf.append(pkt["accel_x"])
            xy_ay_buf.append(pkt["accel_y"])

        except BlockingIOError:
            break

    # ── 2. Processa janelas completas (eixo Z principal) ──────────
    processou = False
    last_rms = last_kurt = last_cfc = 0.0
    last_ts_esp = 0
    last_pkt_duty = 0
    last_pkt_dir  = 0

    while len(buf_az) >= N:
        janela_z = np.array(buf_az[:N], dtype=float)
        # remove OVERLAP apenas de buf_az (o buffer de controle)
        del buf_az[:OVERLAP]
        # os outros buffers: remove na mesma proporção só se tiverem amostras suficientes
        for buf in [buf_ax, buf_ay, buf_gx, buf_gy, buf_gz]:
            if len(buf) >= OVERLAP:
                del buf[:OVERLAP]

        filtrado = filtrar(janela_z)
        rms, kurt, cfc = calcular_atributos(filtrado)
        freqs, psd     = calcular_welch(filtrado)
        coefs          = calcular_dwt(filtrado)

        rms_hist.append(rms)
        kurt_hist.append(kurt)
        duty_hist.append(motor_duty_pct)

        # Giroscópio — valor médio da janela (representativo)
        janela_gx = np.array(buf_gx[:min(N, len(buf_gx))], dtype=float) if buf_gx else np.zeros(1)
        janela_gy = np.array(buf_gy[:min(N, len(buf_gy))], dtype=float) if buf_gy else np.zeros(1)
        janela_gz = np.array(buf_gz[:min(N, len(buf_gz))], dtype=float) if buf_gz else np.zeros(1)

        gx_hist.append(float(np.mean(janela_gx)))
        gy_hist.append(float(np.mean(janela_gy)))
        gz_hist.append(float(np.mean(janela_gz)))

        status, alerta = classificar(rms, kurt)
        ts_pc = time.strftime("%H:%M:%S")

        writer.writerow([
            ts_pc, pkt["timestamp_ms"],
            f"{rms:.5f}", f"{kurt:.4f}", f"{cfc:.4f}",
            f"{np.mean(buf_ax[:min(N,len(buf_ax))] or [0]):.4f}",
            f"{np.mean(buf_ay[:min(N,len(buf_ay))] or [0]):.4f}",
            f"{gx_hist[-1]:.3f}", f"{gy_hist[-1]:.3f}", f"{gz_hist[-1]:.3f}",
            f"{motor_duty_pct:.1f}", motor_dir_str,
            int(alerta), pkts_invalidos,
        ])
        csv_file.flush()

        (pontos_alerta if alerta else pontos_ok).append((rms, kurt))

        txt_status.set_text(
            f"[{ts_pc}]  RMS={rms:.4f} g   Curtose={kurt:.2f}"
            f"   CFC={cfc:.2f}   {status}"
            f"   | Pkts inv.: {pkts_invalidos}/{pkts_total}"
        )
        txt_status.set_color(PAL["alerta"] if alerta else PAL["ok"])
        txt_motor.set_text(
            f"Motor: {motor_duty_pct:.1f}%  |  Sentido: {motor_dir_str}"
        )

        sig_raw    = janela_z
        sig_filt   = filtrado
        psd_freqs  = freqs
        psd_vals   = psd
        dwt_detail = coefs[1]
        processou  = True

    if not processou:
        return

    # ── 3. Atualiza artistas ──────────────────────────────────────

    # Sinal Z
    ln_raw.set_ydata(sig_raw)
    ln_filt.set_ydata(sig_filt)
    ax_sinal.relim(); ax_sinal.autoscale_view()

    # PSD
    ln_psd.set_xdata(psd_freqs)
    ln_psd.set_ydata(np.maximum(psd_vals, 1e-14))
    ax_psd.relim(); ax_psd.autoscale_view()

    # DWT
    ln_dwt.set_xdata(np.arange(len(dwt_detail)))
    ln_dwt.set_ydata(dwt_detail)
    ax_dwt.relim(); ax_dwt.autoscale_view()

    # Tendências
    ln_rms.set_ydata(list(rms_hist))
    ax_rms.relim(); ax_rms.autoscale_view()

    ln_kurt.set_ydata(list(kurt_hist))
    ax_kurt.relim(); ax_kurt.autoscale_view()

    ln_duty.set_ydata(list(duty_hist))
    ax_duty.relim(); ax_duty.autoscale_view()

    # Dispersão
    if pontos_ok:
        rx, kx = zip(*pontos_ok[-300:])
        sc_ok.set_data(rx, kx)
    if pontos_alerta:
        ra, ka = zip(*pontos_alerta[-300:])
        sc_al.set_data(ra, ka)
    ax_disp.relim(); ax_disp.autoscale_view()

    # XY radial
    if xy_ax_buf:
        sc_xy.set_data(list(xy_ax_buf), list(xy_ay_buf))
        ax_xyz.relim(); ax_xyz.autoscale_view()

    # Giroscópio
    x_idx = list(range(len(gx_hist)))
    ln_gx.set_data(x_idx, list(gx_hist))
    ln_gy.set_data(x_idx, list(gy_hist))
    ln_gz.set_data(x_idx, list(gz_hist))
    ax_gyro.set_xlim(0, HIST_LEN)
    ax_gyro.relim(); ax_gyro.autoscale_view()


# ══════════════════════════════════════════════════════════════════
#  ENTRADA PRINCIPAL
# ══════════════════════════════════════════════════════════════════

ani = animation.FuncAnimation(fig, atualizar, interval=500, cache_frame_data=False)

try:
    plt.show()
finally:
    csv_file.close()
    sock.close()
    print(f"[INFO] Encerrado. Total de pacotes: {pkts_total} | Inválidos: {pkts_invalidos}")
    print(f"[INFO] Dados salvos em '{CSV_PATH}'.")