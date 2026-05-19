# Análise de Vibrações em Máquinas Rotativas para Manutenção Preditiva
**Uma abordagem Edge-Host de Baixo Custo**  
FAINOR — Engenharia de Computação, 2026

---

## Sobre o Projeto

Sistema híbrido edge-host para monitoramento de vibrações em máquinas rotativas, desenvolvido como trabalho da disciplina de Processamento Digital de Sinais. O objetivo é demonstrar a viabilidade técnica e financeira de manutenção preditiva embarcada com hardware de **menos de R$ 200**.

---

## Hardware

| Componente | Função | Custo |
|---|---|---|
| ESP32 Dev Kit V1 | Microcontrolador + Wi-Fi | R$ 64,99 |
| MPU6050 (GY-521) | Sensor de vibração MEMS | R$ 19,99 |
| Ponte H BTS7960 | Controle PWM do motor | R$ 44,99 |
| Motor DC + redução | Atuador rotativo | R$ 12,98 |
| Micro-hélice DC130 | Gerador de desbalanceamento | R$ 3,49 |
| Protoboard + outros | — | R$ 12,88 |
| **TOTAL** | | **R$ 155,83** |

---

## Arquitetura

```
[MPU6050] --I²C--> [ESP32] --UDP/Wi-Fi--> [PC - Python Host]
                     │                          │
                  Filtragem IIR           Análise Espectral
                  2 kHz sampling          Welch PSD / DWT
                                         RMS, Curtose, Crest Factor
                                         Dashboard em tempo real
```

**Edge (ESP32):** aquisição a 2 kHz, filtro IIR Butterworth passa-banda (10–800 Hz), transmissão UDP com pacotes de 22 bytes validados por magic number e CRC-16.

**Host (Python):** recepção UDP, análise espectral (Welch), Transformada Wavelet Discreta (db4), extração de atributos estatísticos, dashboard de 9 painéis atualizado a 2 Hz.

---

## Resultados

Três condições mecânicas foram testadas (60 s cada):

| Condição | RMS (g) | Curtose | Fator de Crista |
|---|---|---|---|
| Saudável | 0,0829 | 3,47 | 4,21 |
| Falha leve | 0,0827 | 5,79 | 5,39 |
| Falha pesada | 0,1266 | 22,65 | 8,23 |

- **Curtose é mais sensível que RMS** na detecção de falhas incipientes (+553% vs +52,7%)
- **100% de integridade** de transmissão em pacotes UDP
- Limiar de alerta: RMS > 0,10 g **ou** curtose > 4,0

---

## Stack

- **Firmware:** C++ / Arduino framework (PlatformIO)
- **Host:** Python 3 — `numpy`, `scipy`, `pywavelets`, `matplotlib`
- **Comunicação:** UDP Wi-Fi, pacotes binários little-endian, 44 kB/s

---

## Como Executar

**1. Firmware (ESP32)**

Edite `src/main.cpp` com o SSID/senha da rede e o IP do host antes de compilar.
Compile o código na ide após baixar as bibliotecas necessárias.

**2. Host (Python)**
```bash
cd python
python recebimentodados.py
```
O dashboard abre automaticamente e exibe os dados em tempo real.

---

## Autores

Camila Ramos · Daniel Soares · Gustavo Mendes · Rauan Brandão  
Orientador: Prof. Wallas Fróes de Oliveira  
FAINOR — Vitória da Conquista, BA — 2026
