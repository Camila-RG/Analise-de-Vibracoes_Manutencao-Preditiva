/*
 * main.cpp — Firmware ESP32 | Análise de Vibrações
 * FAINOR | Processamento Digital de Sinais | 2026
 *
 * Captura o eixo Z do MPU6050 a 2 kHz e transmite cada amostra
 * via UDP para o PC. O motor DC é acionado pelo driver BTS7960
 * com controle PWM via LEDC. Velocidade e sentido podem ser
 * ajustados em tempo real pelo Monitor Serial.
 *
 * Dependências (lib/):
 *   - ElectronicCats/mpu6050
 *   - WiFi.h / WiFiUdp.h (já incluso no core ESP32)
 *
 * Conexões BTS7960:
 *   RPWM → GPIO 27   (sentido horário)
 *   LPWM → GPIO 32   (sentido anti-horário)
 *   R_EN, L_EN → 3.3 V (habilitação permanente)
 *   VCC motor → fonte externa (12 V recomendado)
 *   GND comum com ESP32
 *
 * Conexões MPU6050 (I²C):
 *   SDA → GPIO 21
 *   SCL → GPIO 22
 *   VCC → 3.3 V  |  GND → GND
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <MPU6050.h>

// ── Rede ──────────────────────────────────────────────────────────
static constexpr char WIFI_SSID[]  = "Redmi Note 14";
static constexpr char WIFI_PASS[]  = "camilinha1210";
static constexpr char PC_IP[]      = "10.167.196.136";   // ex.: "192.168.1.100"
static constexpr uint16_t UDP_PORT = 5005;

// ── Pinos BTS7960 ─────────────────────────────────────────────────
static constexpr uint8_t PIN_RPWM = 27;  // horário
static constexpr uint8_t PIN_LPWM = 32;  // anti-horário

// ── PWM (LEDC) ────────────────────────────────────────────────────
static constexpr uint32_t PWM_FREQ  = 20000;  // 20 kHz (inaudível)
static constexpr uint8_t  PWM_RES   = 8;      // 0–255
static constexpr uint8_t  LEDC_CH_R = 0;
static constexpr uint8_t  LEDC_CH_L = 1;

// ── Amostragem ────────────────────────────────────────────────────
static constexpr uint32_t FS_US = 500;  // período de amostragem em µs → 2 kHz

// ── Objetos globais ───────────────────────────────────────────────
WiFiUDP udp;
MPU6050 mpu;

// ══════════════════════════════════════════════════════════════════
//  CONTROLE DO MOTOR
// ══════════════════════════════════════════════════════════════════

/**
 * @brief Ajusta velocidade e sentido do motor.
 * @param speed  Duty cycle: 0–255
 * @param dir    1 = horário | -1 = anti-horário | 0 = freio/parado
 */
void setMotor(int speed, int dir) {
    speed = constrain(speed, 0, 255);

    switch (dir) {
        case 1:   // horário
            ledcWrite(LEDC_CH_R, speed);
            ledcWrite(LEDC_CH_L, 0);
            break;
        case -1:  // anti-horário
            ledcWrite(LEDC_CH_R, 0);
            ledcWrite(LEDC_CH_L, speed);
            break;
        default:  // freio / parado
            ledcWrite(LEDC_CH_R, 0);
            ledcWrite(LEDC_CH_L, 0);
            break;
    }
}

// ══════════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);

    // ── PWM (BTS7960) ─────────────────────────────────────────────
    ledcSetup(LEDC_CH_R, PWM_FREQ, PWM_RES);
    ledcSetup(LEDC_CH_L, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_RPWM, LEDC_CH_R);
    ledcAttachPin(PIN_LPWM, LEDC_CH_L);
    setMotor(0, 0);  // garante motor parado na inicialização
    Serial.println("[Motor] Inicializado — parado.");

    // ── MPU6050 ───────────────────────────────────────────────────
    Wire.begin(21, 22);
    mpu.initialize();
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // ±2 g → 16384 LSB/g
    Serial.println("[MPU6050] Inicializado — faixa ±2 g.");

    // ── Wi-Fi ─────────────────────────────────────────────────────
    Serial.printf("[WiFi] Conectando a '%s'…\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\n[WiFi] Conectado! IP local: %s\n",
                  WiFi.localIP().toString().c_str());

    // ── Inicia UDP ────────────────────────────────────────────────
    udp.begin(UDP_PORT);
    Serial.printf("[UDP] Enviando para %s:%u a 2 kHz\n", PC_IP, UDP_PORT);

    // ── Liga o motor (ajuste speed/dir conforme necessário) ───────
    setMotor(153, 1);  // ~60 % de duty, sentido horário
    Serial.println("[Motor] Ligado a 60 % (horário).");
}

// ══════════════════════════════════════════════════════════════════
//  LOOP PRINCIPAL
// ══════════════════════════════════════════════════════════════════

/*
 * Comandos Serial (formato CSV):
 *   S,<speed>,<dir>   ex.: S,200,1   ou   S,0,0
 *   speed: 0–255  |  dir: 1 (horário), -1 (anti-horário), 0 (parar)
 */
void loop() {
    // ── Captura MPU6050 e envia UDP ───────────────────────────────
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    udp.beginPacket(PC_IP, UDP_PORT);
    udp.write(reinterpret_cast<const uint8_t*>(&az), sizeof(int16_t));
    udp.endPacket();

    // ── Comandos via Serial ───────────────────────────────────────
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd.startsWith("S,")) {
            int sep = cmd.indexOf(',', 2);
            if (sep > 2) {
                int spd = cmd.substring(2, sep).toInt();
                int dir = cmd.substring(sep + 1).toInt();
                setMotor(spd, dir);
                Serial.printf("[Motor] speed=%d  dir=%d\n", spd, dir);
            } else {
                Serial.println("[Erro] Formato esperado: S,<speed>,<dir>");
            }
        }
    }

    // ── Aguarda para manter 2 kHz ────────────────────────────────
    delayMicroseconds(FS_US);
}