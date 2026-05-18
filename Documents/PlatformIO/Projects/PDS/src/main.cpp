#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <MPU6050.h>

// ─── Wi-Fi / UDP ───────────────────────────────────────────────
const char* ssid     = "Redmi Note 14";
const char* password = "camilinha1210";
const char* pc_ip    = "10.167.196.179";
const int   udp_port = 5005;

// ─── Pinos BTS7960 ─────────────────────────────────────────────
// R_EN e L_EN podem ser ligados direto no 3.3 V da placa
// (habilitação permanente). Se quiser controle via software,
// defina pinos e chame digitalWrite(R_EN, HIGH) no setup().
#define RPWM  27   // sentido horário
#define LPWM  32   // sentido anti-horário

// ─── Parâmetros PWM (LEDC) ─────────────────────────────────────
#define PWM_FREQ   20000   // 20 kHz — acima da audição humana
#define PWM_RES    8       // 8 bits → duty de 0 a 255
#define CH_R       0       // canal LEDC para RPWM
#define CH_L       1       // canal LEDC para LPWM

// ─── MPU / UDP ─────────────────────────────────────────────────
WiFiUDP udp;
MPU6050 mpu;

// ─── Controle do motor ─────────────────────────────────────────
// Chame setMotor() a qualquer momento para mudar velocidade/sentido
// speed: 0–255
// dir:   1 = horário, -1 = anti-horário, 0 = freio
void setMotor(int speed, int dir) {
  speed = constrain(speed, 0, 255);
  if (dir == 1) {
    ledcWrite(CH_R, speed);
    ledcWrite(CH_L, 0);
  } else if (dir == -1) {
    ledcWrite(CH_R, 0);
    ledcWrite(CH_L, speed);
  } else {            // freio (ambos ativos ou ambos 0)
    ledcWrite(CH_R, 0);
    ledcWrite(CH_L, 0);
  }
}

// ─── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // LEDC — configura canais PWM
  ledcSetup(CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(RPWM, CH_R);
  ledcAttachPin(LPWM, CH_L);
  setMotor(0, 0);   // garante motor parado na inicialização

  // I²C + MPU6050
  Wire.begin(21, 22);
  mpu.initialize();
  mpu.setFullScaleAccelRange(0);   // ±2g

  // Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  Serial.println("WiFi OK — IP: " + WiFi.localIP().toString());

  // ── Exemplo: liga o motor a 60 % no sentido horário ──────────
  // Troque estes valores ou controle via Serial/UDP conforme necessário
  setMotor(153, 1);   // 153/255 ≈ 60 %
}

// ─── Loop principal ────────────────────────────────────────────
// Mantém a taxa de amostragem de 2 kHz e aceita comandos pelo
// Serial para mudar velocidade/sentido em tempo real.
void loop() {
  // ── Leitura do MPU e envio UDP (2 kHz) ─────────────────────
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  udp.beginPacket(pc_ip, udp_port);
  udp.write((uint8_t*)&az, sizeof(int16_t));
  udp.endPacket();

  // ── Controle via Serial (opcional) ─────────────────────────
  // Formato: "S,<speed>,<dir>\n"  ex: "S,200,1"  ou  "S,0,0"
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("S,")) {
      int c1 = cmd.indexOf(',', 2);
      if (c1 > 2) {
        int spd = cmd.substring(2, c1).toInt();
        int dir = cmd.substring(c1 + 1).toInt();
        setMotor(spd, dir);
        Serial.printf("Motor: speed=%d dir=%d\n", spd, dir);
      }
    }
  }

  delayMicroseconds(500);   // 2 kHz
}