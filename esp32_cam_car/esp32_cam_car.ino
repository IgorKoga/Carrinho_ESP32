/*
 * ESP32-CAM Carrinho Seguidor de Linha Autônomo com Web Server & Interface Stream
 * 
 * Hardware:
 * - ESP32-CAM (Módulo AI-Thinker + OV2640)
 * - Driver Ponte H L298N
 * 
 * Pinagem L298N -> ESP32-CAM:
 * - IN1 -> GPIO 12 (Motor Esquerdo A)
 * - IN2 -> GPIO 13 (Motor Esquerdo B)
 * - IN3 -> GPIO 14 (Motor Direito A)
 * - IN4 -> GPIO 15 (Motor Direito B)
 * 
 * Observação no Arduino IDE:
 * 1. Selecionar Placa: "AI Thinker ESP32-CAM"
 * 2. Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
 * 3. PSRAM: "Enabled" (se disponível no modelo)
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <esp_timer.h>
#include <img_converters.h>
#include <ArduinoJson.h>

// ==========================================
// CONFIGURAÇÃO DE REDE WI-FI
// ==========================================
// Escolha o modo de operação:
// true  = ESP32 cria sua própria rede Wi-Fi (Access Point)
// false = ESP32 conecta a um Roteador existente (Station)
const bool USE_ACCESS_POINT = true;

const char* ssid     = "ESP32_Carrinho_Robo";
const char* password = "password123";

// ==========================================
// CONFIGURAÇÃO DOS PINOS DA CÂMERA (AI-THINKER)
// ==========================================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_GPIO_NUM     4

// ==========================================
// CONFIGURAÇÃO DOS PINOS DA PONTE H & SERVO (Fiação Real)
// ==========================================
#define MOTOR_IN1   14 // IO14 -> IN1
#define MOTOR_IN2   15 // IO15 -> IN2
#define MOTOR_IN3   13 // IO13 -> IN3
#define MOTOR_IN4   12 // IO12 -> IN4

// Pino de Sinal do Servomotor de Direção
#define SERVO_PIN   2

// Canais PWM LEDC do ESP32
#define PWM_CHAN_IN1    0
#define PWM_CHAN_IN2    1
#define PWM_CHAN_IN3    2
#define PWM_CHAN_IN4    3
#define PWM_CHAN_SERVO  4

#define PWM_FREQ        5000
#define PWM_RES         8 // Resolution 0-255

#define SERVO_FREQ      50   // Frequência padrão de Servo (50Hz)
#define SERVO_RES       14   // Resolução 14 bits (0-16383) para controle suave

// Compatibilidade entre ESP32 Arduino Core 2.x e 3.x (LEDC API)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, freq, res, chan) ledcAttach(pin, freq, res)
  #define PWM_WRITE(pin, chan, duty)       ledcWrite(pin, duty)
#else
  #define PWM_ATTACH(pin, freq, res, chan) do { ledcSetup(chan, freq, res); ledcAttachPin(pin, chan); } while(0)
  #define PWM_WRITE(pin, chan, duty)       ledcWrite(chan, duty)
#endif

// ==========================================
// VARIÁVEIS DE CONTROLE, SERVO E PID
// ==========================================
enum Mode { MODE_MANUAL, MODE_AUTO };
Mode currentMode = MODE_AUTO;

// Parâmetros de Velocidade de Tração (0 - 255)
int baseSpeed = 160;
int maxSpeed  = 255;
int minSpeed  = 0;
int currentDriveSpeed = 0;

// Parâmetros do Servomotor de Direção (Ângulos em Graus)
int currentServoAngle = 90;
int servoCenterAngle  = 90;  // Posição central / reto (90°)
int servoMinAngle     = 45;  // Limite máximo de curva à esquerda (graus)
int servoMaxAngle     = 135; // Limite máximo de curva à direita (graus)

// Parâmetros PID para o Seguidor de Linha (Ajusta o Ângulo do Servo)
float Kp = 0.50;
float Ki = 0.00;
float Kd = 0.20;

float errorPrev = 0;
float integral  = 0;

// Processamento de Imagem (Resolução QQVGA 160x120 para FPS máximo)
int thresholdVal = 100;      // Limiar para binarização (0-255)
bool isDarkLine  = true;     // true = linha preta em fundo claro, false = linha clara em fundo escuro
int scanRowY     = 90;       // Linha Y de varredura (da imagem de 120px de altura)
int lineCenterPos = 80;      // Posição encontrada da linha (largura 160px)
bool lineDetected = false;
int lastError     = 0;

// Telemetria e FPS
float fps = 0.0;
unsigned long frameCount = 0;
unsigned long lastFpsTime = 0;

// Servidores HTTP
WebServer server(80);
WiFiServer streamServer(81);

// Forward declarations
void startCamera();
void setupMotors();
void setupServo();
void setServoAngle(int angle);
void setDriveSpeed(int speed);
void processLineFollowing(uint8_t* buf, int width, int height);
void handleStream();

// ==========================================
// SETUP INICIAL
// ==========================================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("\n--- INICIALIZANDO CARRINHO ESP32-CAM ---");

  pinMode(FLASH_GPIO_NUM, OUTPUT);
  digitalWrite(FLASH_GPIO_NUM, LOW); // Flash desligado

  setupMotors();
  setupServo();
  startCamera();

  // Conexão Wi-Fi
  if (USE_ACCESS_POINT) {
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("Modo Access Point ativado. IP: ");
    Serial.println(IP);
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nConectado à rede Wi-Fi! IP: ");
    Serial.println(WiFi.localIP());
  }

  // Rotas HTTP da API REST
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", "<h1>ESP32-CAM Carrinho com Servomotor Ativo!</h1><p>Acesse o servidor web frontend para controlar.</p>");
  });

  // Alterar Modo (AUTO / MANUAL)
  server.on("/api/mode", HTTP_GET, []() {
    if (server.hasArg("val")) {
      String m = server.arg("val");
      if (m == "AUTO") currentMode = MODE_AUTO;
      else if (m == "MANUAL") {
        currentMode = MODE_MANUAL;
        setDriveSpeed(0);
        setServoAngle(servoCenterAngle);
      }
    }
    server.send(200, "application/json", "{\"mode\":\"" + String(currentMode == MODE_AUTO ? "AUTO" : "MANUAL") + "\"}");
  });

  // Controle Manual (FORWARD, BACKWARD, LEFT, RIGHT, STOP)
  server.on("/api/control", HTTP_GET, []() {
    if (currentMode == MODE_MANUAL && server.hasArg("dir")) {
      String dir = server.arg("dir");
      int spd = baseSpeed;
      if (server.hasArg("speed")) spd = server.arg("speed").toInt();

      if (dir == "FORWARD") {
        setDriveSpeed(spd);
        setServoAngle(servoCenterAngle);
      } else if (dir == "BACKWARD") {
        setDriveSpeed(-spd);
        setServoAngle(servoCenterAngle);
      } else if (dir == "LEFT") {
        setDriveSpeed(spd);
        setServoAngle(servoMinAngle);
      } else if (dir == "RIGHT") {
        setDriveSpeed(spd);
        setServoAngle(servoMaxAngle);
      } else {
        setDriveSpeed(0);
      }
    }
    server.send(200, "application/json", "{\"status\":\"ok\"}");
  });

  // Endpoint direto para controlar o Ângulo do Servomotor (ex: /api/servo?angle=90)
  server.on("/api/servo", HTTP_GET, []() {
    if (server.hasArg("angle")) {
      int angle = server.arg("angle").toInt();
      setServoAngle(angle);
    }
    server.send(200, "application/json", "{\"servoAngle\":" + String(currentServoAngle) + "}");
  });

  // Configuração dos Parâmetros PID, Threshold e Limites do Servo
  server.on("/api/settings", HTTP_GET, []() {
    if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
    if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
    if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();
    if (server.hasArg("speed")) baseSpeed = server.arg("speed").toInt();
    if (server.hasArg("thresh")) thresholdVal = server.arg("thresh").toInt();
    if (server.hasArg("dark")) isDarkLine = (server.arg("dark") == "true" || server.arg("dark") == "1");
    if (server.hasArg("servoCenter")) servoCenterAngle = server.arg("servoCenter").toInt();
    if (server.hasArg("servoMin")) servoMinAngle = server.arg("servoMin").toInt();
    if (server.hasArg("servoMax")) servoMaxAngle = server.arg("servoMax").toInt();

    server.send(200, "application/json", "{\"status\":\"updated\"}");
  });

  // Endpoint de Telemetria
  server.on("/api/telemetry", HTTP_GET, []() {
    String json = "{";
    json += "\"mode\":\"" + String(currentMode == MODE_AUTO ? "AUTO" : "MANUAL") + "\",";
    json += "\"fps\":" + String(fps, 1) + ",";
    json += "\"error\":" + String(lastError) + ",";
    json += "\"linePos\":" + String(lineCenterPos) + ",";
    json += "\"camWidth\":160,";
    json += "\"camHeight\":120,";
    json += "\"lineDetected\":" + String(lineDetected ? "true" : "false") + ",";
    json += "\"driveSpeed\":" + String(currentDriveSpeed) + ",";
    json += "\"servoAngle\":" + String(currentServoAngle) + ",";
    json += "\"servoCenter\":" + String(servoCenterAngle) + ",";
    json += "\"servoMin\":" + String(servoMinAngle) + ",";
    json += "\"servoMax\":" + String(servoMaxAngle) + ",";
    json += "\"kp\":" + String(Kp, 2) + ",";
    json += "\"ki\":" + String(Ki, 2) + ",";
    json += "\"kd\":" + String(Kd, 2) + ",";
    json += "\"thresh\":" + String(thresholdVal) + ",";
    json += "\"speed\":" + String(baseSpeed);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.enableCORS();
  server.begin();
  streamServer.begin();
  Serial.println("Servidores HTTP e Stream iniciados nas portas 80 e 81.");
}

// ==========================================
// LOOP PRINCIPAL
// ==========================================
void loop() {
  server.handleClient();
  handleStream();
}

// ==========================================
// CONFIGURAÇÃO DOS MOTORES E SERVO
// ==========================================
void setupMotors() {
  // Configuração dos canais/pinos PWM LEDC para os motores de tração da ponte H L298N (IO14, IO15, IO13, IO12)
  PWM_ATTACH(MOTOR_IN1, PWM_FREQ, PWM_RES, PWM_CHAN_IN1);
  PWM_ATTACH(MOTOR_IN2, PWM_FREQ, PWM_RES, PWM_CHAN_IN2);
  PWM_ATTACH(MOTOR_IN3, PWM_FREQ, PWM_RES, PWM_CHAN_IN3);
  PWM_ATTACH(MOTOR_IN4, PWM_FREQ, PWM_RES, PWM_CHAN_IN4);

  setDriveSpeed(0);
}

void setupServo() {
  // Configuração da saída PWM do Servomotor de Direção (50Hz no IO2)
  PWM_ATTACH(SERVO_PIN, SERVO_FREQ, SERVO_RES, PWM_CHAN_SERVO);
  setServoAngle(servoCenterAngle);
}

void setServoAngle(int angle) {
  angle = constrain(angle, servoMinAngle, servoMaxAngle);
  currentServoAngle = angle;

  // Mapeia o ângulo (0° a 180°) para o pulso PWM de 50Hz (500us a 2500us)
  // Em 14 bits (0 a 16383): 500us = ~410, 2500us = ~2048
  int duty = map(angle, 0, 180, 410, 2048);
  PWM_WRITE(SERVO_PIN, PWM_CHAN_SERVO, duty);
}

void setDriveSpeed(int speed) {
  currentDriveSpeed = constrain(speed, -255, 255);

  if (currentDriveSpeed > 0) {
    // Frente (IN1=14 e IN3=13 com PWM / IN2=15 e IN4=12 em LOW)
    PWM_WRITE(MOTOR_IN1, PWM_CHAN_IN1, currentDriveSpeed);
    PWM_WRITE(MOTOR_IN2, PWM_CHAN_IN2, 0);
    PWM_WRITE(MOTOR_IN3, PWM_CHAN_IN3, currentDriveSpeed);
    PWM_WRITE(MOTOR_IN4, PWM_CHAN_IN4, 0);
  } else if (currentDriveSpeed < 0) {
    // Ré (IN2=15 e IN4=12 com PWM / IN1=14 e IN3=13 em LOW)
    int absSpd = abs(currentDriveSpeed);
    PWM_WRITE(MOTOR_IN1, PWM_CHAN_IN1, 0);
    PWM_WRITE(MOTOR_IN2, PWM_CHAN_IN2, absSpd);
    PWM_WRITE(MOTOR_IN3, PWM_CHAN_IN3, 0);
    PWM_WRITE(MOTOR_IN4, PWM_CHAN_IN4, absSpd);
  } else {
    // Parado
    PWM_WRITE(MOTOR_IN1, PWM_CHAN_IN1, 0);
    PWM_WRITE(MOTOR_IN2, PWM_CHAN_IN2, 0);
    PWM_WRITE(MOTOR_IN3, PWM_CHAN_IN3, 0);
    PWM_WRITE(MOTOR_IN4, PWM_CHAN_IN4, 0);
  }
}

// ==========================================
// CONFIGURAÇÃO DA CÂMERA OV2640
// ==========================================
void startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Resolução QQVGA (160x120) para máxima taxa de quadros (FPS ~30) e processamento ultra-rápido
  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 15; // 0-63 (15 otimiza o payload de transmissão e aumenta a fluidez)
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Falha na inicialização da câmera com erro 0x%x\n", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_vflip(s, 0); // Inverter verticalmente (0 = Normal, 1 = Invertido)
  s->set_hmirror(s, 1);
  Serial.println("Câmera OV2640 Inicializada com Sucesso (QQVGA - Alta Performance)!");
}

// ==========================================
// STREAM DE VÍDEO MJPEG E PROCESSAMENTO DE LINHA
// ==========================================
void handleStream() {
  WiFiClient client = streamServer.available();
  if (!client) return;

  client.println("HTTP/1.1 200 OK");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println();

  while (client.connected()) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Falha na captura do Frame Buffer");
      break;
    }

    // Processamento de visão computacional na imagem para o seguidor de linha
    // Alocação dinâmica com base na dimensão real do frame (QQVGA 160x120)
    int frameW = fb->width;
    int frameH = fb->height;
    uint8_t * rgb_buf = (uint8_t *)malloc(frameW * frameH * 3);
    if (rgb_buf != NULL) {
      if (fmt2rgb888(fb->buf, fb->len, fb->format, rgb_buf)) {
        processLineFollowing(rgb_buf, frameW, frameH);
      }
      free(rgb_buf);
    }

    // Envio do Frame JPEG via MJPEG HTTP Stream
    client.println("--frame");
    client.println("Content-Type: image/jpeg");
    client.printf("Content-Length: %u\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.println();

    esp_camera_fb_return(fb);

    // Cálculo de FPS
    frameCount++;
    if (millis() - lastFpsTime >= 1000) {
      fps = frameCount * 1000.0 / (millis() - lastFpsTime);
      frameCount = 0;
      lastFpsTime = millis();
    }

    server.handleClient(); // Processa chamadas HTTP pendentes
  }
}

// ==========================================
// ALGORITMO SEGUIDOR DE LINHA (VISÃO COMPUTACIONAL & PID)
// ==========================================
void processLineFollowing(uint8_t* rgb_buf, int width, int height) {
  // Amostramos uma faixa horizontal da imagem (scanline Y)
  int rowY = constrain(scanRowY, 0, height - 1);
  long weightedSum = 0;
  long sumPixels = 0;

  for (int x = 0; x < width; x++) {
    int index = (rowY * width + x) * 3;
    uint8_t r = rgb_buf[index];
    uint8_t g = rgb_buf[index + 1];
    uint8_t b = rgb_buf[index + 2];

    // Converter para escala de cinza (Luminância)
    uint8_t gray = (uint8_t)(0.299 * r + 0.587 * g + 0.114 * b);

    // Binarização baseada em Threshold
    bool isMatch = isDarkLine ? (gray < thresholdVal) : (gray > thresholdVal);

    if (isMatch) {
      weightedSum += x;
      sumPixels++;
    }
  }

  if (sumPixels > 5) {
    lineDetected = true;
    lineCenterPos = weightedSum / sumPixels;
  } else {
    lineDetected = false;
    // Se a linha for perdida, mantemos a última direção conhecida com curva acentuada
  }

  // Cálculo do Erro de Desvio (-80 a +80 no QQVGA)
  int targetCenter = width / 2;
  int error = lineCenterPos - targetCenter;
  lastError = error;

  // Se estiver em modo AUTO, calcula o PID e esterça o Servomotor
  if (currentMode == MODE_AUTO) {
    if (!lineDetected) {
      // Procura a linha esterçando para o último sentido conhecido
      if (errorPrev > 0) setServoAngle(servoMaxAngle);
      else setServoAngle(servoMinAngle);
      setDriveSpeed(baseSpeed);
      return;
    }

    // Cálculo do PID para controle do Servomotor
    float P = error;
    integral += error;
    integral = constrain(integral, -1000.0, 1000.0);
    float D = error - errorPrev;
    errorPrev = error;

    float steering = (Kp * P) + (Ki * integral) + (Kd * D);

    // O PID ajusta o ângulo em torno da posição central do servo (servoCenterAngle = 90°)
    int targetServoAngle = servoCenterAngle + (int)steering;
    setServoAngle(targetServoAngle);

    // Mantém a velocidade ajustável definida no motor de tração
    setDriveSpeed(baseSpeed);
  }
}
