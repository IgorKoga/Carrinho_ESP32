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
// CONFIGURAÇÃO DOS PINOS DA PONTE H (L298N)
// ==========================================
#define MOTOR_LEFT_IN1   12
#define MOTOR_LEFT_IN2   13
#define MOTOR_RIGHT_IN3  14
#define MOTOR_RIGHT_IN4  15

// Canais PWM LEDC do ESP32
#define PWM_CHAN_IN1  0
#define PWM_CHAN_IN2  1
#define PWM_CHAN_IN3  2
#define PWM_CHAN_IN4  3

#define PWM_FREQ      5000
#define PWM_RES       8 // Resolution 0-255

// ==========================================
// VARIÁVEIS DE CONTROLE E PID
// ==========================================
enum Mode { MODE_MANUAL, MODE_AUTO };
Mode currentMode = MODE_AUTO;

// Parâmetros de Velocidade (0 - 255)
int baseSpeed = 160;
int maxSpeed  = 230;
int minSpeed  = 0;

// Parâmetros PID para o Seguidor de Linha
float Kp = 1.25;
float Ki = 0.00;
float Kd = 0.45;

float errorPrev = 0;
float integral  = 0;

// Processamento de Imagem
int thresholdVal = 100;      // Limiar para binarização (0-255)
bool isDarkLine  = true;     // true = linha preta em fundo claro, false = linha clara em fundo escuro
int scanRowY     = 180;      // Linha Y de varredura (da imagem de 240px de altura)
int lineCenterPos = 160;     // Posição encontrada da linha (largura 320px)
bool lineDetected = false;
int lastError     = 0;

// Telemetria e FPS
float fps = 0.0;
unsigned long frameCount = 0;
unsigned long lastFpsTime = 0;
int motorLeftSpeed = 0;
int motorRightSpeed = 0;

// Servidores HTTP
WebServer server(80);
WiFiServer streamServer(81);

// Forward declarations
void startCamera();
void setupMotors();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
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
    server.send(200, "text/html", "<h1>ESP32-CAM Carrinho Seguidor de Linha Ativo!</h1><p>Acesse o servidor web frontend para controlar.</p>");
  });

  // Alterar Modo (AUTO / MANUAL)
  server.on("/api/mode", HTTP_GET, []() {
    if (server.hasArg("val")) {
      String m = server.arg("val");
      if (m == "AUTO") currentMode = MODE_AUTO;
      else if (m == "MANUAL") {
        currentMode = MODE_MANUAL;
        setMotorSpeeds(0, 0);
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

      if (dir == "FORWARD")       setMotorSpeeds(spd, spd);
      else if (dir == "BACKWARD") setMotorSpeeds(-spd, -spd);
      else if (dir == "LEFT")     setMotorSpeeds(-spd, spd);
      else if (dir == "RIGHT")    setMotorSpeeds(spd, -spd);
      else                        setMotorSpeeds(0, 0);
    }
    server.send(200, "application/json", "{\"status\":\"ok\"}");
  });

  // Configuração dos Parâmetros PID e Threshold
  server.on("/api/settings", HTTP_GET, []() {
    if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
    if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
    if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();
    if (server.hasArg("speed")) baseSpeed = server.arg("speed").toInt();
    if (server.hasArg("thresh")) thresholdVal = server.arg("thresh").toInt();
    if (server.hasArg("dark")) isDarkLine = (server.arg("dark") == "true" || server.arg("dark") == "1");

    server.send(200, "application/json", "{\"status\":\"updated\"}");
  });

  // Endpoint de Telemetria
  server.on("/api/telemetry", HTTP_GET, []() {
    String json = "{";
    json += "\"mode\":\"" + String(currentMode == MODE_AUTO ? "AUTO" : "MANUAL") + "\",";
    json += "\"fps\":" + String(fps, 1) + ",";
    json += "\"error\":" + String(lastError) + ",";
    json += "\"linePos\":" + String(lineCenterPos) + ",";
    json += "\"lineDetected\":" + String(lineDetected ? "true" : "false") + ",";
    json += "\"leftSpeed\":" + String(motorLeftSpeed) + ",";
    json += "\"rightSpeed\":" + String(motorRightSpeed) + ",";
    json += "\"kp\":" + String(Kp, 2) + ",";
    json += "\"ki\":" + String(Ki, 2) + ",";
    json += "\"kd\":" + String(Kd, 2) + ",";
    json += "\"thresh\":" + String(thresholdVal) + ",";
    json += "\"speed\":" + String(baseSpeed);
    json += "}";
    server.send(200, "application/json", json);
  });

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
// CONFIGURAÇÃO DOS MOTORES (L298N)
// ==========================================
void setupMotors() {
  // Configuração dos canais PWM LEDC para controle refinado da ponte H L298N
  ledcSetup(PWM_CHAN_IN1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHAN_IN2, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHAN_IN3, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHAN_IN4, PWM_FREQ, PWM_RES);

  ledcAttachPin(MOTOR_LEFT_IN1, PWM_CHAN_IN1);
  ledcAttachPin(MOTOR_LEFT_IN2, PWM_CHAN_IN2);
  ledcAttachPin(MOTOR_RIGHT_IN3, PWM_CHAN_IN3);
  ledcAttachPin(MOTOR_RIGHT_IN4, PWM_CHAN_IN4);

  setMotorSpeeds(0, 0);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  motorLeftSpeed = constrain(leftSpeed, -255, 255);
  motorRightSpeed = constrain(rightSpeed, -255, 255);

  // Motor Esquerdo
  if (motorLeftSpeed > 0) {
    ledcWrite(PWM_CHAN_IN1, motorLeftSpeed);
    ledcWrite(PWM_CHAN_IN2, 0);
  } else if (motorLeftSpeed < 0) {
    ledcWrite(PWM_CHAN_IN1, 0);
    ledcWrite(PWM_CHAN_IN2, abs(motorLeftSpeed));
  } else {
    ledcWrite(PWM_CHAN_IN1, 0);
    ledcWrite(PWM_CHAN_IN2, 0);
  }

  // Motor Direito
  if (motorRightSpeed > 0) {
    ledcWrite(PWM_CHAN_IN3, motorRightSpeed);
    ledcWrite(PWM_CHAN_IN4, 0);
  } else if (motorRightSpeed < 0) {
    ledcWrite(PWM_CHAN_IN3, 0);
    ledcWrite(PWM_CHAN_IN4, abs(motorRightSpeed));
  } else {
    ledcWrite(PWM_CHAN_IN3, 0);
    ledcWrite(PWM_CHAN_IN4, 0);
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

  // Resolução QVGA (320x240) é a ideal para aliar bom FPS e precisão na detecção
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12; // 0-63 (menor = melhor qualidade)
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Falha na inicialização da câmera com erro 0x%x\n", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_vflip(s, 1); // Inverter verticalmente se necessário
  s->set_hmirror(s, 0);
  Serial.println("Câmera OV2640 Inicializada com Sucesso!");
}

// ==========================================
// STREAM DE VÍDEO MJPEG E PROCESSAMENTO DE LINHA
// ==========================================
void handleStream() {
  WiFiClient client = streamServer.available();
  if (!client) return;

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println();

  while (client.connected()) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Falha na captura do Frame Buffer");
      break;
    }

    // Processamento de visão computacional na imagem para o seguidor de linha
    // Convertemos a imagem para escala de cinza de forma pontual para análise da linha de varredura
    uint8_t * rgb_buf = (uint8_t *)malloc(320 * 240 * 3);
    if (rgb_buf != NULL) {
      if (fmt2rgb888(fb->buf, fb->len, fb->format, rgb_buf)) {
        processLineFollowing(rgb_buf, 320, 240);
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

  // Cálculo do Erro de Desvio (-160 a +160)
  int targetCenter = width / 2; // 160px
  int error = lineCenterPos - targetCenter;
  lastError = error;

  // Se estiver em modo AUTO, calcula e aplica o controle PID nos motores
  if (currentMode == MODE_AUTO) {
    if (!lineDetected) {
      // Procura a linha virando no último sentido detectado
      if (errorPrev > 0) setMotorSpeeds(baseSpeed, -baseSpeed);
      else setMotorSpeeds(-baseSpeed, baseSpeed);
      return;
    }

    // Cálculo do PID
    float P = error;
    integral += error;
    integral = constrain(integral, -1000.0, 1000.0);
    float D = error - errorPrev;
    errorPrev = error;

    float steering = (Kp * P) + (Ki * integral) + (Kd * D);

    int leftMotor  = baseSpeed + (int)steering;
    int rightMotor = baseSpeed - (int)steering;

    leftMotor  = constrain(leftMotor, -maxSpeed, maxSpeed);
    rightMotor = constrain(rightMotor, -maxSpeed, maxSpeed);

    setMotorSpeeds(leftMotor, rightMotor);
  }
}
