/*
 * =====================================================================================
 * FIRMWARE INTEGRAL: CARRINHO SEGUIDOR DE LINHA AUTÔNOMO COM ESP32-CAM (AI-THINKER OV2640)
 * =====================================================================================
 * 
 * Arquitetura Dual-Core FreeRTOS & Servidor Web Dual-Port (Anti-Bloqueio de Streaming):
 * 
 *  1. CORE 1 (Visão Computacional & Controle PD em Tempo Real - ~30 ms):
 *     - Captura QQVGA (160x120) em Grayscale de 8 bits.
 *     - Processamento de ROI inferior (Y de 90 a 110).
 *     - Binarização adaptativa por Threshold e cálculo do centróide Cx.
 *     - Cálculo de Erro de Desvio: Erro = Cx - 80 (-80 a +80).
 *     - Malha Fechada de Controle PD para o Servo de Direção Dianteiro (50° a 130°).
 *     - Modulação PWM com Redução Adaptativa de Velocidade em Curvas Fechadas (|Erro| > 35).
 *     - Failsafe de Perda de Linha: Timeout de 300 ms com parada segura dos motores.
 * 
 *  2. CORE 0 (Servidor Web & Streaming Dual-Port de Alto Desempenho):
 *     - Conexão como STA em Hotspot móvel com IP Estático (192.168.43.50).
 *     - Porta 80: Painel Web Dark Theme, Telemetria JSON (/data), Calibração (/tune), Comandos (/cmd).
 *     - Porta 81: Servidor Dedicado de Streaming MJPEG (/stream) - Nunca bloqueia as rotas de controle!
 * 
 *  3. HARDWARE & SEGURANÇA ELÉTRICA:
 *     - Servo de Direção: GPIO 02 via LEDC (50 Hz, 14 bits).
 *     - Ponte H Traseira: IN1=GPIO 14, IN2=GPIO 15, IN3=GPIO 13, IN4=GPIO 12.
 *     - Proteção de Boot: GPIO 12 e demais pinos inicializados em LOW imediatamente.
 * =====================================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* =====================================================================================
 * 1. CONFIGURAÇÕES DE REDE E IP ESTÁTICO (CORE 0)
 * ===================================================================================== */

// --- Credenciais do Hotspot Móvel ---
const char* WIFI_SSID     = "Redmi Note 10S";
const char* WIFI_PASS     = "monobola8";

// --- Configuração de IP Estático Fixo ---
#define USAR_IP_ESTATICO  true

const IPAddress LOCAL_IP(10, 164, 64, 50);
const IPAddress GATEWAY(10, 164, 64, 84);
const IPAddress SUBNET(255, 255, 255, 0);
const IPAddress PRIMARY_DNS(10, 164, 64, 84);

/* =====================================================================================
 * 2. PINAGEM DO HARDWARE (ESP32-CAM AI-THINKER)
 * ===================================================================================== */

// --- Câmera OV2640 (AI-Thinker) ---
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

// --- Atuadores: Servo Dianteiro e Motores DC Traseiros ---
#define PIN_SERVO         2   // GPIO 02: Servo de direção dianteira
#define PIN_MOTOR_IN1     14  // GPIO 14: Motor Esquerdo Avanço (PWM)
#define PIN_MOTOR_IN2     15  // GPIO 15: Motor Esquerdo Direção (GND/LOW)
#define PIN_MOTOR_IN3     13  // GPIO 13: Motor Direito Avanço (PWM)
#define PIN_MOTOR_IN4     12  // GPIO 12: Motor Direito Direção (GND/LOW)
#define PIN_LED_STATUS    33  // GPIO 33: LED Vermelho Onboard (Active LOW)

// --- Canais PWM LEDC ---
#define LEDC_CH_SERVO         0
#define LEDC_CH_MOTOR_LEFT    1
#define LEDC_CH_MOTOR_RIGHT   2
#define LEDC_FREQ_SERVO       50     // 50 Hz para Servo Motor
#define LEDC_RES_SERVO        14     // 14 bits (0..16383)
#define LEDC_FREQ_MOTOR       5000   // 5 kHz para Ponte H DC
#define LEDC_RES_MOTOR        8      // 8 bits (0..255)

/* =====================================================================================
 * 3. CONSTANTES E PARÂMETROS DINÂMICOS DE CALIBRAÇÃO (THREAD-SAFE)
 * ===================================================================================== */

// --- Dimensões da Imagem e Região de Interesse (ROI) ---
#define CAM_WIDTH             160
#define CAM_HEIGHT            120
#define ROI_Y_START           90
#define ROI_Y_END             110
#define CENTRO_X_IDEAL        80.0f
#define MIN_PIXELS_LINHA      8

// --- Ajuste de Orientação da Câmera (Inversão / Espelhamento) ---
#define CAMERA_VFLIP          0      // 0 = Normal, 1 = Inverter Vertical (De cabeça para baixo)
#define CAMERA_HMIRROR        1      // 0 = Normal, 1 = Espelhar Horizontal (Esquerda/Direita)

// --- Inversão de Sentido do Servo de Direção ---
#define INVERTER_DIRECAO_SERVO false // false: Erro positivo vira >90° | true: Inverte o esterçamento

#define SERVO_ANGULO_CENTRO   90.0f
#define SERVO_ANGULO_MIN      50.0f
#define SERVO_ANGULO_MAX      130.0f
#define TIMEOUT_PERDA_LINHA   300
#define CICLO_CONTROLE_MS     30

struct ConfigControle {
  float kp;
  float kd;
  uint8_t threshold;
  int velocidadeBase;
  int pwmMinimo;
  bool tracaoHabilitada;
};

ConfigControle g_config = {
  .kp = 0.65f,
  .kd = 0.35f,
  .threshold = 80,
  .velocidadeBase = 150,
  .pwmMinimo = 90,
  .tracaoHabilitada = true
};

portMUX_TYPE g_configMux = portMUX_INITIALIZER_UNLOCKED;

struct TelemetriaRobo {
  float centroideX;
  float erroDesvio;
  float anguloServo;
  int pwmAtual;
  bool linhaDetectada;
  bool tracaoAtiva;
  uint32_t pixelsLinha;
  float fpsReal;
  unsigned long timestamp;
};

volatile TelemetriaRobo g_telemetria = {
  .centroideX = 80.0f,
  .erroDesvio = 0.0f,
  .anguloServo = 90.0f,
  .pwmAtual = 0,
  .linhaDetectada = false,
  .tracaoAtiva = true,
  .pixelsLinha = 0,
  .fpsReal = 0.0f,
  .timestamp = 0
};

struct SharedFrame {
  uint8_t* jpg_buf;
  size_t jpg_len;
  SemaphoreHandle_t mutex;
};

SharedFrame g_shared_frame = {
  .jpg_buf = NULL,
  .jpg_len = 0,
  .mutex = NULL
};

TaskHandle_t g_taskVisaoHandle = NULL;
httpd_handle_t g_httpd_web = NULL;    // Servidor Porta 80 (Web UI / API REST)
httpd_handle_t g_httpd_stream = NULL; // Servidor Porta 81 (Streaming MJPEG Dedicado)

/* =====================================================================================
 * 4. ATUAÇÃO E CONTROLE DE HARDWARE (LEDC / MOTORES / SERVO)
 * ===================================================================================== */

void inicializarHardwareSeguro() {
  pinMode(PIN_SERVO, OUTPUT);
  digitalWrite(PIN_SERVO, LOW);

  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  pinMode(PIN_MOTOR_IN3, OUTPUT);
  pinMode(PIN_MOTOR_IN4, OUTPUT);

  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  digitalWrite(PIN_MOTOR_IN3, LOW);
  digitalWrite(PIN_MOTOR_IN4, LOW); // Crítico no bootloader

  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, HIGH); // Inicia desligado (Active LOW)

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(PIN_SERVO, LEDC_FREQ_SERVO, LEDC_RES_SERVO);
  ledcAttach(PIN_MOTOR_IN1, LEDC_FREQ_MOTOR, LEDC_RES_MOTOR);
  ledcAttach(PIN_MOTOR_IN3, LEDC_FREQ_MOTOR, LEDC_RES_MOTOR);
#else
  ledcSetup(LEDC_CH_SERVO, LEDC_FREQ_SERVO, LEDC_RES_SERVO);
  ledcAttachPin(PIN_SERVO, LEDC_CH_SERVO);

  ledcSetup(LEDC_CH_MOTOR_LEFT, LEDC_FREQ_MOTOR, LEDC_RES_MOTOR);
  ledcAttachPin(PIN_MOTOR_IN1, LEDC_CH_MOTOR_LEFT);

  ledcSetup(LEDC_CH_MOTOR_RIGHT, LEDC_FREQ_MOTOR, LEDC_RES_MOTOR);
  ledcAttachPin(PIN_MOTOR_IN3, LEDC_CH_MOTOR_RIGHT);
#endif
}

void setAnguloServo(float angulo) {
  angulo = constrain(angulo, SERVO_ANGULO_MIN, SERVO_ANGULO_MAX);
  
  float pulso_us = 500.0f + (angulo / 180.0f) * 2000.0f;
  uint32_t duty = (uint32_t)((pulso_us / 20000.0f) * 16383.0f);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(PIN_SERVO, duty);
#else
  ledcWrite(LEDC_CH_SERVO, duty);
#endif
}

void setMotoresAvanco(int pwm_esq, int pwm_dir) {
  pwm_esq = constrain(pwm_esq, 0, 255);
  pwm_dir = constrain(pwm_dir, 0, 255);

  digitalWrite(PIN_MOTOR_IN2, LOW);
  digitalWrite(PIN_MOTOR_IN4, LOW);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(PIN_MOTOR_IN1, pwm_esq);
  ledcWrite(PIN_MOTOR_IN3, pwm_dir);
#else
  ledcWrite(LEDC_CH_MOTOR_LEFT, pwm_esq);
  ledcWrite(LEDC_CH_MOTOR_RIGHT, pwm_dir);
#endif
}

void pararMotores() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(PIN_MOTOR_IN1, 0);
  ledcWrite(PIN_MOTOR_IN3, 0);
#else
  ledcWrite(LEDC_CH_MOTOR_LEFT, 0);
  ledcWrite(LEDC_CH_MOTOR_RIGHT, 0);
#endif
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  digitalWrite(PIN_MOTOR_IN3, LOW);
  digitalWrite(PIN_MOTOR_IN4, LOW);
}

/* =====================================================================================
 * 5. INICIALIZAÇÃO DA CÂMERA OV2640 (AI-THINKER)
 * ===================================================================================== */

esp_err_t inicializarCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size   = FRAMESIZE_QQVGA;     // 160 x 120 pixels
  config.jpeg_quality = 12;
  config.fb_count     = 2;                   // Double buffering com PSRAM
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  if (!psramFound()) {
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAMERA] Erro na inicializacao: 0x%x\n", err);
    return err;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_vflip(s, CAMERA_VFLIP);
    s->set_hmirror(s, CAMERA_HMIRROR);
    s->set_brightness(s, 1);
    s->set_contrast(s, 2);
  }

  Serial.println("[CAMERA] OV2640 inicializada em QQVGA Grayscale.");
  return ESP_OK;
}

/* =====================================================================================
 * 6. PIPELINE DE VISÃO COMPUTACIONAL E CONTROLE PD (CORE 1)
 * ===================================================================================== */

void desenharOverlayVisual(uint8_t* buf, int largura, int altura, float cx, bool detectada) {
  // Linhas guia da ROI (Y=90 e Y=110)
  for (int x = 0; x < largura; x += 3) {
    buf[ROI_Y_START * largura + x] = 255;
    buf[ROI_Y_END * largura + x] = 255;
  }

  // Linha central de referência (X=80)
  for (int y = ROI_Y_START; y <= ROI_Y_END; y += 2) {
    buf[y * largura + 80] = 128;
  }

  // Marcador visual do centróide rastreado
  if (detectada && cx >= 0 && cx < largura) {
    int int_cx = (int)cx;
    for (int y = ROI_Y_START; y <= ROI_Y_END; y++) {
      buf[y * largura + int_cx] = 255;
      if (int_cx > 0) buf[y * largura + (int_cx - 1)] = 255;
      if (int_cx < largura - 1) buf[y * largura + (int_cx + 1)] = 255;
    }
  }
}

void taskControleVisao(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(CICLO_CONTROLE_MS);

  float erro_anterior = 0.0f;
  unsigned long ultimo_tempo_linha_ms = millis();
  unsigned long tempo_anterior_fps = millis();
  int contador_frames_fps = 0;

  Serial.println("[CORE 1] Loop de Visao Computacional e Controle PD iniciado.");

  for (;;) {
    camera_fb_t *fb = esp_camera_fb_get();

    // Leitura atômica dos parâmetros de calibração configurados via Web
    portENTER_CRITICAL(&g_configMux);
    float cur_kp = g_config.kp;
    float cur_kd = g_config.kd;
    uint8_t cur_th = g_config.threshold;
    int cur_speed = g_config.velocidadeBase;
    int cur_min_pwm = g_config.pwmMinimo;
    bool tracao_on = g_config.tracaoHabilitada;
    portEXIT_CRITICAL(&g_configMux);

    if (fb != NULL && fb->format == PIXFORMAT_GRAYSCALE) {
      uint64_t soma_ponderada_x = 0;
      uint32_t total_pixels_escuros = 0;

      // 1. Processamento da ROI inferior (linhas 90 a 110)
      for (int y = ROI_Y_START; y <= ROI_Y_END; y++) {
        uint8_t *linha_ptr = &fb->buf[y * CAM_WIDTH];
        for (int x = 0; x < CAM_WIDTH; x++) {
          if (linha_ptr[x] < cur_th) {
            soma_ponderada_x += x;
            total_pixels_escuros++;
          }
        }
      }

      float centroide_x = CENTRO_X_IDEAL;
      float erro = 0.0f;
      float angulo_servo = SERVO_ANGULO_CENTRO;
      int pwm_atuante = 0;
      bool linha_valida = (total_pixels_escuros >= MIN_PIXELS_LINHA);

      if (linha_valida) {
        // 2. Cálculo do Centróide e Erro (-80 a +80)
        centroide_x = (float)soma_ponderada_x / (float)total_pixels_escuros;
        erro = centroide_x - CENTRO_X_IDEAL;
        ultimo_tempo_linha_ms = millis();

        // 3. Algoritmo de Controle PD de Direção
        float d_erro = erro - erro_anterior;
#if INVERTER_DIRECAO_SERVO
        angulo_servo = SERVO_ANGULO_CENTRO - (cur_kp * erro + cur_kd * d_erro);
#else
        angulo_servo = SERVO_ANGULO_CENTRO + (cur_kp * erro + cur_kd * d_erro);
#endif
        angulo_servo = constrain(angulo_servo, SERVO_ANGULO_MIN, SERVO_ANGULO_MAX);
        erro_anterior = erro;

        // 4. Controle Adaptativo de Velocidade com Redução Diferencial em Curvas (|Erro| > 35)
        float abs_erro = fabsf(erro);
        if (abs_erro > 35.0f) {
          float fator_curva = (abs_erro - 35.0f) / 45.0f;
          pwm_atuante = cur_speed - (int)(fator_curva * (cur_speed - cur_min_pwm));
        } else {
          pwm_atuante = cur_speed;
        }
        pwm_atuante = constrain(pwm_atuante, cur_min_pwm, 255);

        // Atuação Física
        setAnguloServo(angulo_servo);
        if (tracao_on) {
          setMotoresAvanco(pwm_atuante, pwm_atuante);
        } else {
          pararMotores();
        }

      } else {
        // 5. Failsafe: Perda de Linha
        if (millis() - ultimo_tempo_linha_ms > TIMEOUT_PERDA_LINHA) {
          angulo_servo = SERVO_ANGULO_CENTRO;
          pwm_atuante = 0;
          setAnguloServo(SERVO_ANGULO_CENTRO);
          pararMotores();
          erro_anterior = 0.0f;
        } else {
#if INVERTER_DIRECAO_SERVO
          angulo_servo = SERVO_ANGULO_CENTRO - (cur_kp * erro_anterior);
#else
          angulo_servo = SERVO_ANGULO_CENTRO + (cur_kp * erro_anterior);
#endif
          angulo_servo = constrain(angulo_servo, SERVO_ANGULO_MIN, SERVO_ANGULO_MAX);
          pwm_atuante = cur_min_pwm;
          setAnguloServo(angulo_servo);
          if (tracao_on) {
            setMotoresAvanco(pwm_atuante, pwm_atuante);
          } else {
            pararMotores();
          }
        }
      }

      // 6. Taxa de Quadros Real (FPS)
      contador_frames_fps++;
      if (millis() - tempo_anterior_fps >= 1000) {
        g_telemetria.fpsReal = (float)contador_frames_fps * 1000.0f / (float)(millis() - tempo_anterior_fps);
        contador_frames_fps = 0;
        tempo_anterior_fps = millis();
      }

      // Atualização de Telemetria Compartilhada
      g_telemetria.centroideX = centroide_x;
      g_telemetria.erroDesvio = erro;
      g_telemetria.anguloServo = angulo_servo;
      g_telemetria.pwmAtual = tracao_on ? pwm_atuante : 0;
      g_telemetria.linhaDetectada = linha_valida;
      g_telemetria.tracaoAtiva = tracao_on;
      g_telemetria.pixelsLinha = total_pixels_escuros;
      g_telemetria.timestamp = millis();

      // 7. Preparação Thread-Safe do Frame JPEG para Streaming MJPEG
      desenharOverlayVisual(fb->buf, CAM_WIDTH, CAM_HEIGHT, centroide_x, linha_valida);

      uint8_t *jpg_out = NULL;
      size_t jpg_len_out = 0;
      if (fmt2jpg(fb->buf, fb->len, CAM_WIDTH, CAM_HEIGHT, PIXFORMAT_GRAYSCALE, 75, &jpg_out, &jpg_len_out)) {
        if (xSemaphoreTake(g_shared_frame.mutex, (TickType_t)5) == pdTRUE) {
          if (g_shared_frame.jpg_buf != NULL) {
            free(g_shared_frame.jpg_buf);
          }
          g_shared_frame.jpg_buf = jpg_out;
          g_shared_frame.jpg_len = jpg_len_out;
          xSemaphoreGive(g_shared_frame.mutex);
        } else {
          free(jpg_out);
        }
      }

      esp_camera_fb_return(fb);

    } else if (fb != NULL) {
      esp_camera_fb_return(fb);
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/* =====================================================================================
 * 7. INTERFACE WEB EMBARCADA OFFLINE (PORTA 80)
 * ===================================================================================== */

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP32-CAM | Painel Autônomo Seguidor de Linha</title>
  <style>
    :root {
      --bg-base: #0a0e17;
      --bg-panel: #121927;
      --bg-card: rgba(23, 32, 54, 0.75);
      --border-color: rgba(0, 240, 255, 0.18);
      --accent-cyan: #00f0ff;
      --accent-blue: #3b82f6;
      --accent-green: #10b981;
      --accent-red: #ef4444;
      --text-main: #f3f4f6;
      --text-muted: #9ca3af;
      --font-stack: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      --font-mono: "Courier New", monospace;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: var(--font-stack);
      background: radial-gradient(circle at 80% 10%, #172554 0%, var(--bg-base) 65%);
      color: var(--text-main);
      min-height: 100vh;
      padding: 20px;
    }
    .wrapper { max-width: 1200px; margin: 0 auto; }
    header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding-bottom: 16px;
      border-bottom: 1px solid var(--border-color);
      margin-bottom: 20px;
      flex-wrap: wrap;
      gap: 12px;
    }
    .logo-area { display: flex; align-items: center; gap: 10px; }
    .logo-area h1 { font-size: 1.35rem; font-weight: 700; color: #fff; }
    .logo-sub { font-size: 0.8rem; color: var(--accent-cyan); font-family: var(--font-mono); }
    .status-badge {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      padding: 6px 14px;
      border-radius: 9999px;
      font-size: 0.85rem;
      font-weight: 700;
      background: rgba(16, 185, 129, 0.15);
      border: 1px solid rgba(16, 185, 129, 0.4);
      color: var(--accent-green);
    }
    .status-badge.stopped {
      background: rgba(239, 68, 68, 0.15);
      border-color: rgba(239, 68, 68, 0.4);
      color: var(--accent-red);
    }
    .dot-pulse {
      width: 8px; height: 8px; border-radius: 50%;
      background: currentColor;
      box-shadow: 0 0 10px currentColor;
    }
    .main-grid {
      display: grid;
      grid-template-columns: 1.15fr 1fr;
      gap: 20px;
    }
    @media (max-width: 900px) { .main-grid { grid-template-columns: 1fr; } }
    .card {
      background: var(--bg-card);
      border: 1px solid var(--border-color);
      border-radius: 14px;
      padding: 18px;
      box-shadow: 0 10px 30px rgba(0,0,0,0.4);
      backdrop-filter: blur(8px);
      margin-bottom: 20px;
    }
    .card-head {
      font-size: 1rem;
      font-weight: 600;
      color: var(--accent-cyan);
      margin-bottom: 14px;
      display: flex;
      align-items: center;
      gap: 8px;
    }
    .stream-box {
      background: #000;
      border-radius: 10px;
      overflow: hidden;
      border: 2px solid rgba(0, 240, 255, 0.3);
      display: flex;
      justify-content: center;
      align-items: center;
      min-height: 240px;
    }
    .stream-box img {
      width: 100%;
      height: auto;
      display: block;
      image-rendering: pixelated;
    }
    .btn-group {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
      margin-top: 14px;
    }
    .btn {
      padding: 12px;
      border: none;
      border-radius: 8px;
      font-weight: 700;
      font-size: 0.95rem;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      transition: all 0.2s ease;
    }
    .btn:active { transform: scale(0.97); }
    .btn-start {
      background: linear-gradient(135deg, #10b981 0%, #059669 100%);
      color: #fff;
      box-shadow: 0 0 15px rgba(16, 185, 129, 0.4);
    }
    .btn-stop {
      background: linear-gradient(135deg, #ef4444 0%, #dc2626 100%);
      color: #fff;
      box-shadow: 0 0 15px rgba(239, 68, 68, 0.4);
    }
    .metrics-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 12px;
    }
    .metric-box {
      background: var(--bg-panel);
      border: 1px solid rgba(255,255,255,0.06);
      padding: 14px;
      border-radius: 10px;
    }
    .metric-label { font-size: 0.75rem; color: var(--text-muted); text-transform: uppercase; font-weight: 600; }
    .metric-val {
      font-family: var(--font-mono);
      font-size: 1.6rem;
      font-weight: 700;
      color: #fff;
      margin-top: 4px;
    }
    .metric-unit { font-size: 0.85rem; color: var(--text-muted); }
    .slider-group { margin-top: 14px; }
    .slider-item { margin-bottom: 12px; }
    .slider-header {
      display: flex;
      justify-content: space-between;
      font-size: 0.85rem;
      margin-bottom: 6px;
    }
    .slider-header span:last-child {
      font-family: var(--font-mono);
      color: var(--accent-cyan);
      font-weight: 700;
    }
    input[type=range] {
      width: 100%;
      height: 6px;
      background: #1e293b;
      border-radius: 4px;
      outline: none;
      -webkit-appearance: none;
    }
    input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 18px;
      height: 18px;
      border-radius: 50%;
      background: var(--accent-cyan);
      cursor: pointer;
      box-shadow: 0 0 8px var(--accent-cyan);
    }
    .btn-apply {
      width: 100%;
      padding: 10px;
      background: linear-gradient(135deg, #3b82f6 0%, #2563eb 100%);
      color: #fff;
      border: none;
      border-radius: 8px;
      font-weight: 700;
      cursor: pointer;
      margin-top: 8px;
    }
    .toast {
      position: fixed;
      bottom: 20px;
      right: 20px;
      background: #1e293b;
      border: 1px solid var(--accent-cyan);
      color: #fff;
      padding: 10px 18px;
      border-radius: 8px;
      font-size: 0.85rem;
      opacity: 0;
      transition: opacity 0.3s ease;
      pointer-events: none;
    }
    .toast.show { opacity: 1; }
  </style>
</head>
<body>
  <div class="wrapper">
    <header>
      <div class="logo-area">
        <h1>🏎️ Carrinho Seguidor ESP32-CAM</h1>
        <div class="logo-sub">OV2640 // FREERTOS DUAL-CORE</div>
      </div>
      <div id="badgeStatus" class="status-badge">
        <div class="dot-pulse"></div>
        <span id="txtStatusBadge">RODANDO</span>
      </div>
    </header>

    <div class="main-grid">
      <div>
        <div class="card">
          <div class="card-head">📷 Visão Computacional Embarcada (Porta 81)</div>
          <div class="stream-box">
            <img id="streamImg" alt="Streaming MJPEG ESP32-CAM">
          </div>
          <div class="btn-group">
            <button class="btn btn-start" onclick="enviarComando('start')">▶ INICIAR TRAÇÃO</button>
            <button class="btn btn-stop" onclick="enviarComando('stop')">🛑 PARAR EMERGÊNCIA</button>
          </div>
        </div>

        <div class="card">
          <div class="card-head">⚡ Telemetria em Tempo Real (/data)</div>
          <div class="metrics-grid">
            <div class="metric-box">
              <div class="metric-label">Taxa de Quadros</div>
              <div class="metric-val" id="valFps">-- <span class="metric-unit">FPS</span></div>
            </div>
            <div class="metric-box">
              <div class="metric-label">Erro de Desvio (Centróide)</div>
              <div class="metric-val" id="valErro">-- <span class="metric-unit">px</span></div>
            </div>
            <div class="metric-box">
              <div class="metric-label">Ângulo do Servo</div>
              <div class="metric-val" id="valAngulo">90.0<span class="metric-unit">°</span></div>
            </div>
            <div class="metric-box">
              <div class="metric-label">PWM dos Motores</div>
              <div class="metric-val" id="valPwm">0 <span class="metric-unit">/255</span></div>
            </div>
          </div>
        </div>
      </div>

      <div>
        <div class="card">
          <div class="card-head">⚙️ Calibração do Algoritmo (/tune)</div>
          <div class="slider-group">
            <div class="slider-item">
              <div class="slider-header">
                <span>Kp (Ganho Proporcional)</span>
                <span id="lblKp">0.65</span>
              </div>
              <input type="range" id="rngKp" min="0.0" max="2.0" step="0.05" value="0.65" oninput="atualizarLabel('lblKp', this.value)">
            </div>

            <div class="slider-item">
              <div class="slider-header">
                <span>Kd (Ganho Derivativo)</span>
                <span id="lblKd">0.35</span>
              </div>
              <input type="range" id="rngKd" min="0.0" max="1.5" step="0.05" value="0.35" oninput="atualizarLabel('lblKd', this.value)">
            </div>

            <div class="slider-item">
              <div class="slider-header">
                <span>Threshold (Binarização Linha)</span>
                <span id="lblTh">80</span>
              </div>
              <input type="range" id="rngTh" min="20" max="200" step="1" value="80" oninput="atualizarLabel('lblTh', this.value)">
            </div>

            <div class="slider-item">
              <div class="slider-header">
                <span>Velocidade Base (PWM)</span>
                <span id="lblSpeed">150</span>
              </div>
              <input type="range" id="rngSpeed" min="60" max="255" step="5" value="150" oninput="atualizarLabel('lblSpeed', this.value)">
            </div>

            <button class="btn-apply" onclick="enviarCalibracao()">💾 APLICAR PARÂMETROS</button>
          </div>
        </div>

        <div class="card">
          <div class="card-head">ℹ️ Informações do Sistema</div>
          <div style="font-size: 0.85rem; line-height: 1.6; color: var(--text-muted);">
            <div>• <b>IP do Robô:</b> <span id="lblIpHost">10.164.64.50</span></div>
            <div>• <b>Portas:</b> 80 (Painel Web) / 81 (Streaming MJPEG)</div>
            <div>• <b>Hotspot:</b> Redmi Note 10S</div>
            <div>• <b>Loop de Controle:</b> Core 1 (~30 ms)</div>
            <div>• <b>Servidor HTTP:</b> Core 0</div>
          </div>
        </div>
      </div>
    </div>
  </div>

  <div id="toast" class="toast">Comando executado</div>

  <script>
    // Inicializa o Stream na porta 81 dedicada
    window.addEventListener('DOMContentLoaded', () => {
      const host = window.location.hostname || '10.164.64.50';
      document.getElementById('lblIpHost').innerText = host;
      document.getElementById('streamImg').src = 'http://' + host + ':81/stream';
    });

    function showToast(msg) {
      const t = document.getElementById('toast');
      t.innerText = msg;
      t.className = 'toast show';
      setTimeout(() => { t.className = 'toast'; }, 2200);
    }

    function atualizarLabel(id, val) {
      document.getElementById(id).innerText = val;
    }

    async function enviarComando(action) {
      try {
        const res = await fetch('/cmd?action=' + action);
        if (res.ok) {
          showToast(action === 'start' ? '▶ Tração Iniciada' : '🛑 Tração Parada');
        }
      } catch (e) {
        showToast('Erro ao enviar comando');
      }
    }

    async function enviarCalibracao() {
      const kp = document.getElementById('rngKp').value;
      const kd = document.getElementById('rngKd').value;
      const th = document.getElementById('rngTh').value;
      const speed = document.getElementById('rngSpeed').value;

      try {
        const url = `/tune?kp=${kp}&kd=${kd}&th=${th}&speed=${speed}`;
        const res = await fetch(url);
        if (res.ok) {
          showToast('💾 Parâmetros salvos com sucesso!');
        }
      } catch (e) {
        showToast('Erro ao calibrar');
      }
    }

    async function pollingTelemetria() {
      try {
        const resp = await fetch('/data');
        if (resp.ok) {
          const d = await resp.json();
          document.getElementById('valFps').innerHTML = d.fps.toFixed(1) + ' <span class="metric-unit">FPS</span>';
          document.getElementById('valErro').innerHTML = (d.erro >= 0 ? '+' : '') + d.erro.toFixed(1) + ' <span class="metric-unit">px</span>';
          document.getElementById('valAngulo').innerHTML = d.angulo.toFixed(1) + '<span class="metric-unit">°</span>';
          document.getElementById('valPwm').innerHTML = d.pwm + ' <span class="metric-unit">/255</span>';

          const badge = document.getElementById('badgeStatus');
          const txtBadge = document.getElementById('txtStatusBadge');
          if (d.status === 'RODANDO' || d.status === 'TRAÇÃO ATIVA') {
            badge.className = 'status-badge';
            txtBadge.innerText = d.detectada ? 'RASTREANDO' : 'BUSCANDO LINHA';
          } else {
            badge.className = 'status-badge stopped';
            txtBadge.innerText = 'PARADO (EMERGÊNCIA)';
          }
        }
      } catch (e) {}
    }

    setInterval(pollingTelemetria, 100);
  </script>
</body>
</html>
)rawliteral";

/* =====================================================================================
 * 8. HANDLERS DAS ROTAS HTTP (CORE 0 - DUAL-PORT HTTP SERVER)
 * ===================================================================================== */

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=esp32cam_boundary";
static const char* _STREAM_BOUNDARY = "\r\n--esp32cam_boundary\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

/**
 * @brief Handler dedicado da rota de Streaming MJPEG (/stream na porta 81).
 */
static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t res = ESP_OK;
  char part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "30");

  Serial.println("[STREAM] Cliente conectado ao streaming (Porta 81).");

  while (true) {
    uint8_t *jpg_temp = NULL;
    size_t jpg_len = 0;

    if (xSemaphoreTake(g_shared_frame.mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (g_shared_frame.jpg_buf != NULL && g_shared_frame.jpg_len > 0) {
        jpg_len = g_shared_frame.jpg_len;
        jpg_temp = (uint8_t*)malloc(jpg_len);
        if (jpg_temp != NULL) {
          memcpy(jpg_temp, g_shared_frame.jpg_buf, jpg_len);
        }
      }
      xSemaphoreGive(g_shared_frame.mutex);
    }

    if (jpg_temp != NULL && jpg_len > 0) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
      if (res == ESP_OK) {
        size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, jpg_len);
        res = httpd_resp_send_chunk(req, part_buf, hlen);
      }
      if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, (const char *)jpg_temp, jpg_len);
      }
      free(jpg_temp);

      if (res != ESP_OK) break;
    }

    vTaskDelay(pdMS_TO_TICKS(30));
  }

  Serial.println("[STREAM] Cliente desconectado.");
  return res;
}

/**
 * @brief Handler para telemetria JSON (/data na porta 80).
 */
static esp_err_t data_handler(httpd_req_t *req) {
  char json[256];
  const char* status_str = g_telemetria.tracaoAtiva ? (g_telemetria.linhaDetectada ? "RODANDO" : "LINHA PERDIDA") : "PARADO";

  snprintf(json, sizeof(json),
    "{\"erro\":%.2f,\"angulo\":%.1f,\"pwm\":%d,\"fps\":%.1f,\"detectada\":%s,\"status\":\"%s\",\"cx\":%.1f,\"pixels\":%u}",
    g_telemetria.erroDesvio,
    g_telemetria.anguloServo,
    g_telemetria.pwmAtual,
    g_telemetria.fpsReal,
    g_telemetria.linhaDetectada ? "true" : "false",
    status_str,
    g_telemetria.centroideX,
    g_telemetria.pixelsLinha
  );

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, strlen(json));
}

/**
 * @brief Handler para ajuste dinâmico (/tune na porta 80).
 */
static esp_err_t tune_handler(httpd_req_t *req) {
  char* buf = NULL;
  size_t buf_len = httpd_req_get_url_query_len(req) + 1;

  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if (buf && httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      char param[32];

      portENTER_CRITICAL(&g_configMux);
      if (httpd_query_key_value(buf, "kp", param, sizeof(param)) == ESP_OK) {
        g_config.kp = atof(param);
      }
      if (httpd_query_key_value(buf, "kd", param, sizeof(param)) == ESP_OK) {
        g_config.kd = atof(param);
      }
      if (httpd_query_key_value(buf, "th", param, sizeof(param)) == ESP_OK) {
        g_config.threshold = (uint8_t)atoi(param);
      }
      if (httpd_query_key_value(buf, "speed", param, sizeof(param)) == ESP_OK) {
        g_config.velocidadeBase = atoi(param);
      }
      portEXIT_CRITICAL(&g_configMux);

      Serial.printf("[TUNE] Kp=%.2f, Kd=%.2f, Th=%d, Speed=%d\n",
        g_config.kp, g_config.kd, g_config.threshold, g_config.velocidadeBase);
    }
    if (buf) free(buf);
  }

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "OK", 2);
}

/**
 * @brief Handler para comandos (/cmd na porta 80).
 */
static esp_err_t cmd_handler(httpd_req_t *req) {
  char* buf = NULL;
  size_t buf_len = httpd_req_get_url_query_len(req) + 1;

  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if (buf && httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      char action[16];
      if (httpd_query_key_value(buf, "action", action, sizeof(action)) == ESP_OK) {
        portENTER_CRITICAL(&g_configMux);
        if (strcmp(action, "start") == 0) {
          g_config.tracaoHabilitada = true;
          Serial.println("[CMD] Tracao Habilitada (START).");
        } else if (strcmp(action, "stop") == 0) {
          g_config.tracaoHabilitada = false;
          pararMotores();
          Serial.println("[CMD] Tracao Desabilitada (STOP).");
        }
        portEXIT_CRITICAL(&g_configMux);
      }
    }
    if (buf) free(buf);
  }

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "OK", 2);
}

/**
 * @brief Handler para o index (HTML na porta 80).
 */
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

/**
 * @brief Inicializa os servidores HTTP separados (Porta 80 e Porta 81).
 */
void iniciarServidoresWeb() {
  // --- 1. Servidor Web & API (Porta 80) ---
  httpd_config_t config_web = HTTPD_DEFAULT_CONFIG();
  config_web.server_port = 80;
  config_web.ctrl_port = 32768;
  config_web.max_open_sockets = 4;
  config_web.stack_size = 8192;

  httpd_uri_t index_uri = { .uri = "/",     .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL };
  httpd_uri_t data_uri  = { .uri = "/data", .method = HTTP_GET, .handler = data_handler,  .user_ctx = NULL };
  httpd_uri_t tune_uri  = { .uri = "/tune", .method = HTTP_GET, .handler = tune_handler,  .user_ctx = NULL };
  httpd_uri_t cmd_uri   = { .uri = "/cmd",  .method = HTTP_GET, .handler = cmd_handler,   .user_ctx = NULL };

  if (httpd_start(&g_httpd_web, &config_web) == ESP_OK) {
    httpd_register_uri_handler(g_httpd_web, &index_uri);
    httpd_register_uri_handler(g_httpd_web, &data_uri);
    httpd_register_uri_handler(g_httpd_web, &tune_uri);
    httpd_register_uri_handler(g_httpd_web, &cmd_uri);
    Serial.println("[HTTP] Servidor Web ativo na Porta 80 (/, /data, /tune, /cmd).");
  }

  // --- 2. Servidor de Streaming MJPEG Dedicado (Porta 81) ---
  httpd_config_t config_stream = HTTPD_DEFAULT_CONFIG();
  config_stream.server_port = 81;
  config_stream.ctrl_port = 32769;
  config_stream.max_open_sockets = 2;
  config_stream.stack_size = 4096;

  httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };

  if (httpd_start(&g_httpd_stream, &config_stream) == ESP_OK) {
    httpd_register_uri_handler(g_httpd_stream, &stream_uri);
    Serial.println("[HTTP] Servidor de Streaming ativo na Porta 81 (/stream).");
  }
}

/* =====================================================================================
 * 9. CONEXÃO WI-FI ESTÁTICA (CORE 0)
 * ===================================================================================== */

void conectarWiFiEstatico() {
  Serial.printf("\n[WIFI] Conectando ao Hotspot: %s...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);

#if USAR_IP_ESTATICO
  if (!WiFi.config(LOCAL_IP, GATEWAY, SUBNET, PRIMARY_DNS)) {
    Serial.println("[WIFI] Falha ao configurar IP Estatico!");
  } else {
    Serial.println("[WIFI] Usando IP Estatico configurado.");
  }
#endif

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int tentativas = 0;
  while (WiFi.status() != WL_CONNECTED && tentativas < 25) {
    digitalWrite(PIN_LED_STATUS, (tentativas % 2 == 0) ? LOW : HIGH); // Pisca LED
    delay(400);
    Serial.print(".");
    tentativas++;
  }

  // Fallback inteligente: se o IP estático falhar ou se o Hotspot tiver trocado de faixa, tenta DHCP
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[WIFI] Tentando reconexao automatica via DHCP...");
    WiFi.disconnect();
    delay(200);
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    tentativas = 0;
    while (WiFi.status() != WL_CONNECTED && tentativas < 20) {
      digitalWrite(PIN_LED_STATUS, (tentativas % 2 == 0) ? LOW : HIGH);
      delay(400);
      Serial.print(".");
      tentativas++;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(PIN_LED_STATUS, LOW); // LED aceso fixo = Wi-Fi CONECTADO!
    if (MDNS.begin("carrinho")) {
      MDNS.addService("http", "tcp", 80);
      Serial.println("[MDNS] Respondedor mDNS iniciado: http://carrinho.local/");
    }

    Serial.println("\n========================================================");
    Serial.println("[WIFI] CONECTADO COM SUCESSO AO HOTSPOT!");
    Serial.printf("[WIFI] Endereco do Painel: http://%s/\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WIFI] Link alternativo:   http://carrinho.local/\n");
    Serial.printf("[WIFI] Stream da Camera:   http://%s:81/stream\n", WiFi.localIP().toString().c_str());
    Serial.println("========================================================");
  } else {
    digitalWrite(PIN_LED_STATUS, HIGH); // LED apagado = Sem Wi-Fi
    Serial.println("\n[WIFI] Hotspot nao encontrado. Executando em modo autonomo.");
  }
}

/* =====================================================================================
 * 10. SETUP E LOOP PRINCIPAL
 * ===================================================================================== */

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================================");
  Serial.println("  ROBO SEGUIDOR DE LINHA AUTONOMO - ESP32-CAM OV2640");
  Serial.println("========================================================");

  // 1. Inicializa pinagem segura
  inicializarHardwareSeguro();
  setAnguloServo(SERVO_ANGULO_CENTRO);
  pararMotores();

  // 2. Cria Mutex para o buffer de imagem
  g_shared_frame.mutex = xSemaphoreCreateMutex();

  // 3. Inicializa Câmera OV2640
  if (inicializarCamera() != ESP_OK) {
    Serial.println("[FATAL] Falha na camera. Reiniciando...");
    delay(3000);
    ESP.restart();
  }

  // 4. Cria tarefa de Visão e Controle no Core 1
  xTaskCreatePinnedToCore(
    taskControleVisao,
    "taskControleVisao",
    8192,
    NULL,
    2,
    &g_taskVisaoHandle,
    1
  );

  // 5. Conecta ao Hotspot Wi-Fi e inicia Servidores na Porta 80 e 81
  conectarWiFiEstatico();
  iniciarServidoresWeb();

  Serial.println("[SISTEMA] Inicializacao concluida.\n");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(PIN_LED_STATUS, HIGH); // Apaga LED se perder conexão
    static unsigned long ultimo_reconnect = 0;
    if (millis() - ultimo_reconnect > 5000) {
      ultimo_reconnect = millis();
      Serial.println("[WIFI] Reconectando ao Hotspot...");
      WiFi.reconnect();
    }
  } else {
    digitalWrite(PIN_LED_STATUS, LOW); // Mantém LED ligado quando conectado
  }

  vTaskDelay(pdMS_TO_TICKS(1000));
}
