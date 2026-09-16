/*
 * =====================================================================================
 * FIRMWARE INTEGRAL: CARRINHO SEGUIDOR DE LINHA AUTÔNOMO COM ESP32-CAM (AI-THINKER OV2640)
 * =====================================================================================
 * 
 * Arquitetura Dual-Core FreeRTOS & Servidor Web Dual-Port (Anti-Bloqueio de Streaming):
 * 
 *  1. CORE 1 (Visão Computacional Multi-Zona & Controle PD Preditivo - ~30 ms):
 *     - Captura QQVGA (160x120) em Grayscale de 8 bits.
 *     - Processamento Multi-ROI em 3 Zonas de Profundidade (Tela Cheia):
 *         * Zona 1 (Base / Perto): Resposta imediata de erro lateral (E_base).
 *         * Zona 2 (Meio / Intermediário): Estabilização de trajetória (E_mid).
 *         * Zona 3 (Longe / Lookahead): Antecipação de curvas (E_far).
 *     - Reconhecimento Preditivo de Curvas:
 *         * Cálculo de Curvatura: Delta_E = E_far - E_base.
 *         * Classificação de Pista: RETA, CURVA SUAVE (ESQ/DIR), CURVA FECHADA (ESQ/DIR).
 *         * Frenagem Antecipada Preditiva antes de entrar na curva.
 *         * Blend de Esterçamento Composto: E_total = (1 - w_far)*E_base + w_far*E_far.
 *     - Suporte dinâmico à Orientação do Sensor (Modo 90° Rotacionado / Modo Normal).
 *     - Thresholding Adaptativo Dinâmico para máxima robustez à iluminação.
 *     - Overlay Visual em Tempo Real com marcação das 3 zonas e vetor da curva no Stream.
 * 
 *  2. CORE 0 (Servidor Web & Streaming Dual-Port de Alto Desempenho):
 *     - Conexão como STA em Hotspot móvel com IP Estático e fallback DHCP.
 *     - Porta 80: Painel Web Dark Theme, Telemetria JSON (/data), Calibração (/tune), Comandos (/cmd).
 *     - Porta 81: Servidor Dedicado de Streaming MJPEG (/stream) com overlay visual completo.
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
 * 3. CONSTANTES E ESTRUTURAS DE CONTROLE MULTI-ZONA (THREAD-SAFE)
 * ===================================================================================== */

#define CAM_WIDTH             160
#define CAM_HEIGHT            120
#define MIN_PIXELS_ZONA       4

// Modos de Orientação da Câmera
enum ModoOrientacao {
  ORIENT_NORMAL_LANDSCAPE = 0, // Câmera na horizontal (160x120)
  ORIENT_ROTACIONADO_90   = 1  // Câmera na vertical/rotacionada (120x160) - Padrão AI-Thinker vertical
};

// Classificação de Pista / Curvas
enum TipoPista {
  PISTA_SEM_LINHA       = 0,
  PISTA_RETA            = 1,
  PISTA_CURVA_SUAVE_ESQ = 2,
  PISTA_CURVA_SUAVE_DIR = 3,
  PISTA_CURVA_FECH_ESQ  = 4,
  PISTA_CURVA_FECH_DIR  = 5
};

// Estrutura de resultados do processamento visual multi-zona
struct ResultadoVisaoMultiZona {
  float c_base, c_mid, c_far; // Posições dos centróides nas zonas
  float e_base, e_mid, e_far; // Erros em relação ao centro da pista
  float e_composto;           // Erro ponderado final
  float curvatura;            // Delta_E = e_far - e_base
  bool ok_base, ok_mid, ok_far;
  bool linha_valida;
  uint8_t tipo_pista;
  uint32_t total_pixels;
  uint8_t threshold_usado;
};

// Protótipos para o preprocessador do Arduino IDE
ResultadoVisaoMultiZona processarVisaoMultiZona(uint8_t* buf, uint8_t th_config, uint8_t orient, float peso_lookahead);
void desenharOverlayVisualMultiZona(uint8_t* buf, const ResultadoVisaoMultiZona& vis, uint8_t orient);
void desenharLinhaGrayscale(uint8_t* buf, int x0, int y0, int x1, int y1, uint8_t cor);

#define SERVO_ANGULO_CENTRO   90.0f
#define SERVO_ANGULO_MIN      50.0f
#define SERVO_ANGULO_MAX      130.0f
#define TIMEOUT_PERDA_LINHA   350
#define CICLO_CONTROLE_MS     30

struct ConfigControle {
  float kp;
  float kd;
  uint8_t threshold;          // 0 = Auto-Threshold Dinâmico, >0 = Fixo manual
  int velocidadeBase;
  int pwmMinimo;
  float pesoAntecipacao;      // Peso de E_far no controle composto (0.0 a 0.6)
  uint8_t modoOrientacao;      // 0 = Normal, 1 = Rotacionado 90°
  bool inverterServo;         // false = normal, true = inverte direção
  bool tracaoHabilitada;
};

ConfigControle g_config = {
  .kp = 0.65f,
  .kd = 0.35f,
  .threshold = 95,            // Threshold padrão para linha preta (ajustável via web)
  .velocidadeBase = 150,
  .pwmMinimo = 90,
  .pesoAntecipacao = 0.30f,   // 30% de peso no lookahead de curva
  .modoOrientacao = ORIENT_NORMAL_LANDSCAPE, // Padrão: Sensor Normal 160x120
  .inverterServo = false,
  .tracaoHabilitada = true
};

portMUX_TYPE g_configMux = portMUX_INITIALIZER_UNLOCKED;

struct TelemetriaRobo {
  float erroComposto;
  float erroBase;
  float erroMid;
  float erroFar;
  float curvatura;            // Delta_E = E_far - E_base
  uint8_t statusPista;        // TipoPista enum
  float anguloServo;
  int pwmAtual;
  bool linhaDetectada;
  bool zonaBaseOk;
  bool zonaMidOk;
  bool zonaFarOk;
  bool tracaoAtiva;
  uint32_t pixelsLinha;
  float fpsReal;
  uint8_t thresholdEfetivo;
  unsigned long timestamp;
};

volatile TelemetriaRobo g_telemetria = {
  .erroComposto = 0.0f,
  .erroBase = 0.0f,
  .erroMid = 0.0f,
  .erroFar = 0.0f,
  .curvatura = 0.0f,
  .statusPista = PISTA_SEM_LINHA,
  .anguloServo = 90.0f,
  .pwmAtual = 0,
  .linhaDetectada = false,
  .zonaBaseOk = false,
  .zonaMidOk = false,
  .zonaFarOk = false,
  .tracaoAtiva = true,
  .pixelsLinha = 0,
  .fpsReal = 0.0f,
  .thresholdEfetivo = 95,
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
httpd_handle_t g_httpd_web = NULL;    // Porta 80
httpd_handle_t g_httpd_stream = NULL; // Porta 81

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
  digitalWrite(PIN_MOTOR_IN4, LOW); // Proteção contra acionamento acidental no boot

  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, HIGH); // Apagado (Active LOW)

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
  config.fb_count     = 2;
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
    s->set_vflip(s, 0);
    s->set_hmirror(s, 1);
    s->set_brightness(s, 1);
    s->set_contrast(s, 2);
  }

  Serial.println("[CAMERA] OV2640 inicializada em QQVGA Grayscale.");
  return ESP_OK;
}

/* =====================================================================================
 * 6. PIPELINE DE VISÃO COMPUTACIONAL MULTI-ZONA E CONTROLE PREDITIVO (CORE 1)
 * ===================================================================================== */

// Função rápida para desenhar linhas na imagem Grayscale
void desenharLinhaGrayscale(uint8_t* buf, int x0, int y0, int x1, int y1, uint8_t cor) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy, e2;
  while (true) {
    if (x0 >= 0 && x0 < CAM_WIDTH && y0 >= 0 && y0 < CAM_HEIGHT) {
      buf[y0 * CAM_WIDTH + x0] = cor;
    }
    if (x0 == x1 && y0 == y1) break;
    e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

// Processador Multi-Zona adaptável à orientação da câmera
ResultadoVisaoMultiZona processarVisaoMultiZona(uint8_t* buf, uint8_t th_config, uint8_t orient, float peso_lookahead) {
  ResultadoVisaoMultiZona res;
  res.ok_base = false; res.ok_mid = false; res.ok_far = false;
  res.total_pixels = 0;

  // 1. Cálculo de Threshold Inteligente (Adaptativo ou Fixo)
  uint8_t th = th_config;
  if (th == 0) {
    // Modo adaptativo: calcula média amostral rápida
    uint32_t soma_amostra = 0;
    int pontos_amostra = 0;
    for (int idx = 0; idx < CAM_WIDTH * CAM_HEIGHT; idx += 32) {
      soma_amostra += buf[idx];
      pontos_amostra++;
    }
    uint8_t media_luz = (uint8_t)(soma_amostra / pontos_amostra);
    th = (media_luz > 60) ? (media_luz - 35) : 50;
  }
  res.threshold_usado = th;

  if (orient == ORIENT_ROTACIONADO_90) {
    /* =========================================================================
     * MODO ROTACIONADO 90° (Câmera montada na vertical no chassi):
     *  - Eixo Lateral da Pista (Esquerda <-> Direita): Sensor Y (0..119, Centro=60)
     *  - Eixo de Profundidade (Perto <-> Longe): Sensor X (0..159)
     *      * Base (Perto do carrinho): X de 125 a 145
     *      * Meio (Alcance intermediário): X de 75 a 95
     *      * Longe (Antecipação no horizonte): X de 25 a 45
     * ========================================================================= */
    const float CENTRO_LATERAL = 60.0f;
    uint64_t soma_base = 0, soma_mid = 0, soma_far = 0;
    uint32_t px_base = 0, px_mid = 0, px_far = 0;

    // Varredura da Zona 1 (Base: X 125..145)
    for (int x = 125; x <= 145; x++) {
      for (int y = 0; y < CAM_HEIGHT; y++) {
        if (buf[y * CAM_WIDTH + x] < th) {
          soma_base += y;
          px_base++;
        }
      }
    }

    // Varredura da Zona 2 (Meio: X 75..95)
    for (int x = 75; x <= 95; x++) {
      for (int y = 0; y < CAM_HEIGHT; y++) {
        if (buf[y * CAM_WIDTH + x] < th) {
          soma_mid += y;
          px_mid++;
        }
      }
    }

    // Varredura da Zona 3 (Longe: X 25..45)
    for (int x = 25; x <= 45; x++) {
      for (int y = 0; y < CAM_HEIGHT; y++) {
        if (buf[y * CAM_WIDTH + x] < th) {
          soma_far += y;
          px_far++;
        }
      }
    }

    res.total_pixels = px_base + px_mid + px_far;

    // Centróides
    if (px_base >= MIN_PIXELS_ZONA) {
      res.c_base = (float)soma_base / (float)px_base;
      res.e_base = res.c_base - CENTRO_LATERAL;
      res.ok_base = true;
    } else {
      res.c_base = CENTRO_LATERAL;
      res.e_base = 0.0f;
    }

    if (px_mid >= MIN_PIXELS_ZONA) {
      res.c_mid = (float)soma_mid / (float)px_mid;
      res.e_mid = res.c_mid - CENTRO_LATERAL;
      res.ok_mid = true;
    } else {
      res.c_mid = res.c_base;
      res.e_mid = res.e_base;
    }

    if (px_far >= MIN_PIXELS_ZONA) {
      res.c_far = (float)soma_far / (float)px_far;
      res.e_far = res.c_far - CENTRO_LATERAL;
      res.ok_far = true;
    } else {
      res.c_far = res.c_mid;
      res.e_far = res.e_mid;
    }

  } else {
    /* =========================================================================
     * MODO NORMAL LANDSCAPE (Câmera na horizontal padrão 160x120):
     *  - Eixo Lateral da Pista (Esquerda <-> Direita): Sensor X (0..159, Centro=80)
     *  - Eixo de Profundidade (Perto <-> Longe): Sensor Y (0..119)
     *      * Base (Perto): Y de 90 a 110
     *      * Meio (Intermediário): Y de 55 a 75
     *      * Longe (Antecipação): Y de 20 a 40
     * ========================================================================= */
    const float CENTRO_LATERAL = 80.0f;
    uint64_t soma_base = 0, soma_mid = 0, soma_far = 0;
    uint32_t px_base = 0, px_mid = 0, px_far = 0;

    // Zona 1 (Base: Y 90..110)
    for (int y = 90; y <= 110; y++) {
      uint8_t* row = &buf[y * CAM_WIDTH];
      for (int x = 0; x < CAM_WIDTH; x++) {
        if (row[x] < th) {
          soma_base += x;
          px_base++;
        }
      }
    }

    // Zona 2 (Meio: Y 55..75)
    for (int y = 55; y <= 75; y++) {
      uint8_t* row = &buf[y * CAM_WIDTH];
      for (int x = 0; x < CAM_WIDTH; x++) {
        if (row[x] < th) {
          soma_mid += x;
          px_mid++;
        }
      }
    }

    // Zona 3 (Longe: Y 20..40)
    for (int y = 20; y <= 40; y++) {
      uint8_t* row = &buf[y * CAM_WIDTH];
      for (int x = 0; x < CAM_WIDTH; x++) {
        if (row[x] < th) {
          soma_far += x;
          px_far++;
        }
      }
    }

    res.total_pixels = px_base + px_mid + px_far;

    // Centróides
    if (px_base >= MIN_PIXELS_ZONA) {
      res.c_base = (float)soma_base / (float)px_base;
      res.e_base = res.c_base - CENTRO_LATERAL;
      res.ok_base = true;
    } else {
      res.c_base = CENTRO_LATERAL;
      res.e_base = 0.0f;
    }

    if (px_mid >= MIN_PIXELS_ZONA) {
      res.c_mid = (float)soma_mid / (float)px_mid;
      res.e_mid = res.c_mid - CENTRO_LATERAL;
      res.ok_mid = true;
    } else {
      res.c_mid = res.c_base;
      res.e_mid = res.e_base;
    }

    if (px_far >= MIN_PIXELS_ZONA) {
      res.c_far = (float)soma_far / (float)px_far;
      res.e_far = res.c_far - CENTRO_LATERAL;
      res.ok_far = true;
    } else {
      res.c_far = res.c_mid;
      res.e_far = res.e_mid;
    }
  }

  // Validação Geral de Linha
  res.linha_valida = (res.ok_base || res.ok_mid || res.ok_far);

  // 2. Cálculo de Curvatura Preditiva (Delta_E = E_far - E_base)
  if (res.ok_far && res.ok_base) {
    res.curvatura = res.e_far - res.e_base;
  } else if (res.ok_mid && res.ok_base) {
    res.curvatura = (res.e_mid - res.e_base) * 1.8f;
  } else {
    res.curvatura = 0.0f;
  }

  // 3. Classificação Automática do Tipo de Pista
  if (!res.linha_valida) {
    res.tipo_pista = PISTA_SEM_LINHA;
  } else {
    float abs_curv = fabsf(res.curvatura);
    if (abs_curv < 10.0f) {
      res.tipo_pista = PISTA_RETA;
    } else if (res.curvatura > 0) {
      res.tipo_pista = (abs_curv >= 24.0f) ? PISTA_CURVA_FECH_DIR : PISTA_CURVA_SUAVE_DIR;
    } else {
      res.tipo_pista = (abs_curv >= 24.0f) ? PISTA_CURVA_FECH_ESQ : PISTA_CURVA_SUAVE_ESQ;
    }
  }

  // 4. Cálculo do Erro Composto Preditivo (Blend entre Base e Lookahead)
  if (res.ok_base && res.ok_far) {
    res.e_composto = (1.0f - peso_lookahead) * res.e_base + peso_lookahead * res.e_far;
  } else if (res.ok_base) {
    res.e_composto = res.e_base;
  } else if (res.ok_mid) {
    res.e_composto = res.e_mid;
  } else {
    res.e_composto = res.e_far;
  }

  return res;
}

// Desenha o Overlay Visual Completo (3 Zonas + Centróides + Trajetória)
void desenharOverlayVisualMultiZona(uint8_t* buf, const ResultadoVisaoMultiZona& vis, uint8_t orient) {
  if (orient == ORIENT_ROTACIONADO_90) {
    // --- Linhas Guia das 3 Zonas no Modo Rotacionado (X=135, X=85, X=35) ---
    // Zona Base (X=125..145)
    for (int y = 0; y < CAM_HEIGHT; y += 4) {
      buf[y * CAM_WIDTH + 125] = 220;
      buf[y * CAM_WIDTH + 145] = 220;
    }
    // Zona Meio (X=75..95)
    for (int y = 0; y < CAM_HEIGHT; y += 4) {
      buf[y * CAM_WIDTH + 75] = 180;
      buf[y * CAM_WIDTH + 95] = 180;
    }
    // Zona Longe (X=25..45)
    for (int y = 0; y < CAM_HEIGHT; y += 4) {
      buf[y * CAM_WIDTH + 25] = 140;
      buf[y * CAM_WIDTH + 45] = 140;
    }

    // Linha Central Ideal (Y=60)
    for (int x = 20; x <= 150; x += 3) {
      buf[60 * CAM_WIDTH + x] = 100;
    }

    // Trajetória Conectando os Centróides
    int y_base = (int)vis.c_base;
    int y_mid  = (int)vis.c_mid;
    int y_far  = (int)vis.c_far;

    if (vis.ok_base && vis.ok_mid) {
      desenharLinhaGrayscale(buf, 135, y_base, 85, y_mid, 255);
    }
    if (vis.ok_mid && vis.ok_far) {
      desenharLinhaGrayscale(buf, 85, y_mid, 35, y_far, 255);
    }

    // Marcadores dos Centróides (Cruzes Brancas)
    auto desenharCruz = [&](int x, int y) {
      if (y >= 1 && y < CAM_HEIGHT - 1 && x >= 1 && x < CAM_WIDTH - 1) {
        buf[y * CAM_WIDTH + x] = 255;
        buf[(y-1) * CAM_WIDTH + x] = 255;
        buf[(y+1) * CAM_WIDTH + x] = 255;
        buf[y * CAM_WIDTH + (x-1)] = 255;
        buf[y * CAM_WIDTH + (x+1)] = 255;
      }
    };

    if (vis.ok_base) desenharCruz(135, y_base);
    if (vis.ok_mid)  desenharCruz(85, y_mid);
    if (vis.ok_far)  desenharCruz(35, y_far);

  } else {
    // --- Linhas Guia no Modo Normal Landscape ---
    // Zona Base (Y=90..110)
    for (int x = 0; x < CAM_WIDTH; x += 4) {
      buf[90 * CAM_WIDTH + x] = 220;
      buf[110 * CAM_WIDTH + x] = 220;
    }
    // Zona Meio (Y=55..75)
    for (int x = 0; x < CAM_WIDTH; x += 4) {
      buf[55 * CAM_WIDTH + x] = 180;
      buf[75 * CAM_WIDTH + x] = 180;
    }
    // Zona Longe (Y=20..40)
    for (int x = 0; x < CAM_WIDTH; x += 4) {
      buf[20 * CAM_WIDTH + x] = 140;
      buf[40 * CAM_WIDTH + x] = 140;
    }

    // Linha Central Ideal (X=80)
    for (int y = 20; y <= 110; y += 3) {
      buf[y * CAM_WIDTH + 80] = 100;
    }

    int x_base = (int)vis.c_base;
    int x_mid  = (int)vis.c_mid;
    int x_far  = (int)vis.c_far;

    if (vis.ok_base && vis.ok_mid) {
      desenharLinhaGrayscale(buf, x_base, 100, x_mid, 65, 255);
    }
    if (vis.ok_mid && vis.ok_far) {
      desenharLinhaGrayscale(buf, x_mid, 65, x_far, 30, 255);
    }

    auto desenharCruz = [&](int x, int y) {
      if (y >= 1 && y < CAM_HEIGHT - 1 && x >= 1 && x < CAM_WIDTH - 1) {
        buf[y * CAM_WIDTH + x] = 255;
        buf[(y-1) * CAM_WIDTH + x] = 255;
        buf[(y+1) * CAM_WIDTH + x] = 255;
        buf[y * CAM_WIDTH + (x-1)] = 255;
        buf[y * CAM_WIDTH + (x+1)] = 255;
      }
    };

    if (vis.ok_base) desenharCruz(x_base, 100);
    if (vis.ok_mid)  desenharCruz(x_mid, 65);
    if (vis.ok_far)  desenharCruz(x_far, 30);
  }
}

void taskControleVisao(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(CICLO_CONTROLE_MS);

  float erro_anterior = 0.0f;
  unsigned long ultimo_tempo_linha_ms = millis();
  unsigned long tempo_anterior_fps = millis();
  int contador_frames_fps = 0;

  Serial.println("[CORE 1] Visão Computacional Multi-Zona e Controle Preditivo iniciados.");

  for (;;) {
    camera_fb_t *fb = esp_camera_fb_get();

    // Leitura Atômica das configurações do usuário
    portENTER_CRITICAL(&g_configMux);
    float cur_kp        = g_config.kp;
    float cur_kd        = g_config.kd;
    uint8_t cur_th      = g_config.threshold;
    int cur_speed       = g_config.velocidadeBase;
    int cur_min_pwm     = g_config.pwmMinimo;
    float cur_w_far     = g_config.pesoAntecipacao;
    uint8_t cur_orient  = g_config.modoOrientacao;
    bool inv_servo      = g_config.inverterServo;
    bool tracao_on      = g_config.tracaoHabilitada;
    portEXIT_CRITICAL(&g_configMux);

    if (fb != NULL && fb->format == PIXFORMAT_GRAYSCALE) {
      // 1. Processamento Multi-Zona em Tela Cheia
      ResultadoVisaoMultiZona vis = processarVisaoMultiZona(fb->buf, cur_th, cur_orient, cur_w_far);

      float angulo_servo = SERVO_ANGULO_CENTRO;
      int pwm_atuante = 0;

      if (vis.linha_valida) {
        ultimo_tempo_linha_ms = millis();
        float erro = vis.e_composto;
        float d_erro = erro - erro_anterior;

        // 2. Controlador PD Preditivo para o Servo
        if (inv_servo) {
          angulo_servo = SERVO_ANGULO_CENTRO - (cur_kp * erro + cur_kd * d_erro);
        } else {
          angulo_servo = SERVO_ANGULO_CENTRO + (cur_kp * erro + cur_kd * d_erro);
        }
        angulo_servo = constrain(angulo_servo, SERVO_ANGULO_MIN, SERVO_ANGULO_MAX);
        erro_anterior = erro;

        // 3. Controle Preditivo de Velocidade com Frenagem em Curvas
        // Utiliza tanto o erro imediato quanto a curvatura antecipada
        float desvio_total = fabsf(erro) * 0.6f + fabsf(vis.curvatura) * 0.4f;
        if (desvio_total > 15.0f) {
          float fator_reducao = (desvio_total - 15.0f) / 35.0f;
          fator_reducao = constrain(fator_reducao, 0.0f, 1.0f);
          pwm_atuante = cur_speed - (int)(fator_reducao * (cur_speed - cur_min_pwm));
        } else {
          pwm_atuante = cur_speed;
        }
        pwm_atuante = constrain(pwm_atuante, cur_min_pwm, 255);

        // Atuação nos Motores e Servo
        setAnguloServo(angulo_servo);
        if (tracao_on) {
          setMotoresAvanco(pwm_atuante, pwm_atuante);
        } else {
          pararMotores();
        }

      } else {
        // 4. Failsafe: Perda de Linha com Timeout Seguro
        if (millis() - ultimo_tempo_linha_ms > TIMEOUT_PERDA_LINHA) {
          angulo_servo = SERVO_ANGULO_CENTRO;
          pwm_atuante = 0;
          setAnguloServo(SERVO_ANGULO_CENTRO);
          pararMotores();
          erro_anterior = 0.0f;
        } else {
          // Mantém última tendência de curva em baixa velocidade durante perda momentânea
          if (inv_servo) {
            angulo_servo = SERVO_ANGULO_CENTRO - (cur_kp * erro_anterior);
          } else {
            angulo_servo = SERVO_ANGULO_CENTRO + (cur_kp * erro_anterior);
          }
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

      // 5. FPS Real
      contador_frames_fps++;
      if (millis() - tempo_anterior_fps >= 1000) {
        g_telemetria.fpsReal = (float)contador_frames_fps * 1000.0f / (float)(millis() - tempo_anterior_fps);
        contador_frames_fps = 0;
        tempo_anterior_fps = millis();
      }

      // 6. Atualização de Telemetria Compartilhada
      g_telemetria.erroComposto     = vis.e_composto;
      g_telemetria.erroBase         = vis.e_base;
      g_telemetria.erroMid          = vis.e_mid;
      g_telemetria.erroFar          = vis.e_far;
      g_telemetria.curvatura        = vis.curvatura;
      g_telemetria.statusPista      = vis.tipo_pista;
      g_telemetria.anguloServo      = angulo_servo;
      g_telemetria.pwmAtual         = tracao_on ? pwm_atuante : 0;
      g_telemetria.linhaDetectada   = vis.linha_valida;
      g_telemetria.zonaBaseOk       = vis.ok_base;
      g_telemetria.zonaMidOk        = vis.ok_mid;
      g_telemetria.zonaFarOk        = vis.ok_far;
      g_telemetria.tracaoAtiva      = tracao_on;
      g_telemetria.pixelsLinha      = vis.total_pixels;
      g_telemetria.thresholdEfetivo = vis.threshold_usado;
      g_telemetria.timestamp        = millis();

      // 7. Renderização do Overlay e Compartilhamento de Frame para Streaming
      desenharOverlayVisualMultiZona(fb->buf, vis, cur_orient);

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
      --bg-base: #090d16;
      --bg-panel: #111827;
      --bg-card: rgba(20, 30, 48, 0.75);
      --border-color: rgba(0, 240, 255, 0.2);
      --accent-cyan: #00f0ff;
      --accent-blue: #3b82f6;
      --accent-green: #10b981;
      --accent-yellow: #f59e0b;
      --accent-red: #ef4444;
      --text-main: #f9fafb;
      --text-muted: #9ca3af;
      --font-stack: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      --font-mono: "Courier New", monospace;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: var(--font-stack);
      background: radial-gradient(circle at 80% 10%, #1e293b 0%, var(--bg-base) 70%);
      color: var(--text-main);
      min-height: 100vh;
      padding: 16px;
    }
    .wrapper { max-width: 1250px; margin: 0 auto; }
    header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding-bottom: 14px;
      border-bottom: 1px solid var(--border-color);
      margin-bottom: 18px;
      flex-wrap: wrap;
      gap: 10px;
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
    .status-badge.curve {
      background: rgba(245, 158, 11, 0.15);
      border-color: rgba(245, 158, 11, 0.4);
      color: var(--accent-yellow);
    }
    .dot-pulse {
      width: 8px; height: 8px; border-radius: 50%;
      background: currentColor;
      box-shadow: 0 0 10px currentColor;
    }
    .main-grid {
      display: grid;
      grid-template-columns: 1.15fr 1fr;
      gap: 18px;
    }
    @media (max-width: 960px) { .main-grid { grid-template-columns: 1fr; } }
    .card {
      background: var(--bg-card);
      border: 1px solid var(--border-color);
      border-radius: 14px;
      padding: 16px;
      box-shadow: 0 10px 30px rgba(0,0,0,0.4);
      backdrop-filter: blur(8px);
      margin-bottom: 16px;
    }
    .card-head {
      font-size: 0.95rem;
      font-weight: 600;
      color: var(--accent-cyan);
      margin-bottom: 12px;
      display: flex;
      align-items: center;
      justify-content: space-between;
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
      gap: 10px;
      margin-top: 12px;
    }
    .btn {
      padding: 11px;
      border: none;
      border-radius: 8px;
      font-weight: 700;
      font-size: 0.9rem;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 6px;
      transition: all 0.2s ease;
    }
    .btn:active { transform: scale(0.97); }
    .btn-start {
      background: linear-gradient(135deg, #10b981 0%, #059669 100%);
      color: #fff;
      box-shadow: 0 0 15px rgba(16, 185, 129, 0.3);
    }
    .btn-stop {
      background: linear-gradient(135deg, #ef4444 0%, #dc2626 100%);
      color: #fff;
      box-shadow: 0 0 15px rgba(239, 68, 68, 0.3);
    }
    .track-status-box {
      background: rgba(0, 240, 255, 0.08);
      border: 1px solid var(--accent-cyan);
      border-radius: 10px;
      padding: 12px;
      text-align: center;
      margin-bottom: 12px;
    }
    .track-title { font-size: 0.75rem; text-transform: uppercase; color: var(--accent-cyan); font-weight: 700; }
    .track-val { font-size: 1.25rem; font-weight: 800; color: #fff; margin-top: 4px; font-family: var(--font-mono); }
    .zone-pills {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 8px;
      margin-top: 8px;
    }
    .zone-pill {
      background: rgba(255,255,255,0.05);
      border: 1px solid rgba(255,255,255,0.1);
      border-radius: 6px;
      padding: 6px;
      font-size: 0.75rem;
      text-align: center;
      font-weight: 600;
    }
    .zone-pill.active {
      background: rgba(16, 185, 129, 0.2);
      border-color: var(--accent-green);
      color: var(--accent-green);
    }
    .metrics-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 10px;
    }
    .metric-box {
      background: var(--bg-panel);
      border: 1px solid rgba(255,255,255,0.06);
      padding: 12px;
      border-radius: 10px;
    }
    .metric-label { font-size: 0.72rem; color: var(--text-muted); text-transform: uppercase; font-weight: 600; }
    .metric-val {
      font-family: var(--font-mono);
      font-size: 1.45rem;
      font-weight: 700;
      color: #fff;
      margin-top: 4px;
    }
    .metric-unit { font-size: 0.8rem; color: var(--text-muted); }
    .slider-item { margin-bottom: 10px; }
    .slider-header {
      display: flex;
      justify-content: space-between;
      font-size: 0.82rem;
      margin-bottom: 4px;
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
      margin-top: 6px;
    }
    .btn-toggle-group {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      margin-top: 6px;
    }
    .btn-toggle {
      padding: 8px;
      background: #1e293b;
      border: 1px solid rgba(255,255,255,0.1);
      border-radius: 6px;
      color: #fff;
      font-size: 0.8rem;
      cursor: pointer;
      text-align: center;
    }
    .btn-toggle.active {
      background: var(--accent-cyan);
      color: #000;
      font-weight: 700;
      border-color: var(--accent-cyan);
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
      z-index: 100;
    }
    .toast.show { opacity: 1; }
  </style>
</head>
<body>
  <div class="wrapper">
    <header>
      <div class="logo-area">
        <h1>🏎️ Carrinho Seguidor ESP32-CAM</h1>
        <div class="logo-sub">VISÃO MULTI-ZONA // CONTROLE PREDITIVO</div>
      </div>
      <div id="badgeStatus" class="status-badge">
        <div class="dot-pulse"></div>
        <span id="txtStatusBadge">RODANDO</span>
      </div>
    </header>

    <div class="main-grid">
      <div>
        <div class="card">
          <div class="card-head">
            <span>📷 Visão Computacional Embarcada (Porta 81)</span>
            <span style="font-size: 0.75rem; color: var(--text-muted);">Overlay: 3 Zonas + Curva</span>
          </div>
          <div class="stream-box">
            <img id="streamImg" alt="Streaming MJPEG ESP32-CAM">
          </div>
          <div class="btn-group">
            <button class="btn btn-start" onclick="enviarComando('start')">▶ INICIAR TRAÇÃO</button>
            <button class="btn btn-stop" onclick="enviarComando('stop')">🛑 PARAR EMERGÊNCIA</button>
          </div>
        </div>

        <div class="card">
          <div class="track-status-box">
            <div class="track-title">Classificação da Trajetória</div>
            <div class="track-val" id="lblTrackStatus">RETA</div>
            <div class="zone-pills">
              <div id="pillBase" class="zone-pill">🔴 Base (Perto)</div>
              <div id="pillMid" class="zone-pill">🔴 Meio</div>
              <div id="pillFar" class="zone-pill">🔴 Longe (Curva)</div>
            </div>
          </div>

          <div class="metrics-grid">
            <div class="metric-box">
              <div class="metric-label">Taxa de Quadros</div>
              <div class="metric-val" id="valFps">-- <span class="metric-unit">FPS</span></div>
            </div>
            <div class="metric-box">
              <div class="metric-label">Curvatura Antecipada (ΔE)</div>
              <div class="metric-val" id="valCurv">0.0 <span class="metric-unit">px</span></div>
            </div>
            <div class="metric-box">
              <div class="metric-label">Erro Composto (Blend)</div>
              <div class="metric-val" id="valErro">-- <span class="metric-unit">px</span></div>
            </div>
            <div class="metric-box">
              <div class="metric-label">Ângulo do Servo</div>
              <div class="metric-val" id="valAngulo">90.0<span class="metric-unit">°</span></div>
            </div>
            <div class="metric-box">
              <div class="metric-label">PWM Motores</div>
              <div class="metric-val" id="valPwm">0 <span class="metric-unit">/255</span></div>
            </div>
            <div class="metric-box">
              <div class="metric-label">Threshold Ativo</div>
              <div class="metric-val" id="valTh">-- <span class="metric-unit">val</span></div>
            </div>
          </div>
        </div>
      </div>

      <div>
        <div class="card">
          <div class="card-head">⚙️ Orientação do Sensor & Calibração (/tune)</div>

          <div style="margin-bottom: 12px;">
            <div style="font-size: 0.8rem; color: var(--text-muted); margin-bottom: 4px;">Orientação da Câmera no Chassi:</div>
            <div class="btn-toggle-group">
              <button id="btnOrientNorm" class="btn-toggle active" onclick="setOrientacao(0)">🖥️ Sensor Normal 160x120</button>
              <button id="btnOrientRot" class="btn-toggle" onclick="setOrientacao(1)">📐 Sensor Rotacionado 90°</button>
            </div>
          </div>

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
                <span>Threshold Binarização (0 = Auto)</span>
                <span id="lblTh">95</span>
              </div>
              <input type="range" id="rngTh" min="0" max="220" step="1" value="95" oninput="atualizarLabel('lblTh', this.value)">
            </div>

            <div class="slider-item">
              <div class="slider-header">
                <span>Velocidade Base (PWM Reta)</span>
                <span id="lblSpeed">150</span>
              </div>
              <input type="range" id="rngSpeed" min="60" max="255" step="5" value="150" oninput="atualizarLabel('lblSpeed', this.value)">
            </div>

            <div class="slider-item">
              <div class="slider-header">
                <span>Peso Antecipação de Curva (w_far)</span>
                <span id="lblWFar">0.30</span>
              </div>
              <input type="range" id="rngWFar" min="0.0" max="0.6" step="0.05" value="0.30" oninput="atualizarLabel('lblWFar', this.value)">
            </div>

            <div style="margin-top: 10px; display: flex; align-items: center; justify-content: space-between;">
              <span style="font-size: 0.82rem;">Inverter Sentido do Servo:</span>
              <input type="checkbox" id="chkInvServo" style="width: 18px; height: 18px; accent-color: var(--accent-cyan);">
            </div>

            <button class="btn-apply" onclick="enviarCalibracao()">💾 APLICAR PARÂMETROS</button>
          </div>
        </div>

        <div class="card">
          <div class="card-head">ℹ️ Informações do Sistema</div>
          <div style="font-size: 0.85rem; line-height: 1.6; color: var(--text-muted);">
            <div>• <b>IP do Robô:</b> <span id="lblIpHost">10.164.64.50</span></div>
            <div>• <b>Portas:</b> 80 (Painel Web) / 81 (Streaming MJPEG)</div>
            <div>• <b>Visão:</b> Multi-ROI 3 Níveis (Perto, Meio, Longe)</div>
            <div>• <b>Frenagem:</b> Preditiva por Vetor de Curvatura</div>
            <div>• <b>Loop de Controle:</b> Core 1 (~30 ms)</div>
          </div>
        </div>
      </div>
    </div>
  </div>

  <div id="toast" class="toast">Comando executado</div>

  <script>
    let curOrient = 0;

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

    function setOrientacao(val) {
      curOrient = val;
      document.getElementById('btnOrientRot').className = val === 1 ? 'btn-toggle active' : 'btn-toggle';
      document.getElementById('btnOrientNorm').className = val === 0 ? 'btn-toggle active' : 'btn-toggle';
      enviarCalibracao();
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
      const wfar = document.getElementById('rngWFar').value;
      const invServo = document.getElementById('chkInvServo').checked ? 1 : 0;

      try {
        const url = `/tune?kp=${kp}&kd=${kd}&th=${th}&speed=${speed}&wfar=${wfar}&orient=${curOrient}&inv_servo=${invServo}`;
        const res = await fetch(url);
        if (res.ok) {
          showToast('💾 Parâmetros salvos com sucesso!');
        }
      } catch (e) {
        showToast('Erro ao calibrar');
      }
    }

    const TRACK_STR = [
      "SEM LINHA",
      "🏎️ RETA",
      "↩️ CURVA SUAVE À ESQUERDA",
      "↪️ CURVA SUAVE À DIREITA",
      "⚡ CURVA FECHADA À ESQUERDA",
      "⚡ CURVA FECHADA À DIREITA"
    ];

    async function pollingTelemetria() {
      try {
        const resp = await fetch('/data');
        if (resp.ok) {
          const d = await resp.json();
          document.getElementById('valFps').innerHTML = d.fps.toFixed(1) + ' <span class="metric-unit">FPS</span>';
          document.getElementById('valErro').innerHTML = (d.erro >= 0 ? '+' : '') + d.erro.toFixed(1) + ' <span class="metric-unit">px</span>';
          document.getElementById('valCurv').innerHTML = (d.curvatura >= 0 ? '+' : '') + d.curvatura.toFixed(1) + ' <span class="metric-unit">px</span>';
          document.getElementById('valAngulo').innerHTML = d.angulo.toFixed(1) + '<span class="metric-unit">°</span>';
          document.getElementById('valPwm').innerHTML = d.pwm + ' <span class="metric-unit">/255</span>';
          document.getElementById('valTh').innerHTML = d.th + ' <span class="metric-unit">efetivo</span>';

          // Status de Pista
          const stIndex = d.status_pista || 0;
          document.getElementById('lblTrackStatus').innerText = TRACK_STR[stIndex] || "DESCONHECIDO";

          // Pílulas das 3 Zonas
          document.getElementById('pillBase').className = d.base_ok ? 'zone-pill active' : 'zone-pill';
          document.getElementById('pillBase').innerText = d.base_ok ? '🟢 Base (Perto)' : '🔴 Base (Perto)';

          document.getElementById('pillMid').className = d.mid_ok ? 'zone-pill active' : 'zone-pill';
          document.getElementById('pillMid').innerText = d.mid_ok ? '🟢 Meio' : '🔴 Meio';

          document.getElementById('pillFar').className = d.far_ok ? 'zone-pill active' : 'zone-pill';
          document.getElementById('pillFar').innerText = d.far_ok ? '🟢 Longe (Curva)' : '🔴 Longe (Curva)';

          // Badge Header
          const badge = document.getElementById('badgeStatus');
          const txtBadge = document.getElementById('txtStatusBadge');
          if (d.status === 'RODANDO') {
            if (stIndex >= 4) {
              badge.className = 'status-badge curve';
              txtBadge.innerText = 'FRENAGEM CURVA';
            } else {
              badge.className = 'status-badge';
              txtBadge.innerText = d.detectada ? 'RASTREANDO' : 'BUSCANDO LINHA';
            }
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
  char json[384];
  const char* status_str = g_telemetria.tracaoAtiva ? (g_telemetria.linhaDetectada ? "RODANDO" : "LINHA PERDIDA") : "PARADO";

  snprintf(json, sizeof(json),
    "{\"erro\":%.2f,\"erro_base\":%.2f,\"erro_mid\":%.2f,\"erro_far\":%.2f,\"curvatura\":%.2f,"
    "\"status_pista\":%u,\"angulo\":%.1f,\"pwm\":%d,\"fps\":%.1f,\"detectada\":%s,"
    "\"base_ok\":%s,\"mid_ok\":%s,\"far_ok\":%s,\"status\":\"%s\",\"th\":%u,\"pixels\":%u}",
    g_telemetria.erroComposto,
    g_telemetria.erroBase,
    g_telemetria.erroMid,
    g_telemetria.erroFar,
    g_telemetria.curvatura,
    g_telemetria.statusPista,
    g_telemetria.anguloServo,
    g_telemetria.pwmAtual,
    g_telemetria.fpsReal,
    g_telemetria.linhaDetectada ? "true" : "false",
    g_telemetria.zonaBaseOk ? "true" : "false",
    g_telemetria.zonaMidOk ? "true" : "false",
    g_telemetria.zonaFarOk ? "true" : "false",
    status_str,
    g_telemetria.thresholdEfetivo,
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
      if (httpd_query_key_value(buf, "wfar", param, sizeof(param)) == ESP_OK) {
        g_config.pesoAntecipacao = atof(param);
      }
      if (httpd_query_key_value(buf, "orient", param, sizeof(param)) == ESP_OK) {
        g_config.modoOrientacao = (uint8_t)atoi(param);
      }
      if (httpd_query_key_value(buf, "inv_servo", param, sizeof(param)) == ESP_OK) {
        g_config.inverterServo = (atoi(param) == 1);
      }
      portEXIT_CRITICAL(&g_configMux);

      Serial.printf("[TUNE] Kp=%.2f, Kd=%.2f, Th=%d, Speed=%d, WFar=%.2f, Orient=%d, InvServo=%d\n",
        g_config.kp, g_config.kd, g_config.threshold, g_config.velocidadeBase,
        g_config.pesoAntecipacao, g_config.modoOrientacao, g_config.inverterServo);
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
          Serial.println("[CMD] Tração Habilitada (START).");
        } else if (strcmp(action, "stop") == 0) {
          g_config.tracaoHabilitada = false;
          pararMotores();
          Serial.println("[CMD] Tração Desabilitada (STOP).");
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
    Serial.println("[WIFI] Falha ao configurar IP Estático!");
  } else {
    Serial.println("[WIFI] Usando IP Estático configurado.");
  }
#endif

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int tentativas = 0;
  while (WiFi.status() != WL_CONNECTED && tentativas < 25) {
    digitalWrite(PIN_LED_STATUS, (tentativas % 2 == 0) ? LOW : HIGH);
    delay(400);
    Serial.print(".");
    tentativas++;
  }

  // Fallback inteligente via DHCP
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[WIFI] Tentando reconexão automática via DHCP...");
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
    digitalWrite(PIN_LED_STATUS, LOW); // LED aceso fixo = Wi-Fi Conectado!
    if (MDNS.begin("carrinho")) {
      MDNS.addService("http", "tcp", 80);
      Serial.println("[MDNS] Respondedor mDNS iniciado: http://carrinho.local/");
    }

    Serial.println("\n========================================================");
    Serial.println("[WIFI] CONECTADO COM SUCESSO AO HOTSPOT!");
    Serial.printf("[WIFI] Endereço do Painel: http://%s/\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WIFI] Link alternativo:   http://carrinho.local/\n");
    Serial.printf("[WIFI] Stream da Câmera:   http://%s:81/stream\n", WiFi.localIP().toString().c_str());
    Serial.println("========================================================");
  } else {
    digitalWrite(PIN_LED_STATUS, HIGH);
    Serial.println("\n[WIFI] Hotspot não encontrado. Executando em modo autônomo.");
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
  Serial.println("  ROBÔ SEGUIDOR DE LINHA AUTÔNOMO - ESP32-CAM OV2640");
  Serial.println("  VISÃO MULTI-ZONA & RECONHECIMENTO PREDITIVO DE CURVAS");
  Serial.println("========================================================");

  // 1. Inicializa pinagem segura
  inicializarHardwareSeguro();
  setAnguloServo(SERVO_ANGULO_CENTRO);
  pararMotores();

  // 2. Cria Mutex para o buffer de imagem
  g_shared_frame.mutex = xSemaphoreCreateMutex();

  // 3. Inicializa Câmera OV2640
  if (inicializarCamera() != ESP_OK) {
    Serial.println("[FATAL] Falha na câmera. Reiniciando...");
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

  Serial.println("[SISTEMA] Inicialização concluída com sucesso.\n");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(PIN_LED_STATUS, HIGH);
    static unsigned long ultimo_reconnect = 0;
    if (millis() - ultimo_reconnect > 5000) {
      ultimo_reconnect = millis();
      Serial.println("[WIFI] Reconectando ao Hotspot...");
      WiFi.reconnect();
    }
  } else {
    digitalWrite(PIN_LED_STATUS, LOW);
  }

  vTaskDelay(pdMS_TO_TICKS(1000));
}
