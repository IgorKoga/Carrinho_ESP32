/*
 * =====================================================================================
 * FIRMWARE INTEGRAL: CARRINHO SEGUIDOR DE LINHA AUTÔNOMO COM ESP32-CAM
 * (AI-THINKER OV2640)
 * =====================================================================================
 *
 * Arquitetura Dual-Core FreeRTOS & Servidor Web Dual-Port (Anti-Bloqueio de
 * Streaming):
 *
 *  1. CORE 1 (Visão Computacional Multi-Zona & Controle PD Preditivo - ~30 ms):
 *     - Captura QQVGA (160x120) em Grayscale de 8 bits.
 *     - Processamento Multi-ROI em 3 Zonas de Profundidade (Tela Cheia):
 *         * Zona 1 (Base / Perto): Resposta imediata de erro lateral (E_base).
 *         * Zona 2 (Meio / Intermediário): Estabilização de trajetória (E_mid).
 *         * Zona 3 (Longe / Lookahead): Antecipação de curvas (E_far).
 *     - Reconhecimento Preditivo de Curvas:
 *         * Cálculo de Curvatura: Delta_E = E_far - E_base.
 *         * Classificação de Pista: RETA, CURVA SUAVE (ESQ/DIR), CURVA FECHADA
 * (ESQ/DIR).
 *         * Frenagem Antecipada Preditiva antes de entrar na curva.
 *         * Blend de Esterçamento Composto: E_total = (1 - w_far)*E_base +
 * w_far*E_far.
 *     - Suporte dinâmico à Orientação do Sensor (Modo 90° Rotacionado / Modo
 * Normal).
 *     - Thresholding Adaptativo Dinâmico para máxima robustez à iluminação.
 *     - Overlay Visual em Tempo Real com marcação das 3 zonas e vetor da curva
 * no Stream.
 *
 *  2. CORE 0 (Servidor Web & Streaming Dual-Port de Alto Desempenho):
 *     - Conexão como STA em Hotspot móvel com IP Estático e fallback DHCP.
 *     - Porta 80: Painel Web Dark Theme, Telemetria JSON (/data), Calibração
 * (/tune), Comandos (/cmd).
 *     - Porta 81: Servidor Dedicado de Streaming MJPEG (/stream) com overlay
 * visual completo.
 *
 *  3. HARDWARE & SEGURANÇA ELÉTRICA:
 *     - Servo de Direção: GPIO 02 via LEDC (50 Hz, 14 bits).
 *     - Ponte H Traseira: IN1=GPIO 14, IN2=GPIO 15, IN3=GPIO 13, IN4=GPIO 12.
 *     - Proteção de Boot: GPIO 12 e demais pinos inicializados em LOW
 * imediatamente.
 * =====================================================================================
 */

#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "img_converters.h"
#include "index_html.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"
#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>

/* =====================================================================================
 * 1. CONFIGURAÇÕES DE REDE WI-FI DINÂMICA (DHCP / CORE 0)
 * =====================================================================================
 */

// --- Credenciais do Hotspot Móvel / Roteador Wi-Fi ---
const char *WIFI_SSID = "firula";
const char *WIFI_PASS = "bebop123";

/* =====================================================================================
 * 2. PINAGEM DO HARDWARE (ESP32-CAM AI-THINKER)
 * =====================================================================================
 */

// --- Câmera OV2640 (AI-Thinker) ---
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// --- Atuadores: Servo Dianteiro e Motores DC Traseiros ---
#define PIN_SERVO 2      // GPIO 02: Servo de direção dianteira
#define PIN_MOTOR_IN1 14 // GPIO 14: Motor Esquerdo Avanço (PWM)
#define PIN_MOTOR_IN2 15 // GPIO 15: Motor Esquerdo Direção (GND/LOW)
#define PIN_MOTOR_IN3 13 // GPIO 13: Motor Direito Avanço (PWM)
#define PIN_MOTOR_IN4 12 // GPIO 12: Motor Direito Direção (GND/LOW)
#define PIN_LED_STATUS                                                         \
  33 // GPIO 33: LED Vermelho Onboard (Active LOW - mantido apagado)
#define PIN_FLASH 4 // GPIO 04: Flash LED de Alta Potência (Active HIGH)

// --- Canais PWM LEDC ---
// Alocação em canais e timers independentes para não colidir com o clock da
// câmera OV2640 (LEDC Channel 0 / Timer 0):
#define LEDC_CH_SERVO 2       // Canal 2 (Timer 1: 50 Hz, 14 bits)
#define LEDC_CH_MOTOR_LEFT 4  // Canal 4 (Timer 2: 5 kHz, 8 bits)
#define LEDC_CH_MOTOR_RIGHT 5 // Canal 5 (Timer 2: 5 kHz, 8 bits)
#define LEDC_FREQ_SERVO 50    // 50 Hz para Servo Motor
#define LEDC_RES_SERVO 14     // 14 bits (0..16383)
#define LEDC_FREQ_MOTOR 5000  // 5 kHz para Ponte H DC
#define LEDC_RES_MOTOR 8      // 8 bits (0..255)

/* =====================================================================================
 * 3. CONSTANTES, ESTRUTURAS E PROTÓTIPOS (THREAD-SAFE)
 * =====================================================================================
 */

#define CAM_WIDTH 160
#define CAM_HEIGHT 120
#define MIN_PIXELS_ZONA 4

// Modos de Orientação da Câmera
enum ModoOrientacao {
  ORIENT_NORMAL_LANDSCAPE = 0, // Câmera na horizontal (160x120)
  ORIENT_ROTACIONADO_90 =
      1 // Câmera na vertical/rotacionada (120x160) - Padrão AI-Thinker vertical
};

// Classificação de Pista / Curvas
enum TipoPista {
  PISTA_SEM_LINHA = 0,
  PISTA_RETA = 1,
  PISTA_CURVA_SUAVE_ESQ = 2,
  PISTA_CURVA_SUAVE_DIR = 3,
  PISTA_CURVA_FECH_ESQ = 4,
  PISTA_CURVA_FECH_DIR = 5
};

// Estrutura para busca em janela de interesse (Spatial ROI Window)
struct ResultadoVarreduraLinha {
  float centroide;
  uint32_t pixels;
  bool ok;
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

// Protótipos para o preprocessador do Arduino IDE e compilador C++
void inicializarHardwareSeguro();
void piscarFlash(int vezes, int tempo_ms = 90);
void setAnguloServo(float angulo);
void setMotoresAvanco(int pwm_esq, int pwm_dir);
void pararMotores();
esp_err_t inicializarCamera();
void desenharLinhaGrayscale(uint8_t *buf, int x0, int y0, int x1, int y1,
                            uint8_t cor);
uint8_t calcularThresholdDinamico(uint8_t *buf, uint8_t orient);
ResultadoVarreduraLinha varrerZonaHorizontal(uint8_t *buf, int y_start,
                                             int y_end, int x_min, int x_max,
                                             uint8_t th);
ResultadoVarreduraLinha varrerZonaVertical(uint8_t *buf, int x_start, int x_end,
                                           int y_min, int y_max, uint8_t th);
ResultadoVisaoMultiZona processarVisaoMultiZona(uint8_t *buf, uint8_t th_config,
                                                uint8_t orient,
                                                float peso_lookahead);
void desenharOverlayVisualMultiZona(uint8_t *buf,
                                    const ResultadoVisaoMultiZona &vis,
                                    uint8_t orient);
void taskControleVisao(void *pvParameters);
void iniciarServidoresWeb();
void conectarWiFiDHCP();

#define SERVO_ANGULO_CENTRO 90.0f
#define SERVO_ANGULO_MIN 50.0f
#define SERVO_ANGULO_MAX 130.0f
#define TIMEOUT_PERDA_LINHA 350
#define CICLO_CONTROLE_MS 30

struct ConfigControle {
  float kp;
  float kd;
  uint8_t threshold; // 0 = Auto-Threshold Dinâmico, >0 = Fixo manual
  int velocidadeBase;
  int pwmMinimo;
  float pesoAntecipacao;  // Peso de E_far no controle composto (0.0 a 0.6)
  uint8_t modoOrientacao; // 0 = Normal, 1 = Rotacionado 90°
  bool inverterServo;     // false = normal, true = inverte direção
  bool tracaoHabilitada;
};

ConfigControle g_config = {
    .kp = 0.65f,
    .kd = 0.35f,
    .threshold = 0, // 0 = Auto-Threshold Inteligente por Contraste Min-Max
                    // (Padrão), >0 = Fixo manual
    .velocidadeBase = 150,
    .pwmMinimo = 90,
    .pesoAntecipacao = 0.30f, // 30% de peso no lookahead de curva
    .modoOrientacao = ORIENT_NORMAL_LANDSCAPE, // Padrão: Sensor Normal 160x120
    .inverterServo = false,
    .tracaoHabilitada = true};

portMUX_TYPE g_configMux = portMUX_INITIALIZER_UNLOCKED;

struct TelemetriaRobo {
  float erroComposto;
  float erroBase;
  float erroMid;
  float erroFar;
  float curvatura;     // Delta_E = E_far - E_base
  uint8_t statusPista; // TipoPista enum
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

TelemetriaRobo g_telemetria = {.erroComposto = 0.0f,
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
                               .timestamp = 0};

struct SharedFrame {
  uint8_t *jpg_buf;
  size_t jpg_len;
  SemaphoreHandle_t mutex;
};

SharedFrame g_shared_frame = {.jpg_buf = NULL, .jpg_len = 0, .mutex = NULL};

TaskHandle_t g_taskVisaoHandle = NULL;
httpd_handle_t g_httpd_web = NULL;    // Porta 80
httpd_handle_t g_httpd_stream = NULL; // Porta 81

/* =====================================================================================
 * 4. ATUAÇÃO E CONTROLE DE HARDWARE (LEDC / MOTORES / SERVO)
 * =====================================================================================
 */

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
  digitalWrite(PIN_MOTOR_IN4,
               LOW); // Proteção contra acionamento acidental no boot

  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, HIGH); // Apagado 100% (Active LOW)

  pinMode(PIN_FLASH, OUTPUT);
  digitalWrite(PIN_FLASH, LOW); // Flash desligado

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

/**
 * @brief Pisca o LED Flash de alta potência um número especificado de vezes.
 */
void piscarFlash(int vezes, int tempo_ms) {
  for (int i = 0; i < vezes; i++) {
    digitalWrite(PIN_FLASH, HIGH);
    delay(tempo_ms);
    digitalWrite(PIN_FLASH, LOW);
    if (i < vezes - 1) {
      delay(tempo_ms);
    }
  }
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
 * =====================================================================================
 */

esp_err_t inicializarCamera() {
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QQVGA; // 160 x 120 pixels
  config.jpeg_quality = 12;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

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
 * =====================================================================================
 */

// Função rápida para desenhar linhas na imagem Grayscale
void desenharLinhaGrayscale(uint8_t *buf, int x0, int y0, int x1, int y1,
                            uint8_t cor) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy, e2;
  while (true) {
    if (x0 >= 0 && x0 < CAM_WIDTH && y0 >= 0 && y0 < CAM_HEIGHT) {
      buf[y0 * CAM_WIDTH + x0] = cor;
    }
    if (x0 == x1 && y0 == y1)
      break;
    e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

// Cálculo de threshold dinâmico com base no contraste real da pista (Min-Max)
uint8_t calcularThresholdDinamico(uint8_t *buf, uint8_t orient) {
  uint8_t min_val = 255;
  uint8_t max_val = 0;

  // Amostragem na região útil central (evita bordas com chão/vinhetagem)
  if (orient == ORIENT_ROTACIONADO_90) {
    for (int x = 30; x <= 130; x += 4) {
      for (int y = 18; y <= CAM_HEIGHT - 19; y += 4) {
        uint8_t p = buf[y * CAM_WIDTH + x];
        if (p < min_val)
          min_val = p;
        if (p > max_val)
          max_val = p;
      }
    }
  } else {
    for (int y = 25; y <= 105; y += 4) {
      uint8_t *row = &buf[y * CAM_WIDTH];
      for (int x = 18; x <= CAM_WIDTH - 19; x += 4) {
        uint8_t p = row[x];
        if (p < min_val)
          min_val = p;
        if (p > max_val)
          max_val = p;
      }
    }
  }

  // Se houver contraste real entre a fita preta e a superfície branca
  if (max_val > min_val && (max_val - min_val) > 28) {
    // Limiar no terço inferior (35% entre o preto mais escuro e o branco mais
    // claro)
    return (uint8_t)(min_val + (max_val - min_val) * 0.35f);
  }
  return 60; // Fallback seguro
}

// Varredura de Zona Horizontal (Modo Landscape) com janela e proteção lateral
ResultadoVarreduraLinha varrerZonaHorizontal(uint8_t *buf, int y_start,
                                             int y_end, int x_min, int x_max,
                                             uint8_t th) {
  ResultadoVarreduraLinha r = {0.0f, 0, false};
  uint64_t soma_x = 0;

  y_start = constrain(y_start, 0, CAM_HEIGHT - 1);
  y_end = constrain(y_end, 0, CAM_HEIGHT - 1);
  if (y_start > y_end) {
    int tmp = y_start;
    y_start = y_end;
    y_end = tmp;
  }

  // Margem de proteção lateral contra vinhetagem e piso externo (descarta
  // bordas)
  x_min = constrain(x_min, 14, CAM_WIDTH - 15);
  x_max = constrain(x_max, 14, CAM_WIDTH - 15);
  if (x_min > x_max) {
    int tmp = x_min;
    x_min = x_max;
    x_max = tmp;
  }

  for (int y = y_start; y <= y_end; y++) {
    uint8_t *row = &buf[y * CAM_WIDTH];
    for (int x = x_min; x <= x_max; x++) {
      if (row[x] < th) {
        soma_x += x;
        r.pixels++;
      }
    }
  }

  if (r.pixels >= MIN_PIXELS_ZONA) {
    r.centroide = (float)soma_x / (float)r.pixels;
    r.ok = true;
  }
  return r;
}

// Varredura de Zona Vertical (Modo Rotacionado 90°) com janela e proteção
// lateral
ResultadoVarreduraLinha varrerZonaVertical(uint8_t *buf, int x_start, int x_end,
                                           int y_min, int y_max, uint8_t th) {
  ResultadoVarreduraLinha r = {0.0f, 0, false};
  uint64_t soma_y = 0;

  x_start = constrain(x_start, 0, CAM_WIDTH - 1);
  x_end = constrain(x_end, 0, CAM_WIDTH - 1);
  if (x_start > x_end) {
    int tmp = x_start;
    x_start = x_end;
    x_end = tmp;
  }

  // Margem de proteção lateral
  y_min = constrain(y_min, 12, CAM_HEIGHT - 13);
  y_max = constrain(y_max, 12, CAM_HEIGHT - 13);
  if (y_min > y_max) {
    int tmp = y_min;
    y_min = y_max;
    y_max = tmp;
  }

  for (int x = x_start; x <= x_end; x++) {
    for (int y = y_min; y <= y_max; y++) {
      if (buf[y * CAM_WIDTH + x] < th) {
        soma_y += y;
        r.pixels++;
      }
    }
  }

  if (r.pixels >= MIN_PIXELS_ZONA) {
    r.centroide = (float)soma_y / (float)r.pixels;
    r.ok = true;
  }
  return r;
}

// Processador Multi-Zona adaptável com Rastreamento Espacial Contínuo (Dynamic
// ROI)
ResultadoVisaoMultiZona processarVisaoMultiZona(uint8_t *buf, uint8_t th_config,
                                                uint8_t orient,
                                                float peso_lookahead) {
  ResultadoVisaoMultiZona res = {};
  res.ok_base = false;
  res.ok_mid = false;
  res.ok_far = false;
  res.total_pixels = 0;

  // 1. Cálculo de Threshold Inteligente (Adaptativo por Contraste Real ou Fixo)
  uint8_t th =
      (th_config == 0) ? calcularThresholdDinamico(buf, orient) : th_config;
  res.threshold_usado = th;

  // Histórico de rastreamento estático (continuidade espacial entre quadros)
  static float s_last_base = 80.0f;
  static bool s_has_lock = false;

  if (orient == ORIENT_ROTACIONADO_90) {
    /* =========================================================================
     * MODO ROTACIONADO 90° (Câmera montada na vertical no chassi):
     *  - Eixo Lateral da Pista: Sensor Y (0..119, Centro=60)
     *  - Eixo de Profundidade: Sensor X (0..159)
     *      * Base (Perto): X 125..145
     *      * Meio (Intermediário): X 75..95
     *      * Longe (Antecipação): X 25..45
     * =========================================================================
     */
    const float CENTRO_LATERAL = 60.0f;
    const int SAFE_MIN_Y = 12;
    const int SAFE_MAX_Y = 107;

    // --- 1. Zona Base (X: 125..145) ---
    ResultadoVarreduraLinha r_base;
    if (s_has_lock) {
      r_base = varrerZonaVertical(buf, 125, 145, (int)(s_last_base - 36),
                                  (int)(s_last_base + 36), th);
      if (!r_base.ok) {
        r_base = varrerZonaVertical(buf, 125, 145, SAFE_MIN_Y, SAFE_MAX_Y, th);
      }
    } else {
      r_base = varrerZonaVertical(buf, 125, 145, SAFE_MIN_Y, SAFE_MAX_Y, th);
    }

    if (r_base.ok) {
      res.c_base = r_base.centroide;
      res.e_base = res.c_base - CENTRO_LATERAL;
      res.ok_base = true;
      s_last_base = 0.75f * res.c_base + 0.25f * s_last_base;
      s_has_lock = true;
    } else {
      res.ok_base = false;
    }

    // --- 2. Zona Meio (X: 75..95) - Rastreamento contínuo a partir da Base ---
    float ref_mid =
        res.ok_base ? res.c_base : (s_has_lock ? s_last_base : CENTRO_LATERAL);
    ResultadoVarreduraLinha r_mid = varrerZonaVertical(
        buf, 75, 95, (int)(ref_mid - 32), (int)(ref_mid + 32), th);
    if (!r_mid.ok && !res.ok_base) {
      r_mid = varrerZonaVertical(buf, 75, 95, SAFE_MIN_Y, SAFE_MAX_Y, th);
    }

    if (r_mid.ok) {
      res.c_mid = r_mid.centroide;
      res.e_mid = res.c_mid - CENTRO_LATERAL;
      res.ok_mid = true;
    } else {
      res.ok_mid = false;
      res.c_mid = res.ok_base ? res.c_base : CENTRO_LATERAL;
      res.e_mid = res.ok_base ? res.e_base : 0.0f;
    }

    // --- 3. Zona Longe (X: 25..45) - Rastreamento contínuo a partir do
    // Meio/Base ---
    float ref_far =
        res.ok_mid
            ? res.c_mid
            : (res.ok_base ? res.c_base
                           : (s_has_lock ? s_last_base : CENTRO_LATERAL));
    ResultadoVarreduraLinha r_far = varrerZonaVertical(
        buf, 25, 45, (int)(ref_far - 32), (int)(ref_far + 32), th);
    if (!r_far.ok && !res.ok_mid && !res.ok_base) {
      r_far = varrerZonaVertical(buf, 25, 45, SAFE_MIN_Y, SAFE_MAX_Y, th);
    }

    if (r_far.ok) {
      res.c_far = r_far.centroide;
      res.e_far = res.c_far - CENTRO_LATERAL;
      res.ok_far = true;
    } else {
      res.ok_far = false;
      res.c_far = res.c_mid;
      res.e_far = res.e_mid;
    }

    // Fallback de Base caso só Meio/Longe estejam visíveis
    if (!res.ok_base) {
      if (res.ok_mid) {
        res.c_base = res.c_mid;
        res.e_base = res.e_mid;
      } else if (res.ok_far) {
        res.c_base = res.c_far;
        res.e_base = res.e_far;
      } else {
        res.c_base = CENTRO_LATERAL;
        res.e_base = 0.0f;
        s_has_lock = false;
      }
    }

    res.total_pixels = r_base.pixels + r_mid.pixels + r_far.pixels;

  } else {
    /* =========================================================================
     * MODO NORMAL LANDSCAPE (Câmera na horizontal padrão 160x120):
     *  - Eixo Lateral da Pista: Sensor X (0..159, Centro=80)
     *  - Eixo de Profundidade: Sensor Y (0..119)
     *      * Base (Perto): Y 85..115
     *      * Meio (Intermediário): Y 50..78
     *      * Longe (Antecipação): Y 15..42
     * =========================================================================
     */
    const float CENTRO_LATERAL = 80.0f;
    const int SAFE_MIN_X = 14;
    const int SAFE_MAX_X = 145;

    // --- 1. Zona Base (Y: 85..115) ---
    ResultadoVarreduraLinha r_base;
    if (s_has_lock) {
      r_base = varrerZonaHorizontal(buf, 85, 115, (int)(s_last_base - 38),
                                    (int)(s_last_base + 38), th);
      if (!r_base.ok) {
        r_base = varrerZonaHorizontal(buf, 85, 115, SAFE_MIN_X, SAFE_MAX_X, th);
      }
    } else {
      r_base = varrerZonaHorizontal(buf, 85, 115, SAFE_MIN_X, SAFE_MAX_X, th);
    }

    if (r_base.ok) {
      res.c_base = r_base.centroide;
      res.e_base = res.c_base - CENTRO_LATERAL;
      res.ok_base = true;
      s_last_base = 0.75f * res.c_base + 0.25f * s_last_base;
      s_has_lock = true;
    } else {
      res.ok_base = false;
    }

    // --- 2. Zona Meio (Y: 50..78) - Rastreamento contínuo a partir da Base ---
    float ref_mid =
        res.ok_base ? res.c_base : (s_has_lock ? s_last_base : CENTRO_LATERAL);
    ResultadoVarreduraLinha r_mid = varrerZonaHorizontal(
        buf, 50, 78, (int)(ref_mid - 34), (int)(ref_mid + 34), th);
    if (!r_mid.ok && !res.ok_base) {
      r_mid = varrerZonaHorizontal(buf, 50, 78, SAFE_MIN_X, SAFE_MAX_X, th);
    }

    if (r_mid.ok) {
      res.c_mid = r_mid.centroide;
      res.e_mid = res.c_mid - CENTRO_LATERAL;
      res.ok_mid = true;
    } else {
      res.ok_mid = false;
      res.c_mid = res.ok_base ? res.c_base : CENTRO_LATERAL;
      res.e_mid = res.ok_base ? res.e_base : 0.0f;
    }

    // --- 3. Zona Longe (Y: 15..42) - Rastreamento contínuo a partir do
    // Meio/Base ---
    float ref_far =
        res.ok_mid
            ? res.c_mid
            : (res.ok_base ? res.c_base
                           : (s_has_lock ? s_last_base : CENTRO_LATERAL));
    ResultadoVarreduraLinha r_far = varrerZonaHorizontal(
        buf, 15, 42, (int)(ref_far - 34), (int)(ref_far + 34), th);
    if (!r_far.ok && !res.ok_mid && !res.ok_base) {
      r_far = varrerZonaHorizontal(buf, 15, 42, SAFE_MIN_X, SAFE_MAX_X, th);
    }

    if (r_far.ok) {
      res.c_far = r_far.centroide;
      res.e_far = res.c_far - CENTRO_LATERAL;
      res.ok_far = true;
    } else {
      res.ok_far = false;
      res.c_far = res.c_mid;
      res.e_far = res.e_mid;
    }

    // Fallback de Base caso só Meio/Longe estejam visíveis
    if (!res.ok_base) {
      if (res.ok_mid) {
        res.c_base = res.c_mid;
        res.e_base = res.e_mid;
      } else if (res.ok_far) {
        res.c_base = res.c_far;
        res.e_base = res.e_far;
      } else {
        res.c_base = CENTRO_LATERAL;
        res.e_base = 0.0f;
        s_has_lock = false;
      }
    }

    res.total_pixels = r_base.pixels + r_mid.pixels + r_far.pixels;
  }

  // Validação Geral de Linha
  res.linha_valida = (res.ok_base || res.ok_mid || res.ok_far);

  // 2. Cálculo de Curvatura Preditiva (Delta_E = E_far - E_base)
  if (res.ok_far && res.ok_base) {
    res.curvatura = res.e_far - res.e_base;
  } else if (res.ok_mid && res.ok_base) {
    res.curvatura = (res.e_mid - res.e_base) * 1.5f;
  } else {
    res.curvatura = 0.0f;
  }

  // 3. Classificação Automática do Tipo de Pista com Deadband Estável
  if (!res.linha_valida) {
    res.tipo_pista = PISTA_SEM_LINHA;
  } else {
    float abs_curv = fabsf(res.curvatura);
    if (abs_curv < 8.0f) {
      res.tipo_pista = PISTA_RETA;
    } else if (res.curvatura > 0) {
      res.tipo_pista =
          (abs_curv >= 20.0f) ? PISTA_CURVA_FECH_DIR : PISTA_CURVA_SUAVE_DIR;
    } else {
      res.tipo_pista =
          (abs_curv >= 20.0f) ? PISTA_CURVA_FECH_ESQ : PISTA_CURVA_SUAVE_ESQ;
    }
  }

  // 4. Cálculo do Erro Composto Preditivo (Blend Balanceado)
  if (res.ok_base && res.ok_far) {
    float w = (fabsf(res.curvatura) > 10.0f)
                  ? (peso_lookahead > 0.40f ? peso_lookahead : 0.40f)
                  : peso_lookahead;
    res.e_composto = (1.0f - w) * res.e_base + w * res.e_far;
  } else if (res.ok_base && res.ok_mid) {
    res.e_composto = 0.70f * res.e_base + 0.30f * res.e_mid;
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
void desenharOverlayVisualMultiZona(uint8_t *buf,
                                    const ResultadoVisaoMultiZona &vis,
                                    uint8_t orient) {
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
    int y_mid = (int)vis.c_mid;
    int y_far = (int)vis.c_far;

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
        buf[(y - 1) * CAM_WIDTH + x] = 255;
        buf[(y + 1) * CAM_WIDTH + x] = 255;
        buf[y * CAM_WIDTH + (x - 1)] = 255;
        buf[y * CAM_WIDTH + (x + 1)] = 255;
      }
    };

    if (vis.ok_base)
      desenharCruz(135, y_base);
    if (vis.ok_mid)
      desenharCruz(85, y_mid);
    if (vis.ok_far)
      desenharCruz(35, y_far);

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
    int x_mid = (int)vis.c_mid;
    int x_far = (int)vis.c_far;

    if (vis.ok_base && vis.ok_mid) {
      desenharLinhaGrayscale(buf, x_base, 100, x_mid, 65, 255);
    }
    if (vis.ok_mid && vis.ok_far) {
      desenharLinhaGrayscale(buf, x_mid, 65, x_far, 30, 255);
    }

    auto desenharCruz = [&](int x, int y) {
      if (y >= 1 && y < CAM_HEIGHT - 1 && x >= 1 && x < CAM_WIDTH - 1) {
        buf[y * CAM_WIDTH + x] = 255;
        buf[(y - 1) * CAM_WIDTH + x] = 255;
        buf[(y + 1) * CAM_WIDTH + x] = 255;
        buf[y * CAM_WIDTH + (x - 1)] = 255;
        buf[y * CAM_WIDTH + (x + 1)] = 255;
      }
    };

    if (vis.ok_base)
      desenharCruz(x_base, 100);
    if (vis.ok_mid)
      desenharCruz(x_mid, 65);
    if (vis.ok_far)
      desenharCruz(x_far, 30);
  }
}

void taskControleVisao(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(CICLO_CONTROLE_MS);

  float erro_anterior = 0.0f;
  unsigned long ultimo_tempo_linha_ms = millis();
  unsigned long tempo_anterior_fps = millis();
  int contador_frames_fps = 0;

  Serial.println("[CORE 1] Visão Computacional Multi-Zona e Controle Preditivo "
                 "iniciados.");

  for (;;) {
    camera_fb_t *fb = esp_camera_fb_get();

    // Leitura Atômica das configurações do usuário
    portENTER_CRITICAL(&g_configMux);
    float cur_kp = g_config.kp;
    float cur_kd = g_config.kd;
    uint8_t cur_th = g_config.threshold;
    int cur_speed = g_config.velocidadeBase;
    int cur_min_pwm = g_config.pwmMinimo;
    float cur_w_far = g_config.pesoAntecipacao;
    uint8_t cur_orient = g_config.modoOrientacao;
    bool inv_servo = g_config.inverterServo;
    bool tracao_on = g_config.tracaoHabilitada;
    portEXIT_CRITICAL(&g_configMux);

    if (fb != NULL && fb->format == PIXFORMAT_GRAYSCALE) {
      // 1. Processamento Multi-Zona em Tela Cheia
      ResultadoVisaoMultiZona vis =
          processarVisaoMultiZona(fb->buf, cur_th, cur_orient, cur_w_far);

      float angulo_servo = SERVO_ANGULO_CENTRO;
      int pwm_esq = 0;
      int pwm_dir = 0;

      if (vis.linha_valida) {
        ultimo_tempo_linha_ms = millis();
        float erro = vis.e_composto;
        float d_erro = erro - erro_anterior;

        // 2. Controlador PD com Ganho Não-Linear Progressivo para Curvas
        float kp_efetivo = cur_kp;
        float abs_erro = fabsf(erro);
        float abs_curv = fabsf(vis.curvatura);

        if (abs_erro > 12.0f || abs_curv > 15.0f) {
          float max_val = (abs_erro > abs_curv) ? abs_erro : abs_curv;
          float boost = constrain((max_val - 12.0f) / 24.0f, 0.0f, 1.0f);
          kp_efetivo +=
              boost *
              0.40f; // Aumenta a autoridade de esterçamento em curvas fechadas
        }

        float delta_angulo = kp_efetivo * erro + cur_kd * d_erro;

        if (inv_servo) {
          angulo_servo = SERVO_ANGULO_CENTRO - delta_angulo;
        } else {
          angulo_servo = SERVO_ANGULO_CENTRO + delta_angulo;
        }
        angulo_servo =
            constrain(angulo_servo, SERVO_ANGULO_MIN, SERVO_ANGULO_MAX);
        erro_anterior = erro;

        // 3. Controle Suave de Velocidade com Frenagem Inteligente em Curvas
        float desvio_total = abs_erro * 0.55f + abs_curv * 0.45f;
        int pwm_base_curva = cur_speed;
        if (desvio_total > 12.0f) {
          // Limita a redução máxima a 25% para evitar parada dos motores por
          // atrito
          float fator_reducao =
              constrain((desvio_total - 12.0f) / 35.0f, 0.0f, 0.25f);
          pwm_base_curva = cur_speed - (int)(fator_reducao * cur_speed);
        }
        // Garante torque mínimo de segurança em curvas (evita stall)
        if (pwm_base_curva < cur_min_pwm)
          pwm_base_curva = cur_min_pwm;
        if (pwm_base_curva < 120)
          pwm_base_curva = 120;

        // 4. Diferencial Eletrônico na Tração Traseira
        float desvio_direcao =
            (angulo_servo - SERVO_ANGULO_CENTRO) / 40.0f; // -1.0 a +1.0
        desvio_direcao = constrain(desvio_direcao, -1.0f, 1.0f);

        pwm_esq = pwm_base_curva;
        pwm_dir = pwm_base_curva;

        if (desvio_direcao < -0.08f) {
          // Virando à Esquerda: reduz roda interna (esq) e impulsiona roda
          // externa (dir)
          float diff = fabsf(desvio_direcao) * 0.25f;
          pwm_esq = (int)(pwm_base_curva * (1.0f - diff));
          pwm_dir = (int)(pwm_base_curva * (1.0f + diff * 0.4f));
        } else if (desvio_direcao > 0.08f) {
          // Virando à Direita: reduz roda interna (dir) e impulsiona roda
          // externa (esq)
          float diff = fabsf(desvio_direcao) * 0.25f;
          pwm_dir = (int)(pwm_base_curva * (1.0f - diff));
          pwm_esq = (int)(pwm_base_curva * (1.0f + diff * 0.4f));
        }

        pwm_esq = constrain(pwm_esq, 0, 255);
        pwm_dir = constrain(pwm_dir, 0, 255);

        // Atuação nos Motores e Servo
        setAnguloServo(angulo_servo);
        if (tracao_on) {
          setMotoresAvanco(pwm_esq, pwm_dir);
        } else {
          pararMotores();
        }

      } else {
        // 5. Failsafe Inteligente: Perda Momentânea de Linha em Curva
        if (millis() - ultimo_tempo_linha_ms > 1200) {
          // Perda prolongada (> 1.2s): para com segurança
          angulo_servo = SERVO_ANGULO_CENTRO;
          pwm_esq = 0;
          pwm_dir = 0;
          setAnguloServo(SERVO_ANGULO_CENTRO);
          pararMotores();
          erro_anterior = 0.0f;
        } else {
          // Perda momentânea durante curva: mantém esterçamento no último lado
          // detectado
          float delta_failsafe = (erro_anterior >= 0.0f) ? 35.0f : -35.0f;
          if (inv_servo)
            delta_failsafe = -delta_failsafe;
          angulo_servo = constrain(SERVO_ANGULO_CENTRO + delta_failsafe,
                                   SERVO_ANGULO_MIN, SERVO_ANGULO_MAX);
          pwm_esq = 135;
          pwm_dir = 135;
          setAnguloServo(angulo_servo);
          if (tracao_on) {
            setMotoresAvanco(pwm_esq, pwm_dir);
          } else {
            pararMotores();
          }
        }
      }

      // 5. FPS Real
      contador_frames_fps++;
      if (millis() - tempo_anterior_fps >= 1000) {
        g_telemetria.fpsReal = (float)contador_frames_fps * 1000.0f /
                               (float)(millis() - tempo_anterior_fps);
        contador_frames_fps = 0;
        tempo_anterior_fps = millis();
      }

      // 6. Atualização de Telemetria Compartilhada (Thread-Safe)
      portENTER_CRITICAL(&g_configMux);
      g_telemetria.erroComposto = vis.e_composto;
      g_telemetria.erroBase = vis.e_base;
      g_telemetria.erroMid = vis.e_mid;
      g_telemetria.erroFar = vis.e_far;
      g_telemetria.curvatura = vis.curvatura;
      g_telemetria.statusPista = vis.tipo_pista;
      g_telemetria.anguloServo = angulo_servo;
      g_telemetria.pwmAtual = tracao_on ? ((pwm_esq + pwm_dir) / 2) : 0;
      g_telemetria.linhaDetectada = vis.linha_valida;
      g_telemetria.zonaBaseOk = vis.ok_base;
      g_telemetria.zonaMidOk = vis.ok_mid;
      g_telemetria.zonaFarOk = vis.ok_far;
      g_telemetria.tracaoAtiva = tracao_on;
      g_telemetria.pixelsLinha = vis.total_pixels;
      g_telemetria.thresholdEfetivo = vis.threshold_usado;
      g_telemetria.timestamp = millis();
      portEXIT_CRITICAL(&g_configMux);

      // 7. Renderização do Overlay e Compartilhamento de Frame para Streaming
      desenharOverlayVisualMultiZona(fb->buf, vis, cur_orient);

      uint8_t *jpg_out = NULL;
      size_t jpg_len_out = 0;
      if (fmt2jpg(fb->buf, fb->len, CAM_WIDTH, CAM_HEIGHT, PIXFORMAT_GRAYSCALE,
                  75, &jpg_out, &jpg_len_out)) {
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
 * 7. HANDLERS DAS ROTAS HTTP (CORE 0 - DUAL-PORT HTTP SERVER)
 * =====================================================================================
 */

static const char *_STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=esp32cam_boundary";
static const char *_STREAM_BOUNDARY = "\r\n--esp32cam_boundary\r\n";
static const char *_STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

/**
 * @brief Handler dedicado da rota de Streaming MJPEG (/stream na porta 81).
 */
static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t res = ESP_OK;
  char part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK)
    return res;

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "30");

  Serial.println("[STREAM] Cliente conectado ao streaming (Porta 81).");

  while (true) {
    uint8_t *jpg_temp = NULL;
    size_t jpg_len = 0;

    if (xSemaphoreTake(g_shared_frame.mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (g_shared_frame.jpg_buf != NULL && g_shared_frame.jpg_len > 0) {
        jpg_len = g_shared_frame.jpg_len;
        jpg_temp = (uint8_t *)malloc(jpg_len);
        if (jpg_temp != NULL) {
          memcpy(jpg_temp, g_shared_frame.jpg_buf, jpg_len);
        }
      }
      xSemaphoreGive(g_shared_frame.mutex);
    }

    if (jpg_temp != NULL && jpg_len > 0) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY,
                                  strlen(_STREAM_BOUNDARY));
      if (res == ESP_OK) {
        size_t hlen =
            snprintf(part_buf, sizeof(part_buf), _STREAM_PART, jpg_len);
        res = httpd_resp_send_chunk(req, part_buf, hlen);
      }
      if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, (const char *)jpg_temp, jpg_len);
      }
      free(jpg_temp);

      if (res != ESP_OK)
        break;
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
  TelemetriaRobo t;

  portENTER_CRITICAL(&g_configMux);
  t = g_telemetria;
  portEXIT_CRITICAL(&g_configMux);

  const char *status_str =
      t.tracaoAtiva ? (t.linhaDetectada ? "RODANDO" : "LINHA PERDIDA")
                    : "PARADO";

  snprintf(json, sizeof(json),
           "{\"erro\":%.2f,\"erro_base\":%.2f,\"erro_mid\":%.2f,\"erro_far\":%."
           "2f,\"curvatura\":%.2f,"
           "\"status_pista\":%u,\"angulo\":%.1f,\"pwm\":%d,\"fps\":%.1f,"
           "\"detectada\":%s,"
           "\"base_ok\":%s,\"mid_ok\":%s,\"far_ok\":%s,\"status\":\"%s\","
           "\"th\":%u,\"pixels\":%u}",
           t.erroComposto, t.erroBase, t.erroMid, t.erroFar, t.curvatura,
           t.statusPista, t.anguloServo, t.pwmAtual, t.fpsReal,
           t.linhaDetectada ? "true" : "false", t.zonaBaseOk ? "true" : "false",
           t.zonaMidOk ? "true" : "false", t.zonaFarOk ? "true" : "false",
           status_str, t.thresholdEfetivo, t.pixelsLinha);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, strlen(json));
}

/**
 * @brief Handler para ajuste dinâmico (/tune na porta 80).
 */
static esp_err_t tune_handler(httpd_req_t *req) {
  char *buf = NULL;
  size_t buf_len = httpd_req_get_url_query_len(req) + 1;

  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);
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
      if (httpd_query_key_value(buf, "orient", param, sizeof(param)) ==
          ESP_OK) {
        g_config.modoOrientacao = (uint8_t)atoi(param);
      }
      if (httpd_query_key_value(buf, "inv_servo", param, sizeof(param)) ==
          ESP_OK) {
        g_config.inverterServo = (atoi(param) == 1);
      }
      portEXIT_CRITICAL(&g_configMux);

      Serial.printf("[TUNE] Kp=%.2f, Kd=%.2f, Th=%d, Speed=%d, WFar=%.2f, "
                    "Orient=%d, InvServo=%d\n",
                    g_config.kp, g_config.kd, g_config.threshold,
                    g_config.velocidadeBase, g_config.pesoAntecipacao,
                    g_config.modoOrientacao, g_config.inverterServo);
    }
    if (buf)
      free(buf);
  }

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "OK", 2);
}

/**
 * @brief Handler para comandos (/cmd na porta 80).
 */
static esp_err_t cmd_handler(httpd_req_t *req) {
  char *buf = NULL;
  size_t buf_len = httpd_req_get_url_query_len(req) + 1;

  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);
    if (buf && httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      char action[16];
      if (httpd_query_key_value(buf, "action", action, sizeof(action)) ==
          ESP_OK) {
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
    if (buf)
      free(buf);
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

  httpd_uri_t index_uri = {.uri = "/",
                           .method = HTTP_GET,
                           .handler = index_handler,
                           .user_ctx = NULL};
  httpd_uri_t data_uri = {.uri = "/data",
                          .method = HTTP_GET,
                          .handler = data_handler,
                          .user_ctx = NULL};
  httpd_uri_t tune_uri = {.uri = "/tune",
                          .method = HTTP_GET,
                          .handler = tune_handler,
                          .user_ctx = NULL};
  httpd_uri_t cmd_uri = {.uri = "/cmd",
                         .method = HTTP_GET,
                         .handler = cmd_handler,
                         .user_ctx = NULL};

  if (httpd_start(&g_httpd_web, &config_web) == ESP_OK) {
    httpd_register_uri_handler(g_httpd_web, &index_uri);
    httpd_register_uri_handler(g_httpd_web, &data_uri);
    httpd_register_uri_handler(g_httpd_web, &tune_uri);
    httpd_register_uri_handler(g_httpd_web, &cmd_uri);
    Serial.println(
        "[HTTP] Servidor Web ativo na Porta 80 (/, /data, /tune, /cmd).");
  }

  // --- 2. Servidor de Streaming MJPEG Dedicado (Porta 81) ---
  httpd_config_t config_stream = HTTPD_DEFAULT_CONFIG();
  config_stream.server_port = 81;
  config_stream.ctrl_port = 32769;
  config_stream.max_open_sockets = 2;
  config_stream.stack_size = 4096;

  httpd_uri_t stream_uri = {.uri = "/stream",
                            .method = HTTP_GET,
                            .handler = stream_handler,
                            .user_ctx = NULL};

  if (httpd_start(&g_httpd_stream, &config_stream) == ESP_OK) {
    httpd_register_uri_handler(g_httpd_stream, &stream_uri);
    Serial.println("[HTTP] Servidor de Streaming ativo na Porta 81 (/stream).");
  }
}

/* =====================================================================================
 * 8. CONEXÃO WI-FI DINÂMICA (DHCP / CORE 0)
 * =====================================================================================
 */

void conectarWiFiDHCP() {
  Serial.printf("\n[WIFI] Conectando ao Wi-Fi via DHCP: %s...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int tentativas = 0;
  while (WiFi.status() != WL_CONNECTED && tentativas < 30) {
    delay(400);
    Serial.print(".");
    tentativas++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(PIN_LED_STATUS,
                 HIGH); // LED vermelho onboard 100% apagado (Active LOW)
    if (MDNS.begin("carrinho")) {
      MDNS.addService("http", "tcp", 80);
      Serial.println(
          "[MDNS] Respondedor mDNS iniciado: http://carrinho.local/");
    }

    Serial.println(
        "\n========================================================");
    Serial.println("[WIFI] CONECTADO COM SUCESSO (DHCP DINÂMICO)!");
    Serial.printf("[WIFI] Endereço IP Obtido: http://%s/\n",
                  WiFi.localIP().toString().c_str());
    Serial.printf("[WIFI] Link alternativo:   http://carrinho.local/\n");
    Serial.printf("[WIFI] Stream da Câmera:   http://%s:81/stream\n",
                  WiFi.localIP().toString().c_str());
    Serial.println("========================================================");

    // Pisca o Flash 4 vezes ao conectar com sucesso no Wi-Fi / Servidor
    piscarFlash(4, 100);
  } else {
    digitalWrite(PIN_LED_STATUS, HIGH); // Apagado
    Serial.println(
        "\n[WIFI] Wi-Fi não conectado. Executando em modo autônomo.");
  }
}

/* =====================================================================================
 * 9. SETUP E LOOP PRINCIPAL
 * =====================================================================================
 */

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
  xTaskCreatePinnedToCore(taskControleVisao, "taskControleVisao", 8192, NULL, 2,
                          &g_taskVisaoHandle, 1);

  // 5. Conecta ao Hotspot Wi-Fi e inicia Servidores na Porta 80 e 81
  conectarWiFiDHCP();
  iniciarServidoresWeb();

  Serial.println("[SISTEMA] Inicialização concluída com sucesso.\n");
}

void loop() {
  digitalWrite(PIN_LED_STATUS, HIGH); // Mantém o LED onboard sempre apagado

  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long ultimo_reconnect = 0;
    if (millis() - ultimo_reconnect > 5000) {
      ultimo_reconnect = millis();
      Serial.println("[WIFI] Reconectando ao Hotspot...");
      WiFi.reconnect();
    }
  }

  vTaskDelay(pdMS_TO_TICKS(1000));
}
