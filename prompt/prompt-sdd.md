# 📐 MASTER PROMPT SDD: FIRMWARE INTEGRAL ESP32-CAM (SEGUIDOR DE LINHA AUTÔNOMO)

Este documento contém o **Prompt de Engenharia Orientado a Especificações (Spec-Driven Development - SDD)** projetado para instruir modelos de IA avançados a gerarem, com 100% de exatidão e sem omissões, o código-fonte integral de firmware do carrinho seguidor de linha autônomo baseado no **ESP32-CAM (AI-Thinker OV2640)**.

---

## 🎯 Como Utilizar este Prompt

Copie o conteúdo da seção **[PROMPT SDD MASTER]** abaixo e envie ao modelo de linguagem. O prompt foi arquitetado segundo as melhores práticas de SDD:
1. **Definição Estrita de Interfaces e Tipos (Domain Contracts)**.
2. **Especificação Algorítmica e Matemática Formal (Invariantes e Equações)**.
3. **Restrições de Tempo Real e Concorrência (Dual-Core FreeRTOS & Thread-Safety)**.
4. **Proteções Elétricas e Hardware Level Constraints**.
5. **Critérios de Aceite e Verificação de Não-Omissão**.

---

```markdown
# SYSTEM PROMPT: ENGENHARIA DE FIRMWARE CRÍTICO & SISTEMAS EMBARCADOS

Você atuará como Engenheiro de Firmware Sênior e Especialista em Visão Computacional Embarcada e RTOS. Sua missão é projetar e implementar o código-fonte C++ completo, robusto e sem omissões (`espCarrinho.ino`) para um Carrinho Seguidor de Linha Autônomo com processamento 100% onboard no microcontrolador ESP32-CAM (módulo AI-Thinker com sensor OV2640 e PSRAM).

O desenvolvimento deve seguir rigorosamente a metodologia **Spec-Driven Development (SDD)** detalhada nas especificações formais a seguir.

---

## 1. ESPECIFICAÇÃO DE HARDWARE E RESTRIÇÕES ELÉTRICAS

### 1.1 Pinout do Módulo ESP32-CAM (AI-Thinker OV2640)
* **Barramento de Dados e Controle da Câmera**:
  * PWDN: `GPIO 32`, RESET: `-1` (desconectado), XCLK: `GPIO 0`, SIOD (SDA): `GPIO 26`, SIOC (SCL): `GPIO 27`.
  * D7: `GPIO 35` (Y9), D6: `GPIO 34` (Y8), D5: `GPIO 39` (Y7), D4: `GPIO 36` (Y6).
  * D3: `GPIO 21` (Y5), D2: `GPIO 19` (Y4), D1: `GPIO 18` (Y3), D0: `GPIO 5` (Y2).
  * VSYNC: `GPIO 25`, HREF: `GPIO 23`, PCLK: `GPIO 22`.
* **Atuadores e Sinais de Controle**:
  * Servo Motor de Direção (Eixo Dianteiro): `GPIO 02`.
  * Ponte H DC (Controle da Tração Traseira Dupla):
    * Motor Esquerdo: `IN1` = `GPIO 14` (PWM/Avanço), `IN2` = `GPIO 15` (Sentido/GND).
    * Motor Direito: `IN3` = `GPIO 13` (PWM/Avanço), `IN4` = `GPIO 12` (Sentido/GND).
  * LED de Status Onboard: `GPIO 33` (Lógica invertida / Active LOW; manter HIGH/apagado para não perturbar a câmera).
  * Flash LED de Alta Potência: `GPIO 04` (Active HIGH; rotina de feedback visual por pulsos).

### 1.2 Restrições Elétricas e Sequenciamento de Boot (Safety Critical)
* **Proteção contra Bootloader Strapping (GPIO 12 / MTDI)**:
  * O `GPIO 12` é um pino de strapping do ESP32 que define a tensão da flash SPI no boot. Se estiver em nível alto no reset, o ESP32 entra em falha (bootloop/brownout).
  * A função `inicializarHardwareSeguro()` deve forçar `GPIO 12`, `13`, `14`, `15` e `2` para `OUTPUT` e `LOW` imediatamente no primeiro ciclo de execução.
* **Supressão de Brownout**:
  * Desativar o detector de brownout no início do `setup()` via registrador RTC:
    `WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);`
* **Alocação de Canais PWM LEDC (Prevenção de Conflito de Timers)**:
  * Câmera OV2640 usa internamente o `LEDC_CHANNEL_0` / `LEDC_TIMER_0` para o sinal XCLK de 20 MHz.
  * O código deve obrigatoriamente isolar os periféricos em canais e timers distintos:
    * **Servo Motor**: Canal 2, Timer 1, Frequência = 50 Hz, Resolução = 14 bits (`0..16383`).
    * **Motor Esquerdo (IN1)**: Canal 4, Timer 2, Frequência = 5 kHz, Resolução = 8 bits (`0..255`).
    * **Motor Direito (IN3)**: Canal 5, Timer 2, Frequência = 5 kHz, Resolução = 8 bits (`0..255`).
  * O código deve suportar compatibilidade dupla entre o ESP32 Arduino Core v2.x (`ledcSetup`, `ledcAttachPin`, `ledcWrite`) e o Core v3.x (`ledcAttach`, `ledcWrite`) utilizando condicionais de pré-processador `#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)`.

---

## 2. ESPECIFICAÇÃO ARQUITETURAL & CONCORRÊNCIA FREERTOS

O sistema deve adotar uma arquitetura Dual-Core assimétrica e estritamente desacoplada:

```
┌────────────────────────────────────────────────────────┐
│                        ESP32                           │
│                                                        │
│  ┌─────────────────────────┐ ┌──────────────────────┐  │
│  │         CORE 1          │ │        CORE 0        │  │
│  │ (Tempo Real Crítico)    │ │ (Rede & Streaming)   │  │
│  ├─────────────────────────┤ ├──────────────────────┤  │
│  │ • Captura QQVGA         │ │ • Wi-Fi STA (DHCP)   │  │
│  │ • Visão Multi-Zona      │ │ • mDNS (carrinho)    │  │
│  │ • Dynamic Threshold     │ │ • HTTP Port 80 (API) │  │
│  │ • Controle PD Não-linear│ │ • HTTP Port 81       │  │
│  │ • Diferencial Eletrônico│ │   (MJPEG Stream)     │  │
│  │ • Overlay Grayscale     │ │ • Fallback & Recon.  │  │
│  └───────────┬─────────────┘ └──────────▲───────────┘  │
│              │                          │              │
│              ▼                          │              │
│   [SharedFrame Mutex] ──────────────────┘              │
│   [Spinlock Seção Crítica (g_configMux)]               │
└────────────────────────────────────────────────────────┘
```

### 2.1 Core 1: Tarefa de Controle e Visão (`taskControleVisao`)
* **Prioridade**: 2 (prioridade alta).
* **Stack**: 8192 bytes.
* **Peridiocidade**: Loop determinístico com ciclo fixo de 30 ms (~33 FPS), implementado via `vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(30))`.
* **Isolamento**: Nunca deve ser bloqueada por operações de I/O de rede, reconexão Wi-Fi ou conexões lentas de clientes HTTP.

### 2.2 Sincronização Thread-Safe
* **Parâmetros de Calibração (`ConfigControle`) e Telemetria (`TelemetriaRobo`)**:
  * Devem ser protegidos por Spinlock atômico de núcleo (`portMUX_TYPE g_configMux = portMUX_INITIALIZER_UNLOCKED;`) com `portENTER_CRITICAL(&g_configMux)` e `portEXIT_CRITICAL(&g_configMux)`.
* **Buffer de Frame de Vídeo Compartilhado (`SharedFrame`)**:
  * Estrutura contendo ponteiro `jpg_buf`, tamanho `jpg_len` e semáforo mutex `SemaphoreHandle_t mutex`.
  * Core 1 comprime o frame processado com overlay para JPEG (`fmt2jpg`) e tenta obter o mutex com timeout não bloqueante de 5 ms. Se obtiver, substitui o buffer de streaming e libera o anterior.
  * Core 0 obtém o mutex com timeout de 50 ms, cria cópia local (`malloc` + `memcpy`) para envio HTTP chunked e libera o mutex imediatamente para não prender o Core 1.

### 2.3 Arquitetura de Servidor HTTP Dual-Port (Anti-Bloqueio de Streaming)
* **Motivação Técnica**: No driver `esp_http_server.h`, um handler de streaming MJPEG executa um loop infinito de `httpd_resp_send_chunk`. Em porta única, isso monopoliza os sockets/threads do servidor e bloqueia requisições simultâneas de telemetria (`/data`), comandos (`/cmd`) e calibração (`/tune`).
* **Especificação Dual-Port**:
  * **Servidor 1 (Porta 80)**: Destinado exclusivamente à API REST e interface Web (rotas `/`, `/data`, `/tune`, `/cmd`). Configuração: 4 sockets abertos, stack de 8192 bytes.
  * **Servidor 2 (Porta 81)**: Destinado exclusivamente ao streaming contínuo MJPEG (rota `/stream`). Configuração: 2 sockets abertos, stack de 4096 bytes.

---

## 3. ESPECIFICAÇÃO DE DOMÍNIO (ESTRUTURAS DE DADOS E ENUMS)

O código deve declarar formalmente os seguintes tipos:

```cpp
enum ModoOrientacao {
  ORIENT_NORMAL_LANDSCAPE = 0, // Sensor na horizontal (160x120)
  ORIENT_ROTACIONADO_90   = 1  // Sensor na vertical (120x160)
};

enum TipoPista {
  PISTA_SEM_LINHA       = 0,
  PISTA_RETA            = 1,
  PISTA_CURVA_SUAVE_ESQ = 2,
  PISTA_CURVA_SUAVE_DIR = 3,
  PISTA_CURVA_FECH_ESQ  = 4,
  PISTA_CURVA_FECH_DIR  = 5
};

struct ResultadoVarreduraLinha {
  float centroide;
  uint32_t pixels;
  bool ok;
};

struct ResultadoVisaoMultiZona {
  float c_base, c_mid, c_far; // Posição dos centróides
  float e_base, e_mid, e_far; // Erro lateral relativo ao centro
  float e_composto;           // Erro ponderado final
  float curvatura;            // Delta_E = e_far - e_base
  bool ok_base, ok_mid, ok_far;
  bool linha_valida;
  uint8_t tipo_pista;
  uint32_t total_pixels;
  uint8_t threshold_usado;
};

struct ConfigControle {
  float kp;
  float kd;
  uint8_t threshold;       // 0 = Auto-dinâmico, >0 = Fixo manual
  int velocidadeBase;      // 0..255
  int pwmMinimo;           // Limite mínimo de torque
  float pesoAntecipacao;   // Lookahead weight (0.0..0.6)
  uint8_t modoOrientacao;  // ModoOrientacao enum
  bool inverterServo;      // Inversão do sentido de esterçamento
  bool tracaoHabilitada;   // Master enable de movimento
};

struct TelemetriaRobo {
  float erroComposto;
  float erroBase;
  float erroMid;
  float erroFar;
  float curvatura;
  uint8_t statusPista;
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
```

---

## 4. ESPECIFICAÇÃO MATEMÁTICA E PIPELINE DE VISÃO COMPUTACIONAL

### 4.1 Configuração da Câmera
* Formato: `PIXFORMAT_GRAYSCALE`, Resolução: `FRAMESIZE_QQVGA` (160x120), `fb_count = 2`, alocação prioritária em PSRAM, `grab_mode = CAMERA_GRAB_LATEST`.
* Inversão e contraste via sensor: `vflip = 0`, `hmirror = 1`, `brightness = 1`, `contrast = 2`.

### 4.2 Thresholding Dinâmico Adaptativo por Contraste Min-Max
Quando `threshold == 0`:
1. Amostrar pixels em passos de 4 na região central (descartando bordas para eliminar vinhetagem da lente).
2. Determinar $I_{min}$ e $I_{max}$.
3. Se $(I_{max} - I_{min}) > 28$:
   $$Th_{dinamico} = I_{min} + (I_{max} - I_{min}) \times 0.35$$
4. Caso o contraste seja insuficiente, adotar fallback seguro: $Th = 60$.

### 4.3 Processamento Multi-ROI em 3 Zonas de Profundidade
O espaço de captura é dividido em 3 zonas espaciais para capturar o erro imediato, a estabilização e a curvatura preditiva antecipada:

| Modo de Orientação | Eixo Lateral (Erro) | Eixo Profundidade | Zona Base (Perto) | Zona Meio (Estabilização) | Zona Longe (Lookahead) | Margem de Segurança |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **0: Normal Landscape** | X (Centro = 80) | Y (0..119) | $Y \in [85, 115]$ | $Y \in [50, 78]$ | $Y \in [15, 42]$ | $X \in [14, 145]$ |
| **1: Rotacionado 90°** | Y (Centro = 60) | X (0..159) | $X \in [125, 145]$ | $X \in [75, 95]$ | $X \in [25, 45]$ | $Y \in [12, 107]$ |

### 4.4 Rastreamento Espacial Contínuo (Spatial Windowing ROI)
Para imunidade contra reflexos de luz e ruídos externos fora da pista:
* A busca na Zona Base ocorre inicialmente em uma janela restrita em torno do último centróide conhecido: $[C_{base,prev} - W, C_{base,prev} + W]$, onde $W = 38$ (Landscape) ou $W = 36$ (Rotacionado).
* Se a linha for perdida na janela local, abre-se a busca para as margens completas de segurança.
* A Zona Meio rastreia referenciada no centróide da Zona Base ($W = 34$ ou $32$).
* A Zona Longe rastreia referenciada no centróide da Zona Meio ($W = 34$ ou $32$).
* Filtro IIR de primeira ordem para suavização do lock de base:
  $$C_{base,filtrado} = 0.75 \cdot C_{base} + 0.25 \cdot C_{base,prev}$$
* Centróide calculado pela média ponderada de pixels escuros ($Pixel < Th$):
  $$C = \frac{\sum (Pos \cdot [Pixel < Th])}{\sum [Pixel < Th]}$$
  Válido apenas se total de pixels escuros $\ge 4$ (`MIN_PIXELS_ZONA`).

### 4.5 Cálculo de Curvatura e Classificação de Pista
* **Curvatura Preditiva**:
  $$\Delta E = E_{far} - E_{base}$$
  *(Caso a zona longe esteja temporariamente oculta: $\Delta E = (E_{mid} - E_{base}) \times 1.5$)*.
* **Classificação de Pista (`TipoPista`)**:
  * Se nenhuma zona válida: `PISTA_SEM_LINHA`.
  * Se $|\Delta E| < 8.0$: `PISTA_RETA`.
  * Se $\Delta E > 0$: `PISTA_CURVA_FECH_DIR` (se $\ge 20.0$) ou `PISTA_CURVA_SUAVE_DIR`.
  * Se $\Delta E < 0$: `PISTA_CURVA_FECH_ESQ` (se $|\Delta E| \ge 20.0$) ou `PISTA_CURVA_SUAVE_ESQ`.

### 4.6 Erro Composto Preditivo
O erro final de realimentação combina a resposta rápida da base com a antecipação de curva:
$$E_{composto} = (1 - w) \cdot E_{base} + w \cdot E_{far}$$
Onde $w = \max(pesoAntecipacao, 0.40)$ quando $|\Delta E| > 10.0$ para aumentar a sensibilidade à entrada de curvas. Fallbacks matemáticos devem garantir continuidade se apenas base ou mid estiverem ativas.

### 4.7 Overlay Visual In-Place no Buffer Grayscale
Antes da conversão para JPEG, o Core 1 deve desenhar graficamente no frame buffer (sem alocar buffers adicionais):
* Linhas pontilhadas delimitando as 3 zonas com intensidades cinza distintas (220, 180, 140).
* Linha pontilhada no centro ideal da pista (intensidade 100).
* Vetor contínuo branco (intensidade 255) conectando os centróides detectados (Base -> Meio -> Longe) usando o algoritmo de linha de Bresenham (`desenharLinhaGrayscale`).
* Cruzes de mira ($5 \times 5$ pixels) centralizadas exatamente sobre as coordenadas dos centróides válidos.

---

## 5. ESPECIFICAÇÃO DO SISTEMA DE CONTROLE E ATUAÇÃO

### 5.1 Controle de Direção: PD com Ganho Progressivo Não-Linear
1. Ganho Proporcional Adaptativo em função da severidade da curva:
   $$K_{p,efetivo} = K_p + Boost \times 0.40$$
   Onde $Boost = \text{constrain}\left(\frac{\max(|E_{composto}|, |\Delta E|) - 12.0}{24.0}, 0.0, 1.0\right)$.
2. Delta angular:
   $$\Delta \theta = K_{p,efetivo} \cdot E_{composto} + K_d \cdot (E_{composto} - E_{ant})$$
3. Ângulo do Servo:
   $$\theta = \text{constrain}(90.0 \pm \Delta \theta, 50.0^\circ, 130.0^\circ)$$
   *(Sinal positivo ou negativo condicionado à flag `inverterServo`)*.
4. Conversão para largura de pulso e duty cycle LEDC (Timer 14 bits em 50 Hz):
   $$\text{pulso}_{\mu s} = 500.0 + \left(\frac{\theta}{180.0}\right) \times 2000.0$$
   $$\text{duty} = \left(\frac{\text{pulso}_{\mu s}}{20000.0}\right) \times 16383$$

### 5.2 Frenagem Preditiva em Curvas (Anti-Stall e Redução de Inércia)
* Desvio combinado:
  $$\text{Desvio} = 0.55 \cdot |E_{composto}| + 0.45 \cdot |\Delta E|$$
* Se $\text{Desvio} > 12.0$: Reduz a velocidade base linearmente em até $25\%$ ($\text{Fator} \le 0.25$).
* Garantia de torque:
  $$PWM_{curva} = \max(PWM_{calculado}, pwmMinimo, 120)$$

### 5.3 Diferencial Eletrônico na Tração Traseira
Calcula o índice de esterçamento $D = \text{constrain}\left(\frac{\theta - 90.0}{40.0}, -1.0, 1.0\right)$:
* **Virando à Esquerda ($D < -0.08$)**:
  $$PWM_{esq} = PWM_{curva} \cdot (1.0 - |D| \times 0.25)$$
  $$PWM_{dir} = PWM_{curva} \cdot (1.0 + |D| \times 0.40)$$
* **Virando à Direita ($D > +0.08$)**:
  $$PWM_{dir} = PWM_{curva} \cdot (1.0 - |D| \times 0.25)$$
  $$PWM_{esq} = PWM_{curva} \cdot (1.0 + |D| \times 0.40)$$
* Valores finais saturados em $[0, 255]$.

### 5.4 Failsafe Inteligente de Perda de Linha
Se `linha_valida == false`:
* **Perda Momentânea ($\le 1200$ ms)**: O robô mantém esterçamento fixo de $35^\circ$ no último sentido detectado ($E_{ant} \ge 0 \rightarrow +35^\circ$ ou $-35^\circ$) com tração constante de recuperação ($PWM = 135$).
* **Perda Prolongada ($> 1200$ ms)**: Desliga imediatamente os motores ($PWM = 0$) e centraliza o servo de direção ($90^\circ$).

---

## 6. ESPECIFICAÇÃO DE REDE, API E SERVIDORES HTTP

### 6.1 Conectividade Wi-Fi e mDNS
* Conexão Station (STA) via DHCP com credenciais configuráveis (`WIFI_SSID`, `WIFI_PASS`).
* Inicialização do mDNS com hostname `carrinho` (`http://carrinho.local/`).
* Feedback visual: Piscar o LED Flash de alta potência (`GPIO 04`) exatamente 4 vezes (100 ms) ao conectar com sucesso.
* Auto-reconexão não-bloqueante no `loop()` a cada 5 segundos se a conexão for perdida.

### 6.2 Endpoints da Porta 80 (API e Dashboard)
1. **`GET /`**:
   * Content-Type: `text/html`. Retorna o array de PROGMEM `INDEX_HTML` (declarado externamente em `#include "index_html.h"`).
2. **`GET /data`**:
   * Content-Type: `application/json` (Header: `Access-Control-Allow-Origin: *`).
   * Schema:
     ```json
     {
       "erro": float,
       "erro_base": float,
       "erro_mid": float,
       "erro_far": float,
       "curvatura": float,
       "status_pista": int,
       "angulo": float,
       "pwm": int,
       "fps": float,
       "detectada": "true"|"false",
       "base_ok": "true"|"false",
       "mid_ok": "true"|"false",
       "far_ok": "true"|"false",
       "status": "RODANDO"|"LINHA PERDIDA"|"PARADO",
       "th": int,
       "pixels": int
     }
     ```
3. **`GET /tune?kp=...&kd=...&th=...&speed=...&wfar=...&orient=...&inv_servo=...`**:
   * Atualização atômica dos parâmetros em `g_config` via `portENTER_CRITICAL(&g_configMux)`.
   * Resposta: `text/plain` com corpo `"OK"`.
4. **`GET /cmd?action=start|stop`**:
   * Habilita ou desabilita `tracaoHabilitada`. Em caso de `stop`, chama imediatamente `pararMotores()`.
   * Resposta: `text/plain` com corpo `"OK"`.

### 6.3 Endpoint da Porta 81 (Servidor de Streaming MJPEG Dedicado)
1. **`GET /stream`**:
   * Content-Type: `multipart/x-mixed-replace;boundary=esp32cam_boundary`.
   * Headers: `Access-Control-Allow-Origin: *`, `X-Framerate: 30`.
   * Operação: Loop `while(true)` com envio multipart chunked dos frames JPEG lidos do mutex `g_shared_frame`. Liberação garantida de memória alocada dinamicamente (`free`) a cada iteração.

---

## 7. CRITÉRIOS DE ACEITE E DIRETRIZES DE SAÍDA

1. **Completude Absoluta**: O arquivo `.ino` deve conter todas as funções, protótipos de função, includes e estruturas, pronto para compilar na Arduino IDE para a placa `AI Thinker ESP32-CAM`.
2. **Sem Placeholders**: É estritamente proibido o uso de omissões como `// insira seu código aqui` ou blocos incompletos.
3. **Gerenciamento de Memória**: Toda alocação de buffer (`malloc` no HTTP server e `fmt2jpg`) deve ter seu respectivo `free()` garantido, e todo `esp_camera_fb_get()` deve ser seguido de `esp_camera_fb_return(fb)`.
4. **Dependências Externas**: Código deve depender exclusivamente de headers padrão do framework ESP32 e do arquivo local `"index_html.h"`.
```

---

## 📋 Resumo das Características-Chave Especificadas no SDD

| Subsistema | Especificação Técnica Atendida |
| :--- | :--- |
| **Arquitetura RTOS** | Dual-Core desacoplado (Core 1: Visão/Controle em ciclo de 30 ms; Core 0: Wi-Fi/Web). |
| **Servidores HTTP** | Dual-Port independente (Porta 80 para API/Painel e Porta 81 para Streaming MJPEG). |
| **Visão Computacional** | Multi-ROI 3 Zonas (Base, Meio, Longe) com Spatial Tracking e Dynamic Thresholding. |
| **Controle Preditivo** | Controlador PD com ganho adaptativo não-linear e antecipação por $\Delta E$. |
| **Dinâmica Veicular** | Frenagem inteligente em curvas e diferencial eletrônico ativo na tração traseira. |
| **Robustez Elétrica** | Proteção de strapping do GPIO 12 no boot, supressão de brownout e isolamento de timers LEDC. |
| **Failsafe** | Recuperação de curva por 1200 ms e parada total segura em caso de perda prolongada. |
