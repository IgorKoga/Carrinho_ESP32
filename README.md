# 🏎️ Carrinho Seguidor de Linha Autônomo com ESP32-CAM (Multi-Zona & Curvas Preditivas)

Firmware completo de alta performance para robô seguidor de linha autônomo com processamento 100% embarcado no módulo **ESP32-CAM (AI-Thinker OV2640)**, arquitetura dual-core com **FreeRTOS**, visão computacional **Multi-ROI (3 Níveis de Profundidade: Base, Meio e Longe)**, identificação preditiva de **Curvas e Vetores de Trajetória**, controle **PD**, servidor HTTP nativo (`esp_http_server.h`), streaming **MJPEG** dedicado e **Painel Web Dark Theme 100% Offline**.

---

## 📋 Sumário
- [Arquitetura Multicore (FreeRTOS)](#-arquitetura-multicore-freertos)
- [Pipeline de Visão Computacional Multi-Zona (Tela Inteira)](#-pipeline-de-visão-computacional-multi-zona-tela-inteira)
- [Identificação e Antecipação Preditiva de Curvas](#-identificação-e-antecipação-preditiva-de-curvas)
- [Pinagem do Hardware](#-pinagem-do-hardware)
- [Dashboard Web e API REST](#-dashboard-web-e-api-rest)
- [Como Compilar e Gravar no ESP32-CAM](#-como-compilar-e-gravar-no-esp32-cam)
- [Alimentação e Cuidados Elétricos](#-alimentação-e-cuidados-elétricos)

---

## 🏗️ Arquitetura Multicore (FreeRTOS)

O sistema opera com separação estrita de tarefas entre os dois núcleos do ESP32 para garantir **latência zero** de controle e **alta taxa de streaming**:

```
 ┌─────────────────────────────────────────────────────────────┐
 │                      ESP32 DUAL-CORE                        │
 ├──────────────────────────────┬──────────────────────────────┤
 │           CORE 1             │            CORE 0            │
 │   (Visão & Controle PD)      │    (Rede Wi-Fi & Servidor)   │
 ├──────────────────────────────┼──────────────────────────────┤
 │ • Captura QQVGA (160x120)    │ • Wi-Fi Station Hotspot      │
 │ • Varredura Multi-Zona (3x)  │ • DHCP Dinâmico / mDNS       │
 │ • Detecção Preditiva Curvas  │ • Host: carrinho.local       │
 │ • Cálculo Curvatura (ΔE)     │ • Servidor HTTP (Porta 80)   │
 │ • Frenagem Antecipada        │ • Streaming MJPEG (/stream)  │
 │ • Overlay Trajetória Stream  │ • Telemetria JSON (/data)    │
 │ • Loop Estrito: ~30 ms       │ • Calibração Dinâmica (/tune)│
 │                              │ • Comandos Tração (/cmd)     │
 └──────────────────────────────┴──────────────────────────────┘
```

---

## 👁️ Pipeline de Visão Computacional Multi-Zona (Tela Inteira)

Ao invés de ler apenas uma faixa restrita, o pipeline processa a tela inteira dividida em 3 zonas estratégicas de profundidade:

```
       ┌───────────────────────────────┐
       │                               │  (Longe / Horizonte)
       │    [Zona 3: Lookahead Longe]  │  --> Detecta Curva Antecipada (E_far)
       │           \     /             │
       │    [Zona 2: Médio Alcance]    │  --> Estabilização de Trajetória (E_mid)
       │             | |               │
       │    [Zona 1: Base / Perto]     │  --> Atuação Imediata do Servo (E_base)
       │             | |               │  (Perto do Para-choque)
       └───────────────────────────────┘
```

1. **Zona 1 (Base / Perto):** Calcula o erro lateral imediato ($E_{\text{base}}$) próximo ao para-choque.
2. **Zona 2 (Meio / Intermediário):** Calcula a posição central da pista ($E_{\text{mid}}$) para amortecimento de oscilação.
3. **Zona 3 (Longe / Lookahead):** Rastreia a linha à distância ($E_{\text{far}}$) para antecipar curvas antes de o carrinho entrar nelas.

### Suporte a Orientação do Sensor (90° Rotacionado / Normal):
- **Sensor 90° (Padrão Chassi):** Mapeia o eixo lateral da pista no eixo $Y$ do sensor ($0..119$) e a profundidade no eixo $X$ ($0..159$).
- **Sensor Normal:** Mapeia lateral em $X$ ($0..159$) e profundidade em $Y$ ($0..119$).
- Pode ser alternado dinamicamente com 1 clique no painel web.

---

## ⚡ Identificação e Antecipação Preditiva de Curvas

- **Cálculo da Curvatura ($\Delta E$):**
  $$\Delta E = E_{\text{far}} - E_{\text{base}}$$
- **Classificação Automática:**
  - $|\Delta E| < 8$: `RETA` (Aceleração máxima na velocidade base).
  - $8 \le |\Delta E| < 20$: `CURVA SUAVE` (Correção suave e leve modulação de PWM).
  - $|\Delta E| \ge 20$: `CURVA FECHADA` (Frenagem preditiva imediata antes de entrar na curva!).
- **Blend de Esterçamento Composto:**
  $$E_{\text{composto}} = (1 - w_{\text{far}}) \cdot E_{\text{base}} + w_{\text{far}} \cdot E_{\text{far}}$$
- **Controlador PD Preditivo:**
  $$\text{Ângulo} = 90^\circ + (K_p \cdot E_{\text{composto}}) + (K_d \cdot \Delta E_{\text{composto}})$$

---

## 🔌 Pinagem do Hardware

### 1. Câmera OV2640 (Padrão AI-Thinker)
| Função | Pino GPIO | Função | Pino GPIO |
| :--- | :--- | :--- | :--- |
| **PWDN** | GPIO 32 | **Y5** | GPIO 21 |
| **RESET** | -1 (NC) | **Y4** | GPIO 19 |
| **XCLK** | GPIO 00 | **Y3** | GPIO 18 |
| **SIOD (SDA)** | GPIO 26 | **Y2** | GPIO 05 |
| **SIOC (SCL)** | GPIO 27 | **VSYNC** | GPIO 25 |
| **Y9** | GPIO 35 | **HREF** | GPIO 23 |
| **Y8** | GPIO 34 | **PCLK** | GPIO 22 |
| **Y7** | GPIO 39 | **Y6** | GPIO 36 |

### 2. Atuadores (Servo & Motores Traseiros)
| Componente | Pino ESP32-CAM | Função / Sinal |
| :--- | :--- | :--- |
| **Servo de Direção** | **GPIO 02** | PWM LEDC (50 Hz, 14-bit, 50° a 130°) |
| **Ponte H - IN1** | **GPIO 14** | Motor Esquerdo - Avanço (PWM) |
| **Ponte H - IN2** | **GPIO 15** | Motor Esquerdo - Direção (GND/LOW) |
| **Ponte H - IN3** | **GPIO 13** | Motor Direito - Avanço (PWM) |
| **Ponte H - IN4** | **GPIO 12** | Motor Direito - Direção (GND/LOW) |

---

## 🌐 Dashboard Web e API REST

A interface web é **100% offline** (armazenada na memória Flash `PROGMEM`), estilizada em **Dark Theme** moderno e totalmente responsiva.

- **SSID do Hotspot:** `firula`
- **Senha:** `bebop123`
- **Endereço mDNS:** `http://carrinho.local/` (ou IP local via DHCP, exibido no Monitor Serial)
- **Porta do Painel:** `80` (`http://carrinho.local/`)
- **Porta do Streaming:** `81` (`http://carrinho.local:81/stream`)
- **Configurações Ideais Padrão:**
  - **Velocidade Base (PWM):** `180` (0 a 255)
  - **Sensibilidade Proporcional ($K_p$):** `0.70`
  - **Amortecimento Derivativo ($K_d$):** `0.45`
  - **Lookahead Antecipação ($W_{\text{far}}$):** `0.35` (35% de peso na zona longe)
  - **Torque Mínimo ($PWM_{\text{mín}}$):** `120`
  - **Threshold:** `0` (Automático adaptativo por contraste Min-Max)

### Rotas e Endpoints HTTP:
1. **`GET http://carrinho.local/`** : Painel de controle com vídeo ao vivo, vetor de curva, status multi-zona e calibração.
2. **`GET http://carrinho.local:81/stream`** : Streaming MJPEG com overlay das 3 zonas e trajetória.
3. **`GET http://carrinho.local/data`** : Telemetria completa (erro composto, erros individuais, curvatura, status da pista, PWM, ângulo, FPS).
4. **`GET http://carrinho.local/tune?kp=...&kd=...&th=...&speed=...&wfar=...&orient=...&inv_servo=...`** : Calibração dinâmica em tempo de execução.
5. **`GET http://carrinho.local/cmd?action=start|stop`** : Habilita ou desabilita a tração dos motores.

---

## 🚀 Como Compilar e Gravar no ESP32-CAM

### Configurações na Arduino IDE:
- **Placa:** `AI Thinker ESP32-CAM`
- **CPU Frequency:** `240MHz (WiFi/BT)`
- **Flash Frequency:** `80MHz`
- **Flash Mode:** `QIO`
- **Partition Scheme:** `Huge APP (3MB No OTA/1MB SPIFFS)`
- **PSRAM:** `Enabled`
- **Upload Speed:** `115200` ou `921600`

### Conexão USB-TTL (FTDI):
- `ESP32-CAM 5V` $\rightarrow$ `FTDI 5V`
- `ESP32-CAM GND` $\rightarrow$ `FTDI GND`
- `ESP32-CAM U0R (RX)` $\rightarrow$ `FTDI TX`
- `ESP32-CAM U0T (TX)` $\rightarrow$ `FTDI RX`
- **Conectar GPIO 0 com GND** durante a gravação (modo bootloader).
