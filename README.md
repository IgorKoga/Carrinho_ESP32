# 🏎️ Carrinho Seguidor de Linha Autônomo com ESP32-CAM

Firmware completo de alta performance para robô seguidor de linha autônomo com processamento 100% embarcado no módulo **ESP32-CAM (AI-Thinker OV2640)**, arquitetura dual-core com **FreeRTOS**, controle **PD**, servidor HTTP nativo (`esp_http_server.h`), streaming **MJPEG** em tempo real e **Painel Web Dark Theme 100% Offline**.

---

## 📋 Sumário
- [Arquitetura Multicore (FreeRTOS)](#-arquitetura-multicore-freertos)
- [Pinagem do Hardware](#-pinagem-do-hardware)
- [Dashboard Web e API REST](#-dashboard-web-e-api-rest)
- [Pipeline de Visão Computacional e Controle PD](#-pipeline-de-visão-computacional-e-controle-pd)
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
 │ • ROI inferior (Y: 90 a 110) │ • IP Estático 192.168.43.50  │
 │ • Binarização e Centróide Cx │ • Servidor HTTP (Porta 80)   │
 │ • Controle PD do Servo       │ • Streaming MJPEG (/stream)  │
 │ • Desaceleração em Curvas    │ • Telemetria JSON (/data)    │
 │ • Loop Estrito: ~30 ms       │ • Calibração Dinâmica (/tune)│
 │ • Failsafe de Perda de Linha │ • Comandos Tração (/cmd)     │
 └──────────────────────────────┴──────────────────────────────┘
```

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

> ⚠️ **Proteção de Boot (GPIO 12):** O código inicializa o GPIO 12 e todos os pinos de saída explicitamente em nível `LOW` logo no início de `setup()` para evitar falhas de bootstrap do ESP32 e acionamentos espúrios dos motores.

---

## 🌐 Dashboard Web e API REST

A interface web é **100% offline** (armazenada na memória Flash `PROGMEM`), estilizada em **Dark Theme** moderno e totalmente responsiva para notebooks, tablets e smartphones.

- **SSID do Hotspot:** `Redmi Note 10S`
- **Senha:** `monobola8`
- **IP Estático Fixo:** `http://10.164.64.50`
- **Porta do Painel:** `80`
- **Porta do Streaming:** `81` (`http://10.164.64.50:81/stream`)

### Rotas e Endpoints HTTP:
1. **`GET http://10.164.64.50/`** : Painel de controle completo com:
   - Streaming de vídeo com overlay gráfico em tempo real.
   - Mostradores: Estado (RODANDO/PARADO), Erro da Linha, Ângulo do Servo, PWM e FPS.
   - Botões de ação: **▶ INICIAR TRAÇÃO** e **🛑 PARAR EMERGÊNCIA**.
   - Sliders interativos para calibração dinâmica (*on-the-fly*).
2. **`GET http://10.164.64.50:81/stream`** : Streaming de vídeo contínuo MJPEG dedicado.
3. **`GET http://10.164.64.50/data`** : Telemetria em tempo real no formato JSON.
4. **`GET http://10.164.64.50/tune?kp=...&kd=...&th=...&speed=...`** : Atualiza os parâmetros de calibração em tempo de execução sem reiniciar o microcontrolador.
5. **`GET http://10.164.64.50/cmd?action=start|stop`** : Habilita ou desabilita o acionamento dos motores traseiros.

---

## 👁️ Pipeline de Visão Computacional e Controle PD

1. **Captura:** Imagem capturada em resolução **QQVGA (160x120)** em escala de cinza de 8 bits (`PIXFORMAT_GRAYSCALE`).
2. **ROI (Região de Interesse):** Processa exclusivamente as linhas horizontais entre $Y = 90$ e $Y = 110$.
3. **Binarização Rápida:** Pixel com intensidade $< \text{Threshold}$ (padrão: 80) é classificado como linha preta.
4. **Cálculo do Centróide ($X_{\text{centróide}}$):**
   $$X_{\text{centróide}} = \frac{\sum (x \cdot I(x))}{\sum I(x)}$$
   $$\text{Erro} = X_{\text{centróide}} - 80 \quad (\text{faixa de } -80 \text{ a } +80)$$
5. **Controlador PD de Direção:**
   $$\text{Ângulo} = 90^\circ + (K_p \cdot \text{Erro}) + (K_d \cdot (\text{Erro} - \text{Erro Anterior}))$$
   Limitado rigidamente entre $50^\circ$ e $130^\circ$.
6. **Controle Adaptativo de Velocidade:**
   - Em retas ($|\text{Erro}| \le 35$): Opera na velocidade base (padrão PWM 150).
   - Em curvas fechadas ($|\text{Erro}| > 35$): Redução diferencial suave proporcional à curvatura.
7. **Failsafe (Perda de Linha):** Se nenhum pixel escuro for detectado na ROI por mais de $300\text{ ms}$, os motores são desligados imediatamente e o servo é centralizado em $90^\circ$.

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

---

## 🔋 Alimentação e Cuidados Elétricos

- **ESP32-CAM:** Requer alimentação dedicada de **5V @ 2A** (recomenda-se conversor Step-Down LM2596/MP1584).
- **Motores DC & Ponte H:** Devem ser alimentados por bateria/fonte separada.
- **GND Comum:** É obrigatório interligar todos os terminais GND do sistema.
