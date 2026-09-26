# Carrinho Seguidor de Linha Autônomo com ESP32-CAM

Sistema embarcado para carrinho seguidor de linha autônomo baseado no microcontrolador ESP32-CAM (módulo AI-Thinker com sensor OV2640). O projeto combina visão computacional em tempo real, arquitetura multitarefa preemptiva (FreeRTOS Dual-Core), controle proporcional-derivativo (PD) adaptativo com diferencial eletrônico de tração e servidores HTTP independentes para telemetria, sintonia e transmissão de vídeo MJPEG com sobreposição gráfica.

---

## Sumário

- [Visão Geral](#visão-geral)
- [Arquitetura de Software (Dual-Core FreeRTOS)](#arquitetura-de-software-dual-core-freertos)
- [Especificação de Hardware e Pinagem](#especificação-de-hardware-e-pinagem)
- [Pipeline de Visão Computacional](#pipeline-de-visão-computacional)
- [Algoritmo de Controle e Atuação](#algoritmo-de-controle-e-atuação)
- [Interface Web e Endpoints da API](#interface-web-e-endpoints-da-api)
- [Requisitos e Configuração do Ambiente](#requisitos-e-configuração-do-ambiente)
- [Gravação do Firmware](#gravação-do-firmware)
- [Instruções de Uso e Calibração](#instruções-de-uso-e-calibração)
- [Estrutura do Repositório](#estrutura-do-repositório)

---

## Visão Geral

Diferente de seguidores de linha tradicionais baseados em sensores reflexivos infravermelhos discretos, este projeto utiliza uma câmera digital OV2640 para rastreamento visual contínuo da pista.

A imagem é capturada em resolução QQVGA (160x120 pixels) em escala de cinza e processada por uma rotina multi-zona que analisa diferentes profundidades da pista. Isso permite ao robô não apenas reagir ao desvio atual sob as rodas dianteiras, mas antecipar curvas à distância (lookahead), adaptando a velocidade dos motores traseiros e aplicando diferencial de giro proporcional ao ângulo de esterçamento das rodas dianteiras.

---

## Arquitetura de Software (Dual-Core FreeRTOS)

O firmware faz uso explícito dos dois núcleos de processamento do ESP32 através do FreeRTOS, garantindo determinismo temporal na malha de controle sem sofrer interferência do tráfego de rede Wi-Fi e das requisições HTTP.

```
                          ESP32 DUAL-CORE
       ┌──────────────────────────────────────────────────┐
       │                                                  │
       │  CORE 1: Visão Computacional & Controle (30 ms)  │
       │  ├── Captura QQVGA Grayscale (fb_count=2, PSRAM) │
       │  ├── Limiarização (Threshold Adaptativo/Fixo)    │
       │  ├── Varredura Multi-Zona (Base, Meio, Longe)    │
       │  ├── Controle PD com Ganho Progressivo           │
       │  ├── Diferencial Eletrônico & Frenagem em Curva  │
       │  └── Renderização de Overlay Visual no Buffer    │
       │                                                  │
       ├──────────────────────────────────────────────────┤
       │           Sincronização Thread-Safe:             │
       │  - portMUX_TYPE (Seções Críticas / Configurações)│
       │  - xSemaphoreCreateMutex (Buffer JPEG do Stream) │
       ├──────────────────────────────────────────────────┤
       │                                                  │
       │  CORE 0: Conectividade & Servidores Web          │
       │  ├── Gerenciamento Wi-Fi STA (DHCP) + mDNS       │
       │  ├── Porta 80: Dashboard Web & API REST          │
       │  │   ├── GET /       (Dashboard HTML/CSS/JS)     │
       │  │   ├── GET /data   (Telemetria JSON)           │
       │  │   ├── GET /tune   (Calibração Dinâmica)       │
       │  │   └── GET /cmd    (Comandos de Operação)      │
       │  └── Porta 81: Servidor Dedicado de Streaming    │
       │      └── GET /stream (Transmissão MJPEG)         │
       │                                                  │
       └──────────────────────────────────────────────────┘
```

### Mecanismos de Concorrência

- **Seções Críticas (`portENTER_CRITICAL` / `portEXIT_CRITICAL`):** Utilizadas para leitura e escrita atômica das estruturas de configuração (`g_config`) e telemetria (`g_telemetria`), evitando race conditions entre o ciclo de controle do Core 1 e os handlers de API do Core 0.
- **Mutex (`xSemaphoreCreateMutex`):** Protege o buffer compartilhado (`g_shared_frame`) contendo o quadro compactado em JPEG com overlay gráfico gerado no Core 1 e consumido sob demanda pelo servidor de streaming na porta 81 do Core 0.

---

## Especificação de Hardware e Pinagem

### Tabela de Ligações

| Componente | Função | Pino GPIO | Detalhes de Configuração |
|---|---|---|---|
| Servo Dianteiro | Esterçamento / Direção | GPIO 2 | Sinal PWM via LEDC (Canal 2, 50 Hz, resolução 14 bits) |
| Ponte H (IN1) | Motor Esquerdo - Avanço | GPIO 14 | Sinal PWM via LEDC (Canal 4, 5 kHz, resolução 8 bits) |
| Ponte H (IN2) | Motor Esquerdo - Direção | GPIO 15 | Saída Digital (Mantido em nível LOW para avanço) |
| Ponte H (IN3) | Motor Direito - Avanço | GPIO 13 | Sinal PWM via LEDC (Canal 5, 5 kHz, resolução 8 bits) |
| Ponte H (IN4) | Motor Direito - Direção | GPIO 12 | Saída Digital (Nível LOW na inicialização) |
| Flash LED | Iluminação Auxiliar / Status | GPIO 4 | Saída Digital (Active HIGH, pulsos de confirmação de rede) |
| LED Onboard | Indicador Interno | GPIO 33 | Saída Digital (Active LOW, mantido desligado no ciclo) |
| Câmera OV2640 | Barramento D0 a D7 | GPIO 5, 18, 19, 21, 36, 39, 34, 35 | Barramento paralelo de dados do sensor |
| Câmera OV2640 | Sinais de Sincronismo | GPIO 25 (VSYNC), 23 (HREF), 22 (PCLK) | Sincronismo vertical, horizontal e clock de pixel |
| Câmera OV2640 | Barramento SCCB (I2C) | GPIO 26 (SIOD), GPIO 27 (SIOC) | Interface de controle e configuração do sensor |
| Câmera OV2640 | Clock Principal (XCLK) | GPIO 0 | Frequência de 20 MHz (LEDC Canal 0 / Timer 0) |
| Câmera OV2640 | Power Down (PWDN) | GPIO 32 | Controle de energia do sensor |

### Cuidados Elétricos e Pinos Especiais

- **GPIO 12 (Strapping Pin):** Define a tensão interna do regulador flash (VDD_SDIO) durante a inicialização do ESP32. Se mantido em nível lógico alto durante o reset, o microcontrolador pode não inicializar. O código força este pino imediatamente para nível baixo (`LOW`) na função de inicialização de hardware.
- **Detector de Brownout:** Motores DC sob carga podem introduzir transientes na linha de alimentação, gerando resets espúrios causados pelo circuito de brownout. O firmware desativa o registrador de brownout (`WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0)`) na inicialização. Recomenda-se ainda o uso de fontes de alimentação separadas ou baterias com reguladores independentes para a lógica (ESP32) e para a potência (Ponte H / motores), com o GND em comum.

---

## Pipeline de Visão Computacional

O processamento é executado a cada ~30 ms no Core 1 diretamente sobre os pixels em escala de cinza:

1. **Configuração da Câmera:**
   - Resolução: `FRAMESIZE_QQVGA` (160 x 120 pixels).
   - Formato: `PIXFORMAT_GRAYSCALE` (1 byte por pixel, intensidade 0 a 255).
   - Buffers: Alocação dupla (`fb_count = 2`) em PSRAM com descarte automático de quadros antigos (`CAMERA_GRAB_LATEST`).

2. **Thresholding (Binarização):**
   - **Modo Dinâmico (Padrão, `th = 0`):** Calcula os valores mínimo e máximo de intensidade na zona de trabalho central da pista e define o limiar como `(min_val + max_val) / 2`, adaptando-se a variações graduais de iluminação ambiente.
   - **Modo Fixo (`th > 0`):** Aplica um limiar constante definido pelo usuário através do dashboard web.

3. **Arquitetura Multi-Zona e Lookahead:**
   A área visível é dividida em três regiões ao longo do eixo de profundidade da pista:
   - **Zona Base (Perto):** Localizada próxima ao para-choque do veículo. Determina o erro imediato de alinhamento lateral ($E_{base}$).
   - **Zona Meio (Intermediária):** Estabiliza a transição entre o veículo e a continuidade da pista ($E_{mid}$).
   - **Zona Longe (Antecipação / Lookahead):** Rastreia a linha à frente ($E_{far}$).
   - **Cálculo de Curvatura:** A diferença $\Delta E = E_{far} - E_{base}$ indica a curvatura da pista, classificando o trecho em reta, curva suave ou curva fechada.

4. **Continuidade Espacial (Dynamic Spatial ROI):**
   O algoritmo restringe a janela de busca de cada zona ao redor da posição do centróide identificado no quadro anterior. Isso reduz o tempo computacional de varredura e evita falsos positivos gerados por sombras ou marcas externas à pista.

5. **Modos de Orientação:**
   - **Paisagem (Landscape, 160x120):** Sensor montado horizontalmente. O eixo lateral corresponde a X (0 a 159, centro 80) e a profundidade corresponde a Y.
   - **Retrato (Rotacionado 90°, 120x160):** Sensor montado na vertical. O eixo lateral corresponde a Y (0 a 119, centro 60) e a profundidade corresponde a X.

6. **Overlay Gráfico no Stream:**
   O Core 1 desenha caixas delimitadoras das três zonas, marcadores de centróide e uma linha vetorial que representa a direção calculada antes de comprimir o quadro para JPEG.

---

## Algoritmo de Controle e Atuação

### Controle da Direção (Servo Dianteiro)

O ângulo do servo é calculado por uma malha Proporcional-Derivativa (PD) baseada no erro composto ponderado:

$$E_{composto} = (1 - w_{far}) \cdot E_{base} + w_{far} \cdot E_{far}$$

Onde $w_{far}$ é o peso de antecipação configurável (padrão: 0.35).

Para lidar com transições rápidas e curvas fechadas, o ganho proporcional ($K_p$) é ajustado dinamicamente: quando o erro ou a curvatura superam um limiar, um ganho progressivo não-linear é adicionado à autoridade de esterçamento:

$$\text{Ângulo} = \text{Centro} \pm (K_{p,\text{efetivo}} \cdot E_{composto} + K_d \cdot (E_{composto} - E_{\text{anterior}}))$$

O ângulo resultante é limitado pela calibração física do servo (50° a 130°, com 90° representando o centro neutro).

### Controle de Tração

A tração dos dois motores DC opera com duas lógicas combinadas:

1. **Frenagem Preditiva em Curvas:** A velocidade base (PWM) sofre redução proporcional de até 25% quando o desvio ou a curvatura aumentam, garantindo que o veículo desacelere antes de adentrar a curva. Um piso mínimo de PWM (`pwmMinimo`) é respeitado para evitar perda de torque (motor stall).
2. **Diferencial Eletrônico:** Ao esterçar, a velocidade da roda interna à curva é reduzida e a velocidade da roda externa recebe um incremento compensatório, reduzindo o raio de giro e prevenindo o subesterçamento (saída de frente).

### Sistema de Segurança e Failsafe

Se a linha preta não for detectada em nenhuma das três zonas por um período superior a 350 ms (`TIMEOUT_PERDA_LINHA`), os motores traseiros são desativados imediatamente, mantendo o servo no último ângulo conhecido.

---

## Interface Web e Endpoints da API

O firmware executa dois servidores HTTP paralelos na pilha nativa `esp_http_server.h`:

### Servidor de Aplicação (Porta 80)

| Rota | Método | Descrição | Formato / Resposta |
|---|---|---|---|
| `/` | `GET` | Carrega o painel de controle web completo (HTML5/CSS3/JavaScript embarcado em PROGMEM). | `text/html` |
| `/data` | `GET` | Retorna telemetria atual do robô em alta frequência. Suporta CORS. | `application/json` |
| `/tune` | `GET` | Atualiza parâmetros de controle sem reiniciar o microcontrolador. | `text/plain` |
| `/cmd` | `GET` | Envia comandos de início e parada para a tração. | `text/plain` |

#### Parâmetros aceitos pelo endpoint `/tune`

- `kp` (float): Ganho proporcional da direção (ex: `0.70`).
- `kd` (float): Ganho derivativo da direção (ex: `0.45`).
- `th` (int): Limiar de binarização (0 para modo adaptativo automático, 1 a 255 para limiar fixo).
- `speed` (int): Velocidade base em PWM (0 a 255).
- `wfar` (float): Fator de peso da zona de antecipação (0.0 a 0.6).
- `orient` (int): Orientação da câmera (`0` para Paisagem, `1` para Retrato 90°).
- `inv_servo` (int): Inversão do sentido de rotação do servo (`0` normal, `1` invertido).

#### Parâmetros aceitos pelo endpoint `/cmd`

- `action=start`: Habilita a tração dos motores.
- `action=stop`: Desabilita a tração e freia os motores imediatamente.

#### Exemplo de Resposta de Telemetria (`/data`)

```json
{
  "erro": 4.12,
  "erro_base": 3.50,
  "erro_mid": 4.20,
  "erro_far": 5.80,
  "curvatura": 2.30,
  "status_pista": 1,
  "angulo": 93.2,
  "pwm": 175,
  "fps": 28.4,
  "detectada": true,
  "base_ok": true,
  "mid_ok": true,
  "far_ok": true,
  "status": "RODANDO",
  "th": 88,
  "pixels": 420
}
```

### Servidor Dedicado de Vídeo (Porta 81)

| Rota | Método | Descrição | Formato |
|---|---|---|---|
| `/stream` | `GET` | Transmissão contínua MJPEG dos quadros capturados com sobreposição gráfica (overlay) em tempo real. | `multipart/x-mixed-replace;boundary=123456789000000000000987654321` |

A separação do streaming na porta 81 garante que o fluxo de vídeo não bloqueie as requisições de telemetria e calibração na porta 80.

---

## Requisitos e Configuração do Ambiente

### Requisitos de Software

- Arduino IDE (versão 1.8.19 ou 2.x) ou extensão Arduino para VS Code / PlatformIO.
- Pacote de placas ESP32 para Arduino (`esp32` by Espressif Systems), versão 2.0.x ou 3.0.x.

### Configurações da Placa na Arduino IDE

No menu **Ferramentas (Tools)**, selecione os seguintes parâmetros:

- **Board:** `AI Thinker ESP32-CAM`
- **CPU Frequency:** `240MHz (WiFi/BT)`
- **Flash Frequency:** `80MHz`
- **Flash Mode:** `QIO`
- **Partition Scheme:** `Huge APP (3MB No OTA/1MB SPIFFS)` ou `Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)`
- **Core Debug Level:** `Nenhum` (ou `Aviso` durante depuração)
- **PSRAM:** `Enabled` (Obrigatório para alocação de frame buffers)

---

## Gravação do Firmware

Como o módulo ESP32-CAM AI-Thinker não possui interface USB integrada, a gravação deve ser feita com uma base adaptadora (ESP32-CAM-MB) ou conversor serial USB-TTL (FTDI):

1. **Conexões do Conversor FTDI:**
   - FTDI `VCC (5V)` -> ESP32-CAM `5V` (utilize alimentação estável de pelo menos 1A a 2A).
   - FTDI `GND` -> ESP32-CAM `GND`.
   - FTDI `TX` -> ESP32-CAM `U0R` (GPIO 3).
   - FTDI `RX` -> ESP32-CAM `U0T` (GPIO 1).
   - Conecte o pino `GPIO 0` ao `GND` do ESP32-CAM para habilitar o modo de gravação (bootloader).
2. Pressione o botão **RESET** do ESP32-CAM.
3. Na Arduino IDE, selecione a porta COM correspondente e clique em **Carregar (Upload)**.
4. Após o término da gravação, **remova o jumper entre GPIO 0 e GND** e pressione o botão **RESET** novamente para iniciar a execução normal.

---

## Instruções de Uso e Calibração

1. **Configuração de Rede:**
   Abra o arquivo `espCarrinho.ino` e atualize as variáveis com o SSID e a senha do seu ponto de acesso Wi-Fi ou roteador:
   ```cpp
   const char *WIFI_SSID = "SEU_HOTSPOT";
   const char *WIFI_PASS = "SUA_SENHA";
   ```
2. **Inicialização:**
   Ao ligar o carrinho e conectar à rede, o LED Flash (GPIO 4) piscará quatro vezes consecutivas confirmando a conexão com sucesso.
3. **Acesso ao Dashboard:**
   Abra um navegador em um dispositivo conectado à mesma rede e acesse:
   - Pelo endereço mDNS: `http://carrinho.local/`
   - Ou pelo IP atribuído exibido no Monitor Serial (ex: `http://192.168.43.100/`).
4. **Calibração:**
   - Verifique pelo streaming se as marcações de zona (verde/amarelo/ciano) estão cobrindo a linha preta adequadamente.
   - Caso a iluminação varie, alterne entre o modo de Threshold Dinâmico (`th = 0`) ou ajuste um valor fixo pelo slider.
   - Ajuste os ganhos $K_p$ e $K_d$ para obter respostas firmes nas curvas sem gerar oscilações na reta.
   - Utilize os botões **INICIAR TRAÇÃO** e **PARADA DE EMERGÊNCIA** para testes na pista.

---

## Estrutura do Repositório

```
.
├── LICENSE                 # Licença MIT do projeto
├── README.md               # Documentação técnica do projeto
├── espCarrinho.ino         # Código-fonte principal (FreeRTOS, Visão, Controle e Servidores)
├── index_html.h            # Interface Web responsiva em PROGMEM (HTML5, CSS3, JavaScript)
└── prompt/
    └── prompt-web.md       # Especificações técnicas e diretrizes de desenvolvimento
```

---

## Licença

Este projeto está licenciado sob a Licença MIT. Veja o arquivo [LICENSE](file:///c:/Users/ihenr/Desktop/Faculdade/Redes/espCarrinho/LICENSE) para mais detalhes.

