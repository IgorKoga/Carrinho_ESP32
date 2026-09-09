Você é um Engenheiro Especialista em Sistemas Embarcados, Robótica e Visão Computacional. Sua tarefa é desenvolver o código-fonte C++ completo, modular e compilável na Arduino IDE para um carrinho seguidor de linha autônomo baseado no ESP32-CAM (módulo AI-Thinker).

---

### 1. ESPECIFICAÇÃO DE HARDWARE E PINAGEM

* **Microcontrolador:** ESP32-CAM (AI-Thinker, OV2640, PSRAM habilitada).
* **Direção Dianteira:** 1 Micro Servo Motor (SG90/MG90S) no **GPIO 2** (sinal PWM via canal `ledc` ou biblioteca compatível, 50 Hz).
* **Tração Traseira:** 2 Motores DC controlados por Ponte H nos pinos:
  * `IN1`: **GPIO 14** (Motor Esquerdo - Sentido/PWM)
  * `IN2`: **GPIO 15** (Motor Esquerdo - Sentido/PWM)
  * `IN3`: **GPIO 13** (Motor Direito - Sentido/PWM)
  * `IN4`: **GPIO 12** (Motor Direito - Sentido/PWM)
  * *Observação crítica:* O código deve inicializar o GPIO 12 explicitamente em nível lógico BAIXO (LOW) na inicialização.

---

### 2. ARQUITETURA MULTICORE (FreeRTOS)

O sistema deve operar em malha multithread estrita para garantir latência nula de controle:
* **Core 1 (Visão e Controle em Tempo Real):**
  * Captura de quadros da câmera em buffer local.
  * Pipeline de visão computacional (ROI + Centróide).
  * Malha de controle PD para cálculo de ângulo e velocidade.
  * Modulação PWM direta para o servo e para os motores da ponte H.
  * Taxa alvo: **25 a 30 iterações por segundo**.
* **Core 0 (Comunicação e Servidor Web):**
  * Gerenciamento de conexão Wi-Fi com IP Estático.
  * Execução do servidor HTTP nativo (`esp_http_server.h`).
  * Envio de frames no formato MJPEG na rota `/stream`.
  * Fornecimento de telemetria JSON e recepção de comandos de calibração via API.

---

### 3. PROCESSAMENTO DE IMAGEM EMBARCADO (Core 1)

1. **Configuração da Câmera:**
   * Resolução: `FRAMESIZE_QQVGA` (160x120 pixels).
   * Formato de Pixel: `PIXFORMAT_GRAYSCALE` (1 byte por pixel).
   * Frame Buffers: `fb_count = 2` com alocação em PSRAM.
2. **Região de Interesse (ROI):**
   * Processar exclusivamente as linhas horizontais entre `Y = 90` e `Y = 110` (fundo do frame, próximo às rodas dianteiras).
3. **Binarização e Centróide:**
   * Aplicar limiar ajustável (*Threshold*, padrão: 80) para isolar a linha preta em fundo claro.
   * Calcular o centro de massa horizontal ($X_{\text{centróide}}$):
     $$X_{\text{centróide}} = \frac{\sum (x \cdot I(x))}{\sum I(x)}$$
   * Calcular o erro relativo ao centro da imagem: $\text{Erro} = X_{\text{centróide}} - 80$ (faixa de $-80$ a $+80$).
   * Failsafe: Se nenhum pixel escuro for detectado na ROI (linha perdida), manter o último ângulo conhecido por 300 ms; persistindo a perda, desligar a tração dos motores traseiros.

---

### 4. ALGORITMO DE CONTROLE PD

* **Controle de Direção (Servo Dianteiro):**
  * Equação: $\text{Ângulo} = \text{Centro} + (K_p \cdot \text{Erro}) + (K_d \cdot (\text{Erro} - \text{Erro Anterior}))$.
  * Ângulo calibrado: 90° (centro), limitado entre 50° (máx. esquerda) e 130° (máx. direita).
  * Ganhos iniciais padrão: $K_p = 0.65$, $K_d = 0.35$.
* **Controle de Tração (Ponte H):**
  * Velocidade base configurável (PWM 0-255, padrão: 150).
  * Redução diferencial adaptativa da velocidade durante curvas fechadas ($|\text{Erro}| > 35$).

---

### 5. CONECTIVIDADE E DASHBOARD EMBARCADO (Core 0)

1. **Configuração Wi-Fi:**
   * Conexão em modo Station (STA) ao hotspot do smartphone.
   * Definição de IP Fixo no código (ex: `192.168.43.100`, Gateway: `192.168.43.1`, Máscara: `255.255.255.0`).
2. **Página Web embutida (PROGMEM):**
   * Servir na rota raiz (`/`) uma página HTML5/CSS/JavaScript completa, contida em string raw literal (`R"rawliteral(...)rawliteral"`).
   * **Layout Dark Theme responsivo e offline** (sem CDNs ou bibliotecas externas):
     * Visualizador com tag `<img>` apontando para `/stream`.
     * Painel com mostradores em tempo real: Estado (RODANDO/PARADO), Erro da Linha, Ângulo do Servo e Leituras de FPS.
     * Botões interativos: Iniciar Tração, Parar Emergência.
     * Sliders em tempo real: Ajuste dinâmico de $K_p$, $K_d$, Threshold e Velocidade Base.
3. **Endpoints da API HTTP (`esp_http_server.h`):**
   * `GET /` : Retorna a interface web completa.
   * `GET /stream` : Envio contínuo MJPEG (`multipart/x-mixed-replace`).
   * `GET /data` : Retorna telemetria atualizada em JSON (`{"erro":..., "angulo":..., "status":..., "fps":...}`).
   * `GET /tune?kp=...&kd=...&th=...&speed=...` : Atualiza os parâmetros do algoritmo sem reiniciar.
   * `GET /cmd?action=start|stop` : Altera a flag de movimento dos motores.

---

### 6. DIRETRIZES DE SAÍDA DO CÓDIGO

* Forneça o código C++ completo no formato do Arduino IDE (`.ino`).
* Não utilize trechos omitidos com comentários do tipo `"adicione seu código aqui"`. Todos os manipuladores de rota HTTP, tarefas do FreeRTOS, rotinas de interrupção e structs devem estar integralmente codificados.
* Inclua comentários pontuais explicando a parametrização dos canais do driver `ledc` no ESP32 Core v2.x/v3.x e o uso de Mutex/Variáveis Atômicas para compartilhamento seguro de variáveis entre os dois núcleos.