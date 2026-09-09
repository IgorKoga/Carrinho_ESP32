```text
Você atuará como Engenheiro de Firmware Sênior especialista em ESP32, FreeRTOS e Visão Computacional Embarcada.
Sua tarefa é gerar o código-fonte completo e funcional (arquivo único .ino para Arduino IDE ou PlatformIO) para um Carrinho Seguidor de Linha Autônomo com processamento 100% embarcado no ESP32-CAM (módulo AI-Thinker OV2640).

### REQUISITOS OBRIGATÓRIOS DO FIRMWARE:

1. CONFIGURAÇÃO DE REDE E IP FIXO (CORE 0):
   - Conexão como estação Wi-Fi em um Hotspot móvel:
     * SSID: "CarrinhoSeguidor"
     * Password: "password123"
   - Configuração de IP Estático obrigatória:
     * IP: 192.168.43.50
     * Gateway: 192.168.43.1
     * Máscara: 255.255.255.0
     * DNS: 192.168.43.1
   - Inicializar um servidor HTTP leve na porta 80 que entregue um streaming MJPEG na rota "/stream".
   - O streaming deve ser gerado a partir do buffer capturado, permitindo ao usuário no notebook visualizar o que o robô está enxergando em tempo real.

2. PINAGEM DO HARDWARE:
   - Câmera: Mapeamento de pinos padrão do modelo AI-Thinker (OV2640).
   - Servo Motor de Direção Dianteira:
     * GPIO 02 -> Canal PWM LEDC (50 Hz, resolução de 14 bits ou biblioteca ESP32Servo).
   - Ponte H (Dois motores traseiros DC acoplados):
     * IN1: GPIO 14 (Motor Esquerdo)
     * IN2: GPIO 15 (Motor Esquerdo)
     * IN3: GPIO 13 (Motor Direito)
     * IN4: GPIO 12 (Motor Direito)
   - Respeitar a inicialização correta dos pinos para evitar acionamentos espúrios no bootloader.

3. ARQUITETURA MULTI-CORE COM FREERTOS:
   - Criar uma tarefa no Core 1 com prioridade alta (ex: prioridade 2): `taskControleVisao`.
     * Esta tarefa deve rodar em loop infinito com controle estrito de tempo de ciclo (~30 ms).
     * Não deve depender do Wi-Fi nem do servidor web para tomar decisões de direção.
   - O Core 0 deve gerenciar o loop do Wi-Fi e o atendimento às requisições do servidor HTTP.

4. PIPELINE DE VISÃO COMPUTACIONAL (CORE 1):
   - Inicializar a câmera com `FRAMESIZE_QQVGA` (160x120) e `PIXFORMAT_GRAYSCALE`.
   - Capturar o frame (`esp_camera_fb_get`).
   - Processar apenas a ROI inferior: linhas y de 90 a 119.
   - Aplicar binarização rápida: se valor do pixel < THRESHOLD (padrão: 80 para linha preta), considerar linha (1); senão fundo (0).
   - Calcular o centróide horizontal (Cx) usando a média ponderada dos índices das colunas.
   - Calcular o erro: erro = Cx - 80.
   - Devolver o frame buffer (`esp_camera_fb_return`).

5. CONTROLE PD E ATUADORES (CORE 1):
   - Controlador PD para o Servo de Direção:
     angulo_servo = 90 + (Kp * erro + Kd * (erro - erro_anterior))
     Limitar o ângulo entre 55° e 125°.
   - Controle de Velocidade:
     pwm_atual = PWM_BASE - (K_CURVA * abs(erro))
     Limitar pwm_atual entre PWM_MIN e PWM_BASE.
   - Direção dos motores: avanço para frente (IN1=HIGH/PWM, IN2=LOW, IN3=HIGH/PWM, IN4=LOW).
   - Lógica de perda de linha: Se nenhum pixel preto for detectado na ROI por mais de 300 ms, zerar o PWM dos motores e manter o servo centralizado (90°).

6. CALIBRAÇÃO E CONSTANTES CONFIGURÁVEIS:
   - Declarar no topo do código como `#define` ou `const` de fácil acesso:
     * Kp (sugestão inicial: 0.45)
     * Kd (sugestão inicial: 0.15)
     * THRESHOLD_LINHA (sugestão inicial: 80)
     * PWM_BASE (sugestão inicial: 180)
     * PWM_MIN (sugestão inicial: 110)
     * K_CURVA (sugestão inicial: 0.8)

ENTREGÁVEL:
Código C++ integral, limpo, bem documentado, sem trechos omitidos ou comentários como "// seu código aqui", estruturado para compilar diretamente na Arduino IDE com suporte a ESP32 instalado.
```