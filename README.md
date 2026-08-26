# 🏎️ Carrinho Seguidor de Linha com ESP32-CAM & Interface Web HUD

Este repositório contém o código-fonte completo (Firmware Arduino IDE, Servidor Node.js e Dashboard Web em tempo real) para a construção de um **Carrinho Seguidor de Linha Autônomo** utilizando a placa **ESP32-CAM** e a ponte H **L298N**.

---

## 🛠️ Arquitetura e Recursos

- **🚗 Modo Autônomo Inteligente**: Processamento de visão computacional em tempo real direto na câmera OV2640 do ESP32-CAM com algoritmo de controle **PID ($K_p, K_i, K_d$)**.
- **🎮 Modo Manual**: Opção na interface web para assumir o controle do carrinho em tempo real via joystick virtual na tela ou teclas do teclado (`W`, `A`, `S`, `D` / Setas).
- **📺 Stream de Vídeo & Visão Computacional**: Transmissão MJPEG em tempo real com overlay no navegador indicando a linha detectada, ponto focal de tomada de decisão e vetor de desvio.
- **⚙️ Ajuste Fino ao Vivo**: Modifique os parâmetros de ganho PID ($K_p, K_i, K_d$), velocidade máxima (PWM) e limiar de binarização de imagem (*Threshold*) diretamente pelo painel web, sem necessidade de rebinarizar ou regravar o código.

---

## 🔌 Esquema de Pinagem (ESP32-CAM + Ponte H L298N)

### 1. Motores (Ponte H L298N)
| Pino L298N | Pino ESP32-CAM | Função |
| :--- | :--- | :--- |
| **IN1** | **GPIO 12** | Motor Esquerdo - Direção A / PWM |
| **IN2** | **GPIO 13** | Motor Esquerdo - Direção B / PWM |
| **IN3** | **GPIO 14** | Motor Direito - Direção A / PWM |
| **IN4** | **GPIO 15** | Motor Direito - Direção B / PWM |
| **GND** | **GND** | Terra Comum (ESP32 + Bateria + L298N) |
| **VCC (5V)** | **5V** | Alimentação do ESP32-CAM |

*Nota: Jumper ENA e ENB da ponte H L298N mantidos conectados (High), pois o controle de velocidade via PWM é realizado diretamente nos pinos IN1..IN4 pelo canal LEDC do ESP32.*

---

## 💻 1. Gravação do Firmware no ESP32-CAM (Arduino IDE)

1. Abra o arquivo [`esp32_cam_car/esp32_cam_car.ino`](esp32_cam_car/esp32_cam_car.ino) no **Arduino IDE**.
2. Certifique-se de ter o suporte a placas ESP32 instalado no Arduino IDE.
3. Em **Ferramentas (Tools)**, configure os seguintes parâmetros:
   - **Placa (Board)**: `AI Thinker ESP32-CAM`
   - **Esquema de Partição (Partition Scheme)**: `Huge APP (3MB No OTA/1MB SPIFFS)`
   - **PSRAM**: `Enabled`
   - **Porta (Port)**: Selecione a porta COM/TTY correspondente ao seu gravador FTDI / USB-TTL.
4. *(Opcional)* Se desejar conectar o carrinho à sua rede Wi-Fi existente, altere `USE_ACCESS_POINT = false` no topo do arquivo `.ino` e digite o `ssid` e `password` da sua rede.
5. Conecte o pino `GPIO 0` ao `GND` para entrar no modo de gravação, pressione o botão Reset no ESP32-CAM e clique em **Carregar (Upload)**.
6. Após a gravação concluída, remova o jumper entre `GPIO 0` e `GND` e reinicie a placa.

---

## 🌐 2. Executando o Servidor e Interface Web

Você pode abrir a interface web de duas formas:

### Opção A: Servidor Node.js Local (Recomendado)
1. Navegue até a pasta `server`:
   ```bash
   cd server
   ```
2. Instale as dependências:
   ```bash
   npm install
   ```
3. Inicie o servidor:
   ```bash
   npm start
   ```
4. Abra seu navegador em: `http://localhost:3000`

### Opção B: Direct Access
- Conecte o seu computador ou smartphone à rede Wi-Fi emitida pelo carrinho (`ESP32_Carrinho_Robo` com a senha `password123`).
- Digite o IP `192.168.4.1` no campo de IP da interface web para visualizar o stream e a telemetria.

---

## 🎯 Dicas de Calibração PID & Visão

1. **Threshold (Binarização)**:
   - Ajuste a barra de Threshold até que a fita preta seja claramente destacada no indicador de varredura.
2. **Ganho Proporcional ($K_p$)**:
   - Aumente o $K_p$ para dar mais resposta nas curvas. Se o carrinho começar a oscilar excessivamente de um lado para o outro ("ziguezague"), reduza o $K_p$.
3. **Ganho Derivativo ($K_d$)**:
   - Aumente o $K_d$ para amortecer as oscilações e suavizar a trajetória em retas.
