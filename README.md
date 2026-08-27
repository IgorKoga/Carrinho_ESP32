# 🏎️ Carrinho Seguidor de Linha com ESP32-CAM & Interface Web HUD

Este repositório contém o código-fonte completo (Firmware Arduino IDE, Servidor Node.js e Dashboard Web em tempo real) para a construção de um **Carrinho Seguidor de Linha Autônomo** utilizando a placa **ESP32-CAM** e a ponte H **L298N**.

---

## 🛠️ Arquitetura e Recursos

- **🚗 Modo Autônomo Inteligente**: Processamento de visão computacional em tempo real direto na câmera OV2640 do ESP32-CAM com algoritmo de controle **PID ($K_p, K_i, K_d$)**.
- **🎮 Modo Manual**: Opção na interface web para assumir o controle do carrinho em tempo real via joystick virtual na tela ou teclas do teclado (`W`, `A`, `S`, `D` / Setas).
- **📺 Stream de Vídeo & Visão Computacional**: Transmissão MJPEG em tempo real com overlay no navegador indicando a linha detectada, ponto focal de tomada de decisão e vetor de desvio.
- **⚙️ Ajuste Fino ao Vivo**: Modifique os parâmetros de ganho PID ($K_p, K_i, K_d$), velocidade máxima (PWM) e limiar de binarização de imagem (*Threshold*) diretamente pelo painel web, sem necessidade de rebinarizar ou regravar o código.
- **💡 Controle de Iluminação (Flash LED)**: Acionamento remoto do LED Flash de alta intensidade (GPIO 4) para testes em ambientes escuros.

---

## 📡 Arquitetura de Comunicação e Protocolo de Rede (TCP)

Toda a comunicação entre o carrinho ESP32-CAM e a interface web/servidor utiliza o protocolo **TCP (Transmission Control Protocol)** nas portas **80** (API HTTP REST) e **81** (Stream de Vídeo MJPEG).

### Por que o protocolo TCP é utilizado?

1. **Garantia de Entrega e Confiabilidade dos Comandos de Controle (Porta 80)**:
   - Os comandos enviados para o carrinho (como iniciar tração, ajustar esterçamento do servo, alternar para modo autônomo, ativar o flash e paradas de emergência) trafegam via requisições HTTP REST.
   - O **TCP** garante a entrega orientada a conexão, sem perda de pacotes, sem duplicação e na ordem exata em que foram enviados. Em um sistema de robótica, perder um pacote de parada (`STOP`) via UDP não confiável poderia causar colisões do carrinho.

2. **Integridade da Transmissão de Vídeo MJPEG (Porta 81)**:
   - O feed da câmera é transmitido como um fluxo contínuo HTTP *Multipart* (`multipart/x-mixed-replace`), onde cada quadro de imagem JPEG é enviado sequencialmente.
   - O TCP assegura que o fluxo de bytes da imagem chegue intacto ao navegador. Caso um segmento de dados da imagem fosse perdido (o que aconteceria no UDP sem tratamento), a imagem JPEG apresentaria artefatos visuais ou falha de decodificação no navegador.

3. **Compatibilidade Nativa com Navegadores Web**:
   - As APIs Web e os elementos HTML5 (como `<img>` para streams MJPEG e `fetch`/`XMLHttpRequest` para APIs REST) rodam nativamente sobre TCP via HTTP, eliminando a necessidade de protocolos proprietários ou adaptadores no cliente.

---

## 🔌 Esquema de Pinagem (ESP32-CAM + Ponte H L298N + Servomotor)

### 1. Motores de Tração (Ponte H L298N)
| Pino L298N | Pino ESP32-CAM | Função |
| :--- | :--- | :--- |
| **IN1** | **GPIO 14** | Motor Esquerdo - Direção A (PWM Canal 1) |
| **IN2** | **GPIO 15** | Motor Esquerdo - Direção B (PWM Canal 2) |
| **IN3** | **GPIO 13** | Motor Direito - Direção A (PWM Canal 3) |
| **IN4** | **GPIO 12** | Motor Direito - Direção B (PWM Canal 4) |
| **GND** | **GND** | Terra Comum (ESP32 + Bateria + L298N) |
| **VCC (5V)** | **5V / VCC** | Alimentação do ESP32-CAM |

### 2. Servomotor de Direção & Periféricos
| Periférico | Pino ESP32-CAM | Função |
| :--- | :--- | :--- |
| **Servo Sinal** | **GPIO 2** | Controle do ângulo de esterçamento (PWM Canal 5 / 50Hz) |
| **Flash LED** | **GPIO 4** | Iluminação auxiliar da câmera |

*Nota: Os jumpers ENA e ENB da ponte H L298N são mantidos conectados (High), pois o controle de velocidade via PWM é realizado diretamente nos pinos IN1..IN4 pelos canais LEDC do ESP32.*

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
# Carrinho_ESP32
