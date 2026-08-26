// ESP32-CAM Line Follower Dashboard Logic
document.addEventListener('DOMContentLoaded', () => {
  // Inicializa Ícones Lucide
  if (window.lucide) {
    lucide.createIcons();
  }

  // Elementos do DOM
  const espIpInput = document.getElementById('espIpInput');
  const btnConnect = document.getElementById('btnConnect');
  const statusBadge = document.getElementById('statusBadge');
  const statusText = document.getElementById('statusText');
  const modeBadge = document.getElementById('modeBadge');

  const mjpegStream = document.getElementById('mjpegStream');
  const overlayCanvas = document.getElementById('overlayCanvas');
  const scanlineCanvas = document.getElementById('scanlineCanvas');
  const streamPlaceholder = document.getElementById('streamPlaceholder');

  const fpsVal = document.getElementById('fpsVal');
  const valLinePos = document.getElementById('valLinePos');
  const valError = document.getElementById('valError');
  const lineDetectState = document.getElementById('lineDetectState');

  const btnModeAuto = document.getElementById('btnModeAuto');
  const btnModeManual = document.getElementById('btnModeManual');
  const dpadCard = document.getElementById('manualControls');

  const rngSpeed = document.getElementById('rngSpeed');
  const lblSpeed = document.getElementById('lblSpeed');
  const rngThresh = document.getElementById('rngThresh');
  const lblThresh = document.getElementById('lblThresh');
  const chkDarkLine = document.getElementById('chkDarkLine');
  const lblLineType = document.getElementById('lblLineType');

  const rngKp = document.getElementById('rngKp');
  const lblKp = document.getElementById('lblKp');
  const rngKi = document.getElementById('rngKi');
  const lblKi = document.getElementById('lblKi');
  const rngKd = document.getElementById('rngKd');
  const lblKd = document.getElementById('lblKd');
  const btnSaveSettings = document.getElementById('btnSaveSettings');

  const barMotorLeft = document.getElementById('barMotorLeft');
  const valMotorLeft = document.getElementById('valMotorLeft');
  const barMotorRight = document.getElementById('barMotorRight');
  const valMotorRight = document.getElementById('valMotorRight');

  const errorCanvas = document.getElementById('errorChart');
  const ctxError = errorCanvas.getContext('2d');
  const ctxOverlay = overlayCanvas.getContext('2d');
  const ctxScanline = scanlineCanvas.getContext('2d');

  // Estado da Aplicação
  let espIp = espIpInput.value.trim();
  let isConnected = false;
  let currentMode = 'AUTO';
  let telemetryTimer = null;
  let errorHistory = new Array(50).fill(0);

  // Inicializar Canvas
  function resizeCanvas() {
    overlayCanvas.width = overlayCanvas.clientWidth;
    overlayCanvas.height = overlayCanvas.clientHeight;
    scanlineCanvas.width = scanlineCanvas.clientWidth;
    scanlineCanvas.height = scanlineCanvas.clientHeight;
    errorCanvas.width = errorCanvas.clientWidth;
    errorCanvas.height = errorCanvas.clientHeight;
  }
  resizeCanvas();
  window.addEventListener('resize', resizeCanvas);

  // Conectar ao ESP32-CAM
  function connect() {
    espIp = espIpInput.value.trim();
    if (!espIp) return;

    // Configurar Fonte de Stream de Vídeo (Porta 81 por padrão ou Proxy)
    const streamUrl = `http://${espIp}:81/`;
    mjpegStream.src = streamUrl;

    mjpegStream.onload = () => {
      streamPlaceholder.style.display = 'none';
      setConnected(true);
    };

    mjpegStream.onerror = () => {
      // Se falhar o carregamento direto, exibe o placeholder mas tenta ler a telemetria
      streamPlaceholder.style.display = 'flex';
    };

    // Iniciar Pooling de Telemetria
    if (telemetryTimer) clearInterval(telemetryTimer);
    telemetryTimer = setInterval(fetchTelemetry, 200);
  }

  function setConnected(connected) {
    isConnected = connected;
    if (connected) {
      statusBadge.className = 'status-badge connected';
      statusText.textContent = 'Conectado';
    } else {
      statusBadge.className = 'status-badge disconnected';
      statusText.textContent = 'Desconectado';
    }
  }

  // Buscar Telemetria do ESP32
  async function fetchTelemetry() {
    try {
      let resp;
      try {
        resp = await fetch(`http://${espIp}/api/telemetry`, { signal: AbortSignal.timeout(1500) });
      } catch (e) {
        resp = await fetch(`/esp32-proxy/api/telemetry?espIp=${espIp}`, { signal: AbortSignal.timeout(1500) });
      }
      if (!resp.ok) throw new Error('HTTP Error');
      const data = await resp.json();

      setConnected(true);
      updateUIWithTelemetry(data);
    } catch (err) {
      setConnected(false);
    }
  }

  // Atualizar Elementos de Interface com os Dados do ESP32
  function updateUIWithTelemetry(data) {
    const camW = data.camWidth || 160;
    const camH = data.camHeight || 120;
    const targetCenter = camW / 2;

    // FPS e Métricas de Visão
    fpsVal.textContent = data.fps ? data.fps.toFixed(1) : '0.0';
    valLinePos.textContent = `${data.linePos || targetCenter} px`;
    valError.textContent = `${data.error || 0} px`;

    if (data.lineDetected) {
      lineDetectState.textContent = 'Linha Detectada';
      lineDetectState.style.background = 'rgba(0, 255, 135, 0.2)';
      lineDetectState.style.color = 'var(--accent-green)';
    } else {
      lineDetectState.textContent = 'Linha Perdida!';
      lineDetectState.style.background = 'rgba(255, 71, 87, 0.2)';
      lineDetectState.style.color = 'var(--accent-red)';
    }

    // Motores
    const leftPwm = data.leftSpeed || 0;
    const rightPwm = data.rightSpeed || 0;
    valMotorLeft.textContent = `${leftPwm} PWM`;
    valMotorRight.textContent = `${rightPwm} PWM`;
    barMotorLeft.style.width = `${Math.min(100, Math.abs(leftPwm) / 2.55)}%`;
    barMotorRight.style.width = `${Math.min(100, Math.abs(rightPwm) / 2.55)}%`;

    // Desenhar Visão e Gráfico
    drawOverlay(data.linePos || targetCenter, data.error || 0, data.lineDetected, camW, camH);
    drawScanlinePreview(data.linePos || targetCenter, data.thresh || 100, camW);
    pushErrorChart(data.error || 0);
  }

  // Desenhar Overlay no Vídeo
  function drawOverlay(linePos, error, detected, camW = 160, camH = 120) {
    const w = overlayCanvas.width;
    const h = overlayCanvas.height;
    ctxOverlay.clearRect(0, 0, w, h);

    if (w === 0 || h === 0) return;

    // Escala (Câmera -> Tela Canvas)
    const scaleX = w / camW;
    const scaleY = h / camH;
    const targetCenter = camW / 2;
    const scanY = camH * 0.75;

    // Linha Central Guia (Alvo)
    ctxOverlay.strokeStyle = 'rgba(0, 242, 254, 0.5)';
    ctxOverlay.setLineDash([5, 5]);
    ctxOverlay.lineWidth = 2;
    ctxOverlay.beginPath();
    ctxOverlay.moveTo(targetCenter * scaleX, 0);
    ctxOverlay.lineTo(targetCenter * scaleX, h);
    ctxOverlay.stroke();
    ctxOverlay.setLineDash([]);

    // Linha Y de Varredura
    ctxOverlay.strokeStyle = 'rgba(255, 165, 0, 0.6)';
    ctxOverlay.lineWidth = 1;
    ctxOverlay.beginPath();
    ctxOverlay.moveTo(0, scanY * scaleY);
    ctxOverlay.lineTo(w, scanY * scaleY);
    ctxOverlay.stroke();

    if (detected) {
      const posX = linePos * scaleX;
      const posY = scanY * scaleY;

      // Ponto focal da linha detectada
      ctxOverlay.fillStyle = '#00ff87';
      ctxOverlay.beginPath();
      ctxOverlay.arc(posX, posY, 8, 0, Math.PI * 2);
      ctxOverlay.fill();

      // Vetor de Erro
      ctxOverlay.strokeStyle = '#ff4757';
      ctxOverlay.lineWidth = 3;
      ctxOverlay.beginPath();
      ctxOverlay.moveTo(targetCenter * scaleX, posY);
      ctxOverlay.lineTo(posX, posY);
      ctxOverlay.stroke();
    }
  }

  // Desenhar Fita da Linha Binarizada
  function drawScanlinePreview(linePos, thresh, camW = 160) {
    const w = scanlineCanvas.width;
    const h = scanlineCanvas.height;
    ctxScanline.fillStyle = '#0a0e17';
    ctxScanline.fillRect(0, 0, w, h);

    const scaleX = w / camW;
    const lineWidthPx = 20 * scaleX;
    const posX = linePos * scaleX;

    // Desenhar pista simulada
    ctxScanline.fillStyle = '#1e293b';
    ctxScanline.fillRect(0, 0, w, h);

    // Desenhar linha
    ctxScanline.fillStyle = chkDarkLine.checked ? '#000000' : '#ffffff';
    ctxScanline.fillRect(posX - lineWidthPx / 2, 0, lineWidthPx, h);

    // Marcação do centro
    ctxScanline.strokeStyle = '#00f2fe';
    ctxScanline.lineWidth = 2;
    ctxScanline.beginPath();
    ctxScanline.moveTo(posX, 0);
    ctxScanline.lineTo(posX, h);
    ctxScanline.stroke();
  }

  // Desenhar Gráfico de Erro
  function pushErrorChart(errorVal) {
    errorHistory.push(errorVal);
    errorHistory.shift();

    const w = errorCanvas.width;
    const h = errorCanvas.height;
    ctxError.clearRect(0, 0, w, h);

    // Grid Central (Erro Zero)
    const centerY = h / 2;
    ctxError.strokeStyle = 'rgba(255, 255, 255, 0.1)';
    ctxError.beginPath();
    ctxError.moveTo(0, centerY);
    ctxError.lineTo(w, centerY);
    ctxError.stroke();

    // Desenhar curva do erro
    ctxError.strokeStyle = '#00f2fe';
    ctxError.lineWidth = 2;
    ctxError.beginPath();

    const stepX = w / (errorHistory.length - 1);
    for (let i = 0; i < errorHistory.length; i++) {
      const x = i * stepX;
      // Normalizar erro (-160 a +160) para altura da tela
      const normErr = errorHistory[i] / 160;
      const y = centerY + normErr * (h / 2 - 10);

      if (i === 0) ctxError.moveTo(x, y);
      else ctxError.lineTo(x, y);
    }
    ctxError.stroke();
  }

  // Alternar Modo de Operação (AUTO / MANUAL)
  function setMode(mode) {
    currentMode = mode;
    if (mode === 'AUTO') {
      btnModeAuto.classList.add('active');
      btnModeManual.classList.remove('active');
      dpadCard.classList.remove('enabled');
      modeBadge.className = 'mode-badge auto';
      modeBadge.innerHTML = '<i data-lucide="cpu"></i> MODO AUTÔNOMO';
    } else {
      btnModeManual.classList.add('active');
      btnModeAuto.classList.remove('active');
      dpadCard.classList.add('enabled');
      modeBadge.className = 'mode-badge manual';
      modeBadge.innerHTML = '<i data-lucide="gamepad-2"></i> MODO MANUAL';
    }
    if (window.lucide) lucide.createIcons();

    // Enviar comando ao ESP32
    fetch(`http://${espIp}/api/mode?val=${mode}`).catch(() => { });
  }

  btnModeAuto.addEventListener('click', () => setMode('AUTO'));
  btnModeManual.addEventListener('click', () => setMode('MANUAL'));

  // Controle Manual (D-Pad & Teclado)
  function sendControl(dir) {
    if (currentMode !== 'MANUAL') return;
    const speed = rngSpeed.value;
    fetch(`http://${espIp}/api/control?dir=${dir}&speed=${speed}`).catch(() => { });
  }

  document.querySelectorAll('.dpad-btn').forEach(btn => {
    btn.addEventListener('mousedown', () => sendControl(btn.dataset.dir));
    btn.addEventListener('mouseup', () => sendControl('STOP'));
    btn.addEventListener('touchstart', (e) => { e.preventDefault(); sendControl(btn.dataset.dir); });
    btn.addEventListener('touchend', () => sendControl('STOP'));
  });

  // Atalhos de Teclado (WASD / Setas)
  window.addEventListener('keydown', (e) => {
    if (currentMode !== 'MANUAL') return;
    const key = e.key.toLowerCase();
    if (key === 'w' || key === 'arrowup') sendControl('FORWARD');
    else if (key === 's' || key === 'arrowdown') sendControl('BACKWARD');
    else if (key === 'a' || key === 'arrowleft') sendControl('LEFT');
    else if (key === 'd' || key === 'arrowright') sendControl('RIGHT');
    else if (key === ' ') sendControl('STOP');
  });

  window.addEventListener('keyup', (e) => {
    if (currentMode !== 'MANUAL') return;
    sendControl('STOP');
  });

  // Sliders e Parâmetros
  rngSpeed.addEventListener('input', () => lblSpeed.textContent = rngSpeed.value);
  rngThresh.addEventListener('input', () => lblThresh.textContent = rngThresh.value);
  chkDarkLine.addEventListener('change', () => {
    lblLineType.textContent = chkDarkLine.checked ? 'Linha Preta (Fundo Claro)' : 'Linha Clara (Fundo Escuro)';
  });

  rngKp.addEventListener('input', () => lblKp.textContent = parseFloat(rngKp.value).toFixed(2));
  rngKi.addEventListener('input', () => lblKi.textContent = parseFloat(rngKi.value).toFixed(2));
  rngKd.addEventListener('input', () => lblKd.textContent = parseFloat(rngKd.value).toFixed(2));

  btnSaveSettings.addEventListener('click', () => {
    const params = new URLSearchParams({
      kp: rngKp.value,
      ki: rngKi.value,
      kd: rngKd.value,
      speed: rngSpeed.value,
      thresh: rngThresh.value,
      dark: chkDarkLine.checked
    });
    fetch(`http://${espIp}/api/settings?${params.toString()}`)
      .then(res => res.json())
      .then(() => alert('Configurações aplicadas com sucesso no ESP32-CAM!'))
      .catch(() => alert('Erro ao conectar ao ESP32-CAM para aplicar configurações.'));
  });

  btnConnect.addEventListener('click', connect);

  // Auto conectar ao carregar
  connect();
});
