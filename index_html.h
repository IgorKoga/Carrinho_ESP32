#ifndef INDEX_HTML_H
#define INDEX_HTML_H

#include <pgmspace.h>

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP32-CAM | Painel Autônomo Seguidor de Linha</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:ital,wght@0,300;0,400;0,500;0,600;0,700;0,800;1,400;1,700&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg-base: #000000;
      --bg-panel: #050505;
      --bg-card: #080808;
      --border-color: #1a1a1a;
      --border-focus: #333333;
      --accent-cyan: #00f0ff;
      --accent-blue: #3b82f6;
      --accent-green: #10b981;
      --accent-yellow: #f59e0b;
      --accent-red: #ef4444;
      --text-main: #ffffff;
      --text-muted: #888888;
      --font-mono: 'JetBrains Mono', ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
    }
    * { 
      box-sizing: border-box; 
      margin: 0; 
      padding: 0; 
      font-family: var(--font-mono); 
    }
    body {
      font-family: var(--font-mono);
      background: var(--bg-base);
      color: var(--text-main);
      min-height: 100vh;
      padding: 16px;
    }
    .wrapper { max-width: 1280px; margin: 0 auto; }
    header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding-bottom: 14px;
      border-bottom: 1px solid var(--border-color);
      margin-bottom: 18px;
      flex-wrap: wrap;
      gap: 10px;
    }
    .logo-area { display: flex; align-items: center; gap: 10px; }
    .logo-area h1 { font-size: 1.35rem; font-weight: 700; color: #fff; }
    .logo-sub { font-size: 0.8rem; color: var(--accent-cyan); letter-spacing: 0.5px; }
    .status-badge {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      padding: 6px 14px;
      border-radius: 9999px;
      font-size: 0.82rem;
      font-weight: 700;
      background: #04140b;
      border: 1px solid rgba(16, 185, 129, 0.4);
      color: var(--accent-green);
    }
    .status-badge.stopped {
      background: #180506;
      border-color: rgba(239, 68, 68, 0.4);
      color: var(--accent-red);
    }
    .status-badge.curve {
      background: #1c1303;
      border-color: rgba(245, 158, 11, 0.4);
      color: var(--accent-yellow);
    }
    .dot-pulse {
      width: 8px; height: 8px; border-radius: 50%;
      background: currentColor;
      box-shadow: 0 0 10px currentColor;
    }
    .main-grid {
      display: grid;
      grid-template-columns: 1.15fr 1fr;
      gap: 18px;
    }
    @media (max-width: 960px) { .main-grid { grid-template-columns: 1fr; } }
    .card {
      background: var(--bg-card);
      border: 1px solid var(--border-color);
      border-radius: 12px;
      padding: 16px;
      box-shadow: 0 8px 24px rgba(0,0,0,0.8);
      margin-bottom: 16px;
    }
    .card-head {
      font-size: 0.92rem;
      font-weight: 700;
      color: var(--accent-cyan);
      margin-bottom: 12px;
      display: flex;
      align-items: center;
      justify-content: space-between;
    }
    .stream-box {
      background: #000000;
      border-radius: 8px;
      overflow: hidden;
      border: 1px solid var(--border-color);
      display: flex;
      justify-content: center;
      align-items: center;
      min-height: 240px;
    }
    .stream-box img {
      width: 100%;
      height: auto;
      display: block;
      image-rendering: pixelated;
    }
    .btn-group {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-top: 12px;
    }
    .btn {
      padding: 11px;
      border: none;
      border-radius: 8px;
      font-family: var(--font-mono);
      font-weight: 700;
      font-size: 0.85rem;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 6px;
      transition: all 0.2s ease;
    }
    .btn:active { transform: scale(0.98); }
    .btn-start {
      background: #062814;
      border: 1px solid #10b981;
      color: #10b981;
      box-shadow: 0 0 12px rgba(16, 185, 129, 0.2);
    }
    .btn-start:hover {
      background: #10b981;
      color: #000000;
    }
    .btn-stop {
      background: #28080a;
      border: 1px solid #ef4444;
      color: #ef4444;
      box-shadow: 0 0 12px rgba(239, 68, 68, 0.2);
    }
    .btn-stop:hover {
      background: #ef4444;
      color: #000000;
    }
    .track-status-box {
      background: #030303;
      border: 1px solid var(--border-color);
      border-radius: 8px;
      padding: 12px;
      text-align: center;
      margin-bottom: 12px;
    }
    .track-title { font-size: 0.72rem; text-transform: uppercase; color: var(--accent-cyan); font-weight: 700; letter-spacing: 0.5px; }
    .track-val { font-size: 1.25rem; font-weight: 800; color: #fff; margin-top: 4px; }
    .zone-pills {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 8px;
      margin-top: 8px;
    }
    .zone-pill {
      background: #000000;
      border: 1px solid #1f1f1f;
      border-radius: 6px;
      padding: 6px;
      font-size: 0.72rem;
      text-align: center;
      font-weight: 600;
      color: #777777;
    }
    .zone-pill.active {
      background: #062312;
      border-color: var(--accent-green);
      color: var(--accent-green);
    }
    .metrics-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 10px;
    }
    .metric-box {
      background: #020202;
      border: 1px solid var(--border-color);
      padding: 12px;
      border-radius: 8px;
    }
    .metric-label { font-size: 0.7rem; color: var(--text-muted); text-transform: uppercase; font-weight: 600; }
    .metric-val {
      font-size: 1.35rem;
      font-weight: 700;
      color: #fff;
      margin-top: 4px;
    }
    .metric-unit { font-size: 0.75rem; color: var(--text-muted); }
    .slider-item { margin-bottom: 10px; }
    .slider-header {
      display: flex;
      justify-content: space-between;
      font-size: 0.78rem;
      margin-bottom: 4px;
    }
    .slider-header span:last-child {
      color: var(--accent-cyan);
      font-weight: 700;
    }
    input[type=range] {
      width: 100%;
      height: 6px;
      background: #141414;
      border-radius: 4px;
      outline: none;
      -webkit-appearance: none;
      border: 1px solid #1f1f1f;
    }
    input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 16px;
      height: 16px;
      border-radius: 50%;
      background: var(--accent-cyan);
      cursor: pointer;
      box-shadow: 0 0 8px var(--accent-cyan);
    }
    select {
      width: 100%;
      background: #111111;
      border: 1px solid #2a2a2a;
      color: #ffffff;
      padding: 7px 10px;
      border-radius: 6px;
      font-size: 0.8rem;
      outline: none;
    }
    .btn-apply {
      width: 100%;
      padding: 11px;
      background: #071927;
      border: 1px solid var(--accent-cyan);
      color: var(--accent-cyan);
      border-radius: 8px;
      font-family: var(--font-mono);
      font-weight: 700;
      font-size: 0.85rem;
      cursor: pointer;
      margin-top: 12px;
      transition: all 0.2s ease;
    }
    .btn-apply:hover {
      background: var(--accent-cyan);
      color: #000000;
    }
    .log-box {
      background: #020202;
      border: 1px solid var(--border-color);
      border-radius: 8px;
      height: 180px;
      overflow-y: auto;
      padding: 10px;
      font-size: 0.74rem;
      line-height: 1.5;
      display: flex;
      flex-direction: column;
      gap: 4px;
    }
    .log-box::-webkit-scrollbar {
      width: 6px;
    }
    .log-box::-webkit-scrollbar-track {
      background: #000000;
    }
    .log-box::-webkit-scrollbar-thumb {
      background: #222222;
      border-radius: 3px;
    }
    .log-row {
      display: flex;
      gap: 6px;
      align-items: baseline;
      word-break: break-all;
    }
    .log-ts { color: #555555; font-size: 0.7rem; flex-shrink: 0; }
    .log-tag { font-weight: 700; font-size: 0.68rem; flex-shrink: 0; }
    .log-tag.info { color: var(--accent-cyan); }
    .log-tag.cmd { color: var(--accent-green); }
    .log-tag.warn { color: var(--accent-yellow); }
    .log-tag.err { color: var(--accent-red); }
    .log-tag.track { color: #c084fc; }
    .log-txt { color: #d1d5db; }
    .btn-clear-log {
      background: transparent;
      border: 1px solid #222222;
      color: var(--text-muted);
      padding: 2px 8px;
      border-radius: 4px;
      font-size: 0.7rem;
      cursor: pointer;
      font-family: var(--font-mono);
      transition: all 0.2s ease;
    }
    .btn-clear-log:hover {
      border-color: #444444;
      color: #ffffff;
      background: #111111;
    }
    .toast {
      position: fixed;
      bottom: 20px;
      right: 20px;
      background: #0a0a0a;
      border: 1px solid var(--accent-cyan);
      color: #fff;
      font-family: var(--font-mono);
      padding: 10px 18px;
      border-radius: 8px;
      font-size: 0.82rem;
      opacity: 0;
      transition: opacity 0.3s ease;
      pointer-events: none;
      z-index: 100;
      box-shadow: 0 0 20px rgba(0, 240, 255, 0.2);
    }
    .toast.show { opacity: 1; }
  </style>
</head>
<body>
  <div class="wrapper">
    <header>
      <div class="logo-area">
        <h1>🏎️ Carrinho Seguidor ESP32-CAM</h1>
        <div class="logo-sub">DHCP DINÂMICO // DUAL-CORE FREERTOS</div>
      </div>
      <div id="badgeStatus" class="status-badge">
        <div class="dot-pulse"></div>
        <span id="txtStatusBadge">RODANDO</span>
      </div>
    </header>

    <div class="main-grid">
      <div>
        <div class="card">
          <div class="card-head">
            <span>📷 Visão Computacional Embarcada (Porta 81)</span>
            <span style="font-size: 0.75rem; color: var(--text-muted);">Overlay: 3 Zonas + Curva</span>
          </div>
          <div class="stream-box">
            <img id="streamImg" alt="Streaming MJPEG ESP32-CAM">
          </div>
          <div class="btn-group">
            <button class="btn btn-start" onclick="enviarComando('start')">▶ INICIAR TRAÇÃO</button>
            <button class="btn btn-stop" onclick="enviarComando('stop')">🛑 PARAR EMERGÊNCIA</button>
          </div>
        </div>

        <div class="card">
          <div class="track-status-box">
            <div class="track-title">Classificação da Trajetória</div>
            <div class="track-val" id="lblTrackStatus">RETA</div>
            <div class="zone-pills">
              <div id="pillBase" class="zone-pill">🔴 Base (Perto)</div>
              <div id="pillMid" class="zone-pill">🔴 Meio</div>
              <div id="pillFar" class="zone-pill">🔴 Longe (Curva)</div>
            </div>
          </div>

          <div class="metrics-grid">
            <div class="metric-box">
              <div class="metric-label">Taxa de Quadros</div>
              <div class="metric-val" id="valFps">-- <span class="metric-unit">FPS</span></div>
            </div>
            <div class="metric-box">
              <div class="metric-label">Curvatura Antecipada (ΔE)</div>
              <div class="metric-val" id="valCurv">0.0 <span class="metric-unit">px</span></div>
            </div>
            <div class="metric-box">
              <div class="metric-label">Erro Composto (Blend)</div>
              <div class="metric-val" id="valErro">-- <span class="metric-unit">px</span></div>
            </div>
            <div class="metric-box">
              <div class="metric-label">Ângulo do Servo</div>
              <div class="metric-val" id="valAngulo">90.0<span class="metric-unit">°</span></div>
            </div>
            <div class="metric-box">
              <div class="metric-label">PWM Motores</div>
              <div class="metric-val" id="valPwm">0 <span class="metric-unit">/255</span></div>
            </div>
            <div class="metric-box">
              <div class="metric-label">Threshold Ativo</div>
              <div class="metric-val" id="valTh">-- <span class="metric-unit">val</span></div>
            </div>
          </div>
        </div>
      </div>

      <div>
        <div class="card">
          <div class="card-head">
            <span>⚙️ Calibração Rápida em Tempo Real</span>
            <span style="font-size: 0.72rem; color: var(--text-muted);" id="lblHostInfo">DHCP Auto</span>
          </div>

          <div class="slider-group">
            <div class="slider-item">
              <div class="slider-header">
                <span>Sensibilidade Proporcional (Kp)</span>
                <span id="lblKp">0.70</span>
              </div>
              <input type="range" id="rngKp" min="0.10" max="2.00" step="0.05" value="0.70" oninput="atualizarLabel('lblKp', this.value)">
            </div>

            <div class="slider-item">
              <div class="slider-header">
                <span>Amortecimento Derivativo (Kd)</span>
                <span id="lblKd">0.45</span>
              </div>
              <input type="range" id="rngKd" min="0.00" max="1.50" step="0.05" value="0.45" oninput="atualizarLabel('lblKd', this.value)">
            </div>

            <div class="slider-item">
              <div class="slider-header">
                <span>Velocidade Base Motores (PWM)</span>
                <span id="lblSpeed">180</span>
              </div>
              <input type="range" id="rngSpeed" min="60" max="255" step="5" value="180" oninput="atualizarLabel('lblSpeed', this.value)">
            </div>

            <div class="slider-item">
              <div class="slider-header">
                <span>Peso Antecipação Lookahead (W_far)</span>
                <span id="lblWfar">0.35</span>
              </div>
              <input type="range" id="rngWfar" min="0.00" max="0.60" step="0.05" value="0.35" oninput="atualizarLabel('lblWfar', this.value)">
            </div>

            <div class="slider-item">
              <div class="slider-header">
                <span>Threshold da Linha (0 = Auto Inteligente)</span>
                <span id="lblTh">0 (Auto)</span>
              </div>
              <input type="range" id="rngTh" min="0" max="220" step="1" value="0" oninput="atualizarLabel('lblTh', this.value == 0 ? '0 (Auto)' : this.value)">
            </div>

            <div class="slider-item" style="margin-top: 8px;">
              <div class="slider-header">
                <span>Orientação do Sensor da Câmera</span>
              </div>
              <select id="selOrient">
                <option value="0" selected>Normal (Landscape 160x120)</option>
                <option value="1">Rotacionado 90° (Vertical 120x160)</option>
              </select>
            </div>

            <div style="margin-top: 10px; display: flex; align-items: center; justify-content: space-between;">
              <span style="font-size: 0.8rem; color: var(--text-main);">Inverter Sentido do Servo:</span>
              <input type="checkbox" id="chkInvServo" style="width: 18px; height: 18px; accent-color: var(--accent-cyan); cursor: pointer;">
            </div>

            <button class="btn-apply" onclick="enviarCalibracao()">💾 APLICAR PARÂMETROS</button>
          </div>
        </div>

        <div class="card">
          <div class="card-head">
            <span>📟 Registro de Logs do Sistema</span>
            <button class="btn-clear-log" onclick="limparLogs()">🗑️ Limpar</button>
          </div>
          <div id="logContainer" class="log-box"></div>
        </div>
      </div>
    </div>
  </div>

  <div id="toast" class="toast">Comando executado</div>

  <script>
    const MAX_LOGS = 100;
    let lastTrackStatus = -1;
    let lastDetectada = null;
    let lastStatusTracao = null;
    let isOffline = false;

    function getTimestamp() {
      const now = new Date();
      return now.toTimeString().split(' ')[0];
    }

    function addLog(msg, tipo = 'info') {
      const container = document.getElementById('logContainer');
      if (!container) return;

      const row = document.createElement('div');
      row.className = 'log-row';

      const tagLabels = {
        info: '[INFO]',
        cmd: '[CMD]',
        warn: '[WARN]',
        err: '[ERRO]',
        track: '[PISTA]'
      };

      row.innerHTML = `<span class="log-ts">${getTimestamp()}</span><span class="log-tag ${tipo}">${tagLabels[tipo] || '[LOG]'}</span><span class="log-txt">${msg}</span>`;
      container.appendChild(row);

      while (container.children.length > MAX_LOGS) {
        container.removeChild(container.firstChild);
      }
      container.scrollTop = container.scrollHeight;
    }

    function limparLogs() {
      const container = document.getElementById('logContainer');
      if (container) {
        container.innerHTML = '';
        addLog('Logs limpos pelo operador.', 'info');
      }
    }

    window.addEventListener('DOMContentLoaded', () => {
      const host = window.location.hostname || 'carrinho.local';
      const lblHost = document.getElementById('lblHostInfo');
      if (lblHost) lblHost.innerText = 'IP: ' + host;

      document.getElementById('streamImg').src = 'http://' + host + ':81/stream';
      addLog('Painel inicializado. Conectado a http://' + host, 'info');
    });

    function showToast(msg) {
      const t = document.getElementById('toast');
      t.innerText = msg;
      t.className = 'toast show';
      setTimeout(() => { t.className = 'toast'; }, 2200);
    }

    function atualizarLabel(id, val) {
      document.getElementById(id).innerText = val;
    }

    async function enviarComando(action) {
      try {
        const res = await fetch('/cmd?action=' + action);
        if (res.ok) {
          if (action === 'start') {
            showToast('▶ Tração Iniciada');
            addLog('Comando enviado: Tração INICIADA', 'cmd');
          } else {
            showToast('🛑 Tração Parada');
            addLog('Comando enviado: PARADA DE EMERGÊNCIA acionada', 'warn');
          }
        }
      } catch (e) {
        showToast('Erro ao enviar comando');
        addLog('Falha ao enviar comando: ' + action, 'err');
      }
    }

    async function enviarCalibracao() {
      const kp = document.getElementById('rngKp').value;
      const kd = document.getElementById('rngKd').value;
      const th = document.getElementById('rngTh').value;
      const speed = document.getElementById('rngSpeed').value;
      const wfar = document.getElementById('rngWfar').value;
      const orient = document.getElementById('selOrient').value;
      const invServo = document.getElementById('chkInvServo').checked ? 1 : 0;

      try {
        const url = `/tune?kp=${kp}&kd=${kd}&th=${th}&speed=${speed}&wfar=${wfar}&orient=${orient}&inv_servo=${invServo}`;
        const res = await fetch(url);
        if (res.ok) {
          showToast('💾 Parâmetros salvos!');
          addLog(`Parâmetros salvos: Kp=${kp} | Kd=${kd} | PWM=${speed} | WFar=${wfar} | Th=${th} | Orient=${orient} | InvServo=${invServo}`, 'info');
        }
      } catch (e) {
        showToast('Erro ao calibrar');
        addLog('Falha ao enviar calibração', 'err');
      }
    }

    const TRACK_STR = [
      "SEM LINHA",
      "🏎️ RETA",
      "↩️ CURVA SUAVE (ESQ)",
      "↪️ CURVA SUAVE (DIR)",
      "⚡ CURVA FECHADA (ESQ)",
      "⚡ CURVA FECHADA (DIR)"
    ];

    async function pollingTelemetria() {
      try {
        const resp = await fetch('/data');
        if (resp.ok) {
          if (isOffline) {
            isOffline = false;
            addLog('Comunicação com o ESP32 restabelecida.', 'info');
          }

          const d = await resp.json();
          document.getElementById('valFps').innerHTML = d.fps.toFixed(1) + ' <span class="metric-unit">FPS</span>';
          document.getElementById('valErro').innerHTML = (d.erro >= 0 ? '+' : '') + d.erro.toFixed(1) + ' <span class="metric-unit">px</span>';
          document.getElementById('valCurv').innerHTML = (d.curvatura >= 0 ? '+' : '') + d.curvatura.toFixed(1) + ' <span class="metric-unit">px</span>';
          document.getElementById('valAngulo').innerHTML = d.angulo.toFixed(1) + '<span class="metric-unit">°</span>';
          document.getElementById('valPwm').innerHTML = d.pwm + ' <span class="metric-unit">/255</span>';
          document.getElementById('valTh').innerHTML = d.th + ' <span class="metric-unit">efetivo</span>';

          // Status de Pista
          const stIndex = d.status_pista || 0;
          document.getElementById('lblTrackStatus').innerText = TRACK_STR[stIndex] || "DESCONHECIDO";

          if (lastTrackStatus !== stIndex) {
            lastTrackStatus = stIndex;
            addLog(`Trajetória: ${TRACK_STR[stIndex]} (Erro: ${d.erro.toFixed(1)}px)`, 'track');
          }

          if (lastDetectada !== d.detectada) {
            lastDetectada = d.detectada;
            if (!d.detectada) {
              addLog('Linha não detectada! Robô em modo de busca.', 'warn');
            } else {
              addLog('Linha preta detectada e rastreada.', 'info');
            }
          }

          // Pílulas das 3 Zonas
          document.getElementById('pillBase').className = d.base_ok ? 'zone-pill active' : 'zone-pill';
          document.getElementById('pillBase').innerText = d.base_ok ? '🟢 Base (Perto)' : '🔴 Base (Perto)';

          document.getElementById('pillMid').className = d.mid_ok ? 'zone-pill active' : 'zone-pill';
          document.getElementById('pillMid').innerText = d.mid_ok ? '🟢 Meio' : '🔴 Meio';

          document.getElementById('pillFar').className = d.far_ok ? 'zone-pill active' : 'zone-pill';
          document.getElementById('pillFar').innerText = d.far_ok ? '🟢 Longe (Curva)' : '🔴 Longe (Curva)';

          // Badge Header
          const badge = document.getElementById('badgeStatus');
          const txtBadge = document.getElementById('txtStatusBadge');
          if (d.status === 'RODANDO') {
            if (stIndex >= 4) {
              badge.className = 'status-badge curve';
              txtBadge.innerText = 'FRENAGEM CURVA';
            } else {
              badge.className = 'status-badge';
              txtBadge.innerText = d.detectada ? 'RASTREANDO' : 'BUSCANDO LINHA';
            }
          } else {
            badge.className = 'status-badge stopped';
            txtBadge.innerText = 'PARADO (EMERGÊNCIA)';
          }

          if (lastStatusTracao !== d.status) {
            lastStatusTracao = d.status;
            addLog('Estado da Tração: ' + d.status, d.status === 'RODANDO' ? 'cmd' : 'warn');
          }
        }
      } catch (e) {
        if (!isOffline) {
          isOffline = true;
          addLog('Sem resposta do ESP32 (/data). Verifique o Wi-Fi.', 'err');
        }
      }
    }

    setInterval(pollingTelemetria, 120);
  </script>
</body>
</html>
)rawliteral";

#endif // INDEX_HTML_H
