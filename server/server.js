const express = require('express');
const path = require('path');
const http = require('http');

const app = express();
const PORT = process.env.PORT || 3000;

// Middleware para arquivos estáticos
app.use(express.static(path.join(__dirname, 'public')));
app.use(express.json());

// Endpoint de status do servidor
app.get('/api/server-status', (req, res) => {
  res.json({ status: 'online', timestamp: new Date().toISOString() });
});

// Proxy HTTP simplificado para repassar requisições para o ESP32-CAM (opcional)
app.get('/esp32-proxy/*', (req, res) => {
  const espIp = req.query.espIp || '192.168.4.1';
  const targetPath = req.params[0];
  const url = `http://${espIp}/${targetPath}?${new URLSearchParams(req.query).toString()}`;

  http.get(url, (espRes) => {
    res.writeHead(espRes.statusCode, espRes.headers);
    espRes.pipe(res);
  }).on('error', (err) => {
    res.status(502).json({ error: 'Não foi possível conectar ao ESP32-CAM', details: err.message });
  });
});

app.listen(PORT, () => {
  console.log(`==================================================`);
  console.log(`🚀 SERVIDOR DO CARRINHO ESP32-CAM INICIADO!`);
  console.log(`📡 Acesse a Interface Web no Navegador: http://localhost:${PORT}`);
  console.log(`==================================================`);
});
