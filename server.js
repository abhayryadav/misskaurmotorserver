const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const cors = require('cors');
const app = express();
const port = process.env.PORT || 8080;

// State variables
let movement = 0; // 0: stop, 1: forward, -1: backward, 2: left, 3: right
let toggle = false; // true: run servo, false: stop servo

// Middleware to parse JSON and enable CORS
app.use(cors({ origin: '*' }));
app.use(express.json());

// Create HTTP server
const server = http.createServer(app);

// Create WebSocket server, attached to the HTTP server
const wss = new WebSocket.Server({ server });

// Broadcast state to all WebSocket clients
function broadcastState() {
    const state = { movement, toggle };
    wss.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(JSON.stringify(state));
        }
    });
    console.log(`Broadcasted state: ${JSON.stringify(state)}`);
}

// WebSocket connection handler
wss.on('connection', ws => {
    console.log('ESP8266 connected via WebSocket');
    ws.send(JSON.stringify({ movement, toggle }));
    
    ws.on('message', (msg) => {
        console.log(`Received from ESP8266: ${msg}`);
    });
    
    ws.on('error', (err) => {
        console.error(`WebSocket error: ${err}`);
    });
    
    ws.on('close', () => {
        console.log('ESP8266 disconnected');
    });
});

// HTTP Endpoints
app.get('/set-forward', (req, res) => {
    movement = 1;
    console.log('Set movement to forward (1)');
    broadcastState();
    res.json({ status: 'success', movement, toggle });
});

app.get('/set-backward', (req, res) => {
    movement = -1;
    console.log('Set movement to backward (-1)');
    broadcastState();
    res.json({ status: 'success', movement, toggle });
});

app.get('/set-left', (req, res) => {
    movement = 2;
    console.log('Set movement to left (2)');
    broadcastState();
    res.json({ status: 'success', movement, toggle });
});

app.get('/set-right', (req, res) => {
    movement = 3;
    console.log('Set movement to right (3)');
    broadcastState();
    res.json({ status: 'success', movement, toggle });
});

app.get('/set-stop', (req, res) => {
    movement = 0;
    console.log('Set movement to stop (0)');
    broadcastState();
    res.json({ status: 'success', movement, toggle });
});

app.get('/toggle', (req, res) => {
    movement = 9;
    console.log(`Toggled to ${toggle ? 'ON' : 'OFF'}`);
    broadcastState();
    res.json({ status: 'success', movement, toggle });
});

// Health check endpoint
app.get('/health', (req, res) => {
    res.json({ status: 'ok' });
});

// Start server
server.listen(port, () => {
    console.log(`Server running on port ${port}`);
    console.log(`WebSocket server running on the same port`);
});