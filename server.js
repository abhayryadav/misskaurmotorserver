const express = require('express');
const WebSocket = require('ws');
const app = express();
const port = 8080;
const wsPort = 8081;

// State variables
let movement = 1; // 1: forward, -1: backward, 0: stop
let toggle = false; // true: run servo, false: stop servo

// Middleware to parse JSON
app.use(express.json());

// Create WebSocket server
const wss = new WebSocket.Server({ port: wsPort });

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
    // Send current state to newly connected client
    ws.send(JSON.stringify({ movement, toggle }));
    
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

app.get('/set-stop', (req, res) => {
    movement = 0;
    console.log('Set movement to stop (0)');
    broadcastState();
    res.json({ status: 'success', movement, toggle });
});

app.get('/toggle', (req, res) => {
    toggle = !toggle;
    console.log(`Toggled to ${toggle ? 'ON' : 'OFF'}`);
    broadcastState();
    res.json({ status: 'success', movement, toggle });
});

// app.get('/state', (req, res) => {
//     console.log('State requested');
//     res.json({ movement, toggle });
// });

// Start HTTP server
app.listen(port, () => {
    console.log(`HTTP server running on http://localhost:${port}`);
    console.log(`WebSocket server running on ws://localhost:${wsPort}`);
});