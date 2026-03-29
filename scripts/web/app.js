// DOM Elements
const connectionDot = document.getElementById('connection-dot');
const connectionText = document.getElementById('connection-text');
const portSelect = document.getElementById('port-select');
const maxRadiusInput = document.getElementById('max-radius');
const minQualityInput = document.getElementById('min-quality');
const btnStart = document.getElementById('btn-start');
const btnStop = document.getElementById('btn-stop');
const btnExport = document.getElementById('btn-export-csv');
const statCoverage = document.getElementById('stat-coverage');
const statQuality = document.getElementById('stat-quality');
const statStatus = document.getElementById('stat-status');
const canvas = document.getElementById('lidar-canvas');
const ctx = canvas.getContext('2d');

// State
let ws = null;
let currentPoints = [];
let maxRadius = parseFloat(maxRadiusInput.value);
let deviceState = { isScanning: false };
let reconnectInterval = null;

// Connect WebSocket
function connectWebSocket() {
    // Connect to WebSocket dynamically based on current host
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;
    
    statStatus.textContent = 'Connecting...';
    
    ws = new WebSocket(wsUrl);
    
    ws.onopen = () => {
        connectionDot.classList.add('connected');
        connectionText.textContent = 'Connected to Scanner';
        statStatus.textContent = 'WebSocket connected.';
        clearInterval(reconnectInterval);
        
        // Request parameters/ports on connect
        ws.send(JSON.stringify({ action: "get_ports" }));
        ws.send(JSON.stringify({ action: "get_status" }));
    };
    
    ws.onclose = () => {
        connectionDot.classList.remove('connected');
        connectionText.textContent = 'Disconnected';
        statStatus.textContent = 'Connection lost. Retrying...';
        btnStart.disabled = true;
        btnStop.disabled = true;
        portSelect.innerHTML = '<option>Disconnected</option>';
        
        if (!reconnectInterval) {
            reconnectInterval = setInterval(connectWebSocket, 3000);
        }
    };
    
    ws.onerror = (error) => {
        console.error("WebSocket Error:", error);
    };
    
    ws.onmessage = (event) => {
        const msg = JSON.parse(event.data);
        handleServerMessage(msg);
    };
}

// Handle incoming messages
function handleServerMessage(msg) {
    switch(msg.type) {
        case "ports":
            updatePortsList(msg.data);
            break;
            
        case "status":
            deviceState.isScanning = msg.is_scanning;
            statStatus.textContent = msg.status_text;
            
            if (msg.is_scanning) {
                btnStart.disabled = true;
                btnStop.disabled = false;
                portSelect.disabled = true;
            } else {
                btnStart.disabled = false;
                btnStop.disabled = true;
                portSelect.disabled = false;
            }
            break;
            
        case "scan_data":
            // Render new scan frame
            currentPoints = msg.points; // array of [quality, angle, distance]
            statCoverage.textContent = msg.coverage.toFixed(1) + '°';
            statQuality.textContent = msg.quality_str;
            break;
            
        case "export_ready":
            // Prompt download using Blob
            const blob = new Blob([msg.csv_data], { type: 'text/csv' });
            const url = window.URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = `lidar_scan_${Date.now()}.csv`;
            a.click();
            window.URL.revokeObjectURL(url);
            break;
    }
}

// UI Event Listeners
btnStart.addEventListener('click', () => {
    ws.send(JSON.stringify({
        action: "start_scan",
        port: portSelect.value,
        config: {
            min_quality: parseInt(minQualityInput.value),
            max_radius: parseFloat(maxRadiusInput.value)
        }
    }));
});

btnStop.addEventListener('click', () => {
    ws.send(JSON.stringify({ action: "stop_scan" }));
});

btnExport.addEventListener('click', () => {
    ws.send(JSON.stringify({ action: "export_csv" }));
});

// Update settings when user changes them locally, push to server if needed
maxRadiusInput.addEventListener('change', () => {
    maxRadius = parseFloat(maxRadiusInput.value) || 8000;
    ws.send(JSON.stringify({ action: "update_config", config: { max_radius: maxRadius } }));
});
minQualityInput.addEventListener('change', () => {
    ws.send(JSON.stringify({ action: "update_config", config: { min_quality: parseInt(minQualityInput.value) } }));
});


function updatePortsList(ports) {
    portSelect.innerHTML = '';
    if (ports.length === 0) {
        portSelect.innerHTML = '<option value="">No ports found</option>';
        btnStart.disabled = true;
    } else {
        ports.forEach(port => {
            const opt = document.createElement('option');
            opt.value = port;
            opt.textContent = port;
            portSelect.appendChild(opt);
        });
        if (!deviceState.isScanning) {
            btnStart.disabled = false;
            portSelect.disabled = false;
        }
    }
}

// ==== High Performance Canvas Render Loop ====
function resizeCanvas() {
    const parent = canvas.parentElement;
    canvas.width = parent.clientWidth * window.devicePixelRatio;
    canvas.height = parent.clientHeight * window.devicePixelRatio;
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas(); // Initial size

function drawLidar() {
    // Use an alpha-fade instead of clearing text to create a cool 'trail' effect
    ctx.fillStyle = 'rgba(15, 17, 26, 0.2)'; 
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    
    const centerX = canvas.width / 2;
    const centerY = canvas.height / 2;
    const maxScreenRadius = Math.min(centerX, centerY) * 0.9; 
    
    // Scale factor: screen pixels per mm
    const scale = maxScreenRadius / maxRadius;

    // Draw grid rings
    ctx.strokeStyle = 'rgba(59, 130, 246, 0.1)'; // Brand primary transparent
    ctx.lineWidth = 1 * window.devicePixelRatio;
    ctx.setLineDash([5, 5]);

    // Draw 4 concentric rings
    for (let i = 1; i <= 4; i++) {
        const ringRad = (maxRadius / 4) * i;
        const screenRad = ringRad * scale;
        
        ctx.beginPath();
        ctx.arc(centerX, centerY, screenRad, 0, 2 * Math.PI);
        ctx.stroke();
        
        // Add distance labels
        ctx.fillStyle = 'rgba(148, 163, 184, 0.5)';
        ctx.font = `${10 * window.devicePixelRatio}px Inter`;
        ctx.fillText(`${ringRad}mm`, centerX + screenRad + 5, centerY);
    }
    
    // Draw Axis Cross
    ctx.beginPath();
    ctx.moveTo(centerX, 0); ctx.lineTo(centerX, canvas.height);
    ctx.moveTo(0, centerY); ctx.lineTo(canvas.width, centerY);
    ctx.stroke();
    ctx.setLineDash([]); // Reset line dash

    // Render Point Cloud
    if (currentPoints.length > 0) {
        for (let i = 0; i < currentPoints.length; i++) {
            const [q, ang, dist] = currentPoints[i];
            
            // Skip points outside UI radius visually
            if (dist > maxRadius || dist <= 0) continue; 
            
            // LiDAR angle defaults to top usually (0 is top rotating clockwise, or right depending on unit)
            // Typically in rplidar, 0 is right turning counter-clockwise, or 0 is up turning clockwise.
            // We use standard math: Angle in radians
            // Subtracting 90deg to make 0 up, adding to turn clockwise.
            const radians = (ang - 90) * (Math.PI / 180); 
            
            const px = centerX + (dist * scale) * Math.cos(radians);
            const py = centerY + (dist * scale) * Math.sin(radians);
            
            // Vibrant glow dot color
            ctx.fillStyle = '#38bdf8'; 
            
            ctx.beginPath();
            ctx.arc(px, py, 2 * window.devicePixelRatio, 0, 2 * Math.PI);
            ctx.fill();
        }
    }

    // Draw LiDAR center representation
    ctx.fillStyle = '#ef4444';
    ctx.beginPath();
    ctx.arc(centerX, centerY, 4 * window.devicePixelRatio, 0, 2 * Math.PI);
    ctx.fill();

    requestAnimationFrame(drawLidar);
}

// Start Render Loop
drawLidar();

// Connect
connectWebSocket();
