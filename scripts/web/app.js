// DOM Lookups
const connectionDot = document.getElementById('connection-dot');
const connectionText = document.getElementById('connection-text');
const statStatus = document.getElementById('stat-status');
const statPoints = document.getElementById('stat-points');
const statTilt = document.getElementById('stat-tilt');

const portSelect = document.getElementById('port-select');
const btnStart = document.getElementById('btn-start');
const btnStop = document.getElementById('btn-stop');
const btnClear = document.getElementById('btn-clear');
const btnExport = document.getElementById('btn-export-csv');

const cfgRadius = document.getElementById('max-radius');
const cfgQuality = document.getElementById('min-quality');
const cfgTiltMin = document.getElementById('tilt-min');
const cfgTiltMax = document.getElementById('tilt-max');

// 3D Engine Global State
let ws = null;
let reconnectInterval = null;
let deviceState = { isScanning: false };
let latestTilt = 90.0;

// ============================================
// THREE.JS 3D ENGINE INITIALIZATION
// ============================================
const container = document.getElementById('threejs-container');
const scene = new THREE.Scene();
scene.fog = new THREE.FogExp2(0x0a0c10, 0.0001); // Subtle depth fog

// Renderer setup
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(container.clientWidth, container.clientHeight);
renderer.setClearColor(0x0a0c10, 0.5); // Deep dark background
container.appendChild(renderer.domElement);

// Camera Setup
const camera = new THREE.PerspectiveCamera(50, container.clientWidth / container.clientHeight, 10, 100000);
// Start looking isometric at the scanner
camera.position.set(2000, 2000, 3000);

// Orbit Controls (Mouse tracking)
const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.05;
controls.maxDistance = 25000;

// Grid & Axis Helpers
const gridHelper = new THREE.GridHelper(16000, 16, 0x1f2937, 0x111827);
scene.add(gridHelper);
// X=Red, Y=Green, Z=Blue
const axesHelper = new THREE.AxesHelper(1000);
scene.add(axesHelper);

// Scanner Local Orientation object (Little box in the middle)
const scannerGeo = new THREE.BoxGeometry(200, 100, 200);
const scannerMat = new THREE.MeshBasicMaterial({ color: 0xef4444, wireframe: true });
const scannerMesh = new THREE.Mesh(scannerGeo, scannerMat);
scene.add(scannerMesh);

// ============================================
// MASSIVE POINT CLOUD BUFFER (1 Million Points max)
// ============================================
const MAX_POINTS = 1000000;
let pointCount = 0;

// We use BufferGeometry assigned sequentially for insane performance
const geometry = new THREE.BufferGeometry();
const positions = new Float32Array(MAX_POINTS * 3); // x,y,z arrays
const colors = new Float32Array(MAX_POINTS * 3); // r,g,b arrays

geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
geometry.setDrawRange(0, 0);

// Point Material with Vertex Colors
const material = new THREE.PointsMaterial({
    size: 25,           // Point visual size
    vertexColors: true, // Use the colors array
    transparent: true,
    opacity: 0.8,
    sizeAttenuation: true
});
const pointCloud = new THREE.Points(geometry, material);
scene.add(pointCloud);

// Color Palette for Depth/Z-mapping
const colorScale = new THREE.Color();

// Window resizing handling
window.addEventListener('resize', () => {
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
});

// Render Loop
function animate() {
    requestAnimationFrame(animate);
    
    // Rotate scanner mesh based on latest tilt for visual feedback
    // pitch rotates around X axis usually
    scannerMesh.rotation.x = (latestTilt - 90) * (Math.PI / 180);
    
    controls.update(); // only required if controls.enableDamping = true
    renderer.render(scene, camera);
}
animate(); // Fire standard loop

// ============================================
// DATA INGESTION & MATHEMATICS
// ============================================
function addPointsToCloud(pointsArr) {
    if (pointCount >= MAX_POINTS) return; // Buffer full! Use clear
    
    // Max constraints
    const rLimit = parseFloat(cfgRadius.value);
    
    let added = 0;
    
    for (let i = 0; i < pointsArr.length; i++) {
        const [q, yawDeg, dist, pitchDeg] = pointsArr[i];
        
        if (dist > rLimit || dist < 10) continue; 
        if (pointCount >= MAX_POINTS) break;
        
        latestTilt = pitchDeg;
        
        // Math Transform (Spherical to Cartesian)
        // Adjusting logic: 
        // 90deg tilt = looking straight forward. Yaw = rotation around 0-360.
        // Therefore, if Pitch goes to 135deg, it's looking UP by 45deg.
        const yaw = (yawDeg) * (Math.PI / 180); 
        const pitch = (pitchDeg - 90) * (Math.PI / 180);
        
        // Raw polar assumes X-Z plane, Y is altitude
        // Let's sweep around the Y axis for LiDAR spin
        const localX = dist * Math.cos(yaw);
        const localZ = dist * Math.sin(yaw);
        
        // Apply Tilt mechanism Pitch (rotate around local X)
        const finalX = localX;
        const finalY = -localZ * Math.sin(pitch);
        const finalZ = localZ * Math.cos(pitch);
        
        const idx = pointCount * 3;
        positions[idx] = finalX;
        positions[idx+1] = finalY;
        positions[idx+2] = finalZ;
        
        // Color mapping by height (Y-axis) relative to radius mapping
        // Heatmap style: Blue=Low, Red=High
        // Normalize height from -3000 to +3000
        const hNorm = Math.max(0, Math.min(1, (finalY + 3000) / 6000));
        colorScale.setHSL((1.0 - hNorm) * 0.7, 1.0, 0.6); // HSL sweep from blue to red
        
        colors[idx] = colorScale.r;
        colors[idx+1] = colorScale.g;
        colors[idx+2] = colorScale.b;
        
        pointCount++;
        added++;
    }
    
    if (added > 0) {
        statPoints.textContent = `${pointCount.toLocaleString()} vertices`;
        statTilt.textContent = `${latestTilt.toFixed(1)}°`;
        
        // Tell graphic engine buffer has mutated
        geometry.attributes.position.needsUpdate = true;
        geometry.attributes.color.needsUpdate = true;
        geometry.setDrawRange(0, pointCount);
    }
}

// Reset 3D Scan Logic
function clearPointCloud() {
    pointCount = 0;
    geometry.setDrawRange(0, 0);
    statPoints.textContent = `0 vertices`;
}
btnClear.addEventListener('click', clearPointCloud);


// ============================================
// WEBSOCKET LOGIC 
// ============================================
function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;
    
    statStatus.textContent = 'Connecting...';
    ws = new WebSocket(wsUrl);
    
    ws.onopen = () => {
        connectionDot.classList.add('connected');
        connectionText.textContent = 'Connected to Scanner';
        statStatus.textContent = 'WebSocket connected.';
        clearInterval(reconnectInterval);
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
        if (!reconnectInterval) reconnectInterval = setInterval(connectWebSocket, 3000);
    };
    
    ws.onmessage = (event) => {
        const msg = JSON.parse(event.data);
        switch(msg.type) {
            case "ports":
                portSelect.innerHTML = '<option value="">No ports found</option>';
                if (msg.data.length > 0) {
                    portSelect.innerHTML = '';
                    msg.data.forEach(p => portSelect.appendChild(new Option(p, p)));
                    if (!deviceState.isScanning) btnStart.disabled = false;
                }
                break;
            case "status":
                deviceState.isScanning = msg.is_scanning;
                statStatus.textContent = msg.status_text;
                btnStart.disabled = msg.is_scanning;
                btnStop.disabled = !msg.is_scanning;
                portSelect.disabled = msg.is_scanning;
                break;
            case "scan_data":
                // Feed the point engine the massive arrays
                addPointsToCloud(msg.points);
                break;
            case "export_ready_3d":
                const blob = new Blob([msg.csv_data], { type: 'text/csv' });
                const a = document.createElement('a');
                a.href = window.URL.createObjectURL(blob);
                a.download = `3d_point_cloud_${Date.now()}.csv`;
                a.click();
                break;
        }
    };
}

// Events
btnStart.addEventListener('click', () => {
    ws.send(JSON.stringify({
        action: "start_scan",
        port: portSelect.value,
        config: {
            min_quality: parseInt(cfgQuality.value),
            max_radius: parseFloat(cfgRadius.value),
            tilt_min: parseFloat(cfgTiltMin.value),
            tilt_max: parseFloat(cfgTiltMax.value)
        }
    }));
});
btnStop.addEventListener('click', () => ws.send(JSON.stringify({ action: "stop_scan" })));

// Real-time config patching
[cfgRadius, cfgQuality, cfgTiltMin, cfgTiltMax].forEach(el => {
    el.addEventListener('change', () => {
        ws.send(JSON.stringify({
            action: "update_config",
            config: {
                max_radius: parseFloat(cfgRadius.value) || 8000,
                min_quality: parseInt(cfgQuality.value) || 0,
                tilt_min: parseFloat(cfgTiltMin.value) || 85,
                tilt_max: parseFloat(cfgTiltMax.value) || 95
            }
        }));
    });
});

// CSV trigger: instead of asking server, we can build it client side since we have Cartesian logic mapped 
// actually, calculating it server side is more exact if we want raw data. But if we want XYZ, client side is fastest!
btnExport.addEventListener('click', () => {
    if (pointCount === 0) return alert("Nothing to export!");
    let csv = "X,Y,Z,Color_R,Color_G,Color_B\n";
    for(let i=0; i<pointCount; i++){
        const idx = i*3;
        csv += `${positions[idx].toFixed(2)},${positions[idx+1].toFixed(2)},${positions[idx+2].toFixed(2)},${colors[idx].toFixed(2)},${colors[idx+1].toFixed(2)},${colors[idx+2].toFixed(2)}\n`;
    }
    const blob = new Blob([csv], { type: 'text/csv' });
    const a = document.createElement('a');
    a.href = window.URL.createObjectURL(blob);
    a.download = `3d_scan_${Date.now()}.csv`;
    a.click();
});

connectWebSocket();
