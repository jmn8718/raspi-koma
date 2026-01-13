// Configuration
const CONFIG = {
    rosbridgeUrl: 'ws://localhost:9090',
    cameraTopic: '/camera/image_raw',
    videoServerPort: 8080
};

// State
let state = {
    connected: false,
    motor: { speed: 0, angular: 0 },
    imu: { roll: 0, pitch: 0, yaw: 0 },
    sonar: { left: 0, right: 0 }
};

// DOM Elements
const els = {
    status: document.getElementById('connection-status'),
    stream: document.getElementById('camera-stream'),
    speed: document.getElementById('motor-speed'),
    arrowUp: document.getElementById('arrow-up'),
    arrowDown: document.getElementById('arrow-down'),
    arrowLeft: document.getElementById('arrow-left'),
    arrowRight: document.getElementById('arrow-right'),
    sonarLeftBar: document.getElementById('sonar-left-bar'),
    sonarRightBar: document.getElementById('sonar-right-bar'),
    sonarLeftVal: document.getElementById('sonar-left-val'),
    sonarRightVal: document.getElementById('sonar-right-val'),
    imuRoll: document.getElementById('imu-roll'),
    imuPitch: document.getElementById('imu-pitch'),
    imuYaw: document.getElementById('imu-yaw'),
    accelX: document.getElementById('accel-x'),
    accelY: document.getElementById('accel-y'),
    accelZ: document.getElementById('accel-z'),
    gyroX: document.getElementById('gyro-x'),
    gyroY: document.getElementById('gyro-y'),
    gyroZ: document.getElementById('gyro-z'),
};

// Initialize ROS
const ros = new ROSLIB.Ros({
    url: CONFIG.rosbridgeUrl
});

// ROS Connection Handlers
ros.on('connection', () => {
    console.log('Connected to websocket server.');
    state.connected = true;
    updateStatusUI();
    initCamera();
});

ros.on('error', (error) => {
    console.log('Error connecting to websocket server: ', error);
    state.connected = false;
    updateStatusUI();
});

ros.on('close', () => {
    console.log('Connection to websocket server closed.');
    state.connected = false;
    updateStatusUI();
});

// Subscribers

// 1. Motor Status (cmd_vel)
const cmdVelListener = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/Twist'
});

cmdVelListener.subscribe((message) => {
    const linear = message.linear.x;
    const angular = message.angular.z;

    // Update State
    state.motor.speed = linear.toFixed(2);

    // Determine direction for UI
    resetArrows();
    if (linear > 0.1) els.arrowUp.classList.add('active');
    else if (linear < -0.1) els.arrowDown.classList.add('active');

    if (angular > 0.1) els.arrowLeft.classList.add('active');
    else if (angular < -0.1) els.arrowRight.classList.add('active');

    // Update Speed Display
    els.speed.innerText = Math.abs(state.motor.speed);
});

// 2. IMU Data
const imuListener = new ROSLIB.Topic({
    ros: ros,
    name: '/imu/data_raw',
    messageType: 'sensor_msgs/Imu'
});

imuListener.subscribe((message) => {
    // Convert Quaternion to Euler (Simplified for visualization)
    const q = message.orientation;
    const roll = Math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    const pitch = Math.asin(2.0 * (q.w * q.y - q.z * q.x));
    const yaw = Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    // Convert to degrees
    const toDeg = (rad) => (rad * 180 / Math.PI).toFixed(0);

    els.imuRoll.innerText = toDeg(roll) + '°';
    els.imuPitch.innerText = toDeg(pitch) + '°';
    els.imuYaw.innerText = toDeg(yaw) + '°';

    // Raw Data
    if (message.linear_acceleration) {
        els.accelX.innerText = message.linear_acceleration.x.toFixed(2);
        els.accelY.innerText = message.linear_acceleration.y.toFixed(2);
        els.accelZ.innerText = message.linear_acceleration.z.toFixed(2);
    }
    if (message.angular_velocity) {
        els.gyroX.innerText = message.angular_velocity.x.toFixed(2);
        els.gyroY.innerText = message.angular_velocity.y.toFixed(2);
        els.gyroZ.innerText = message.angular_velocity.z.toFixed(2);
    }
});

// 3. Sonar Sensors
const sonarLeftListener = new ROSLIB.Topic({
    ros: ros,
    name: '/ultrasonic/front_left',
    messageType: 'sensor_msgs/Range'
});

sonarLeftListener.subscribe((msg) => {
    updateSonarBar(els.sonarLeftBar, els.sonarLeftVal, msg.range, msg.max_range);
});

const sonarRightListener = new ROSLIB.Topic({
    ros: ros,
    name: '/ultrasonic/front_right',
    messageType: 'sensor_msgs/Range'
});

sonarRightListener.subscribe((msg) => {
    updateSonarBar(els.sonarRightBar, els.sonarRightVal, msg.range, msg.max_range);
});

// Helper Functions
function updateStatusUI() {
    if (state.connected) {
        els.status.innerHTML = '<span class="indicator"></span> Connected';
        els.status.classList.remove('disconnected');
        els.status.classList.add('connected');
    } else {
        els.status.innerHTML = '<span class="indicator"></span> Disconnected';
        els.status.classList.remove('connected');
        els.status.classList.add('disconnected');
    }
}

function initCamera() {
    // Determine stream URL for web_video_server
    // MJPEG stream
    const streamUrl = `http://${window.location.hostname}:${CONFIG.videoServerPort}/stream?topic=${CONFIG.cameraTopic}&type=mjpeg&quality=80`;
    els.stream.src = streamUrl;
}

function resetArrows() {
    els.arrowUp.classList.remove('active');
    els.arrowDown.classList.remove('active');
    els.arrowLeft.classList.remove('active');
    els.arrowRight.classList.remove('active');
}

function updateSonarBar(barEl, textEl, val, max) {
    // Clamp value
    const range = Math.min(Math.max(val, 0), max);
    const percentage = (range / max) * 100;

    // Update UI
    barEl.style.width = `${percentage}%`;
    textEl.innerText = `${range.toFixed(2)} m`;

    // Color coding based on distance
    if (percentage < 20) {
        barEl.style.backgroundColor = 'var(--danger)';
    } else {
        barEl.style.backgroundColor = ''; // Reset to gradient
    }
}
