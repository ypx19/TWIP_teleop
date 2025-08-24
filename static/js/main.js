// TWIP 网页仿真器前端主逻辑
let socket = null;
let isRecording = false;
let isPreviewingRecord = false;
let recordedDatasets = [];
let currentDatasetIndex = -1;

window.onload = function() {
    // 连接 SocketIO
    socket = io();

    // 绑定按钮事件
    document.getElementById('btn-up').onclick = () => sendKey('up');
    document.getElementById('btn-down').onclick = () => sendKey('down');
    document.getElementById('btn-left').onclick = () => sendKey('left');
    document.getElementById('btn-right').onclick = () => sendKey('right');
    document.getElementById('btn-pause').onclick = togglePause;
    document.getElementById('btn-record').onclick = startRecord;
    document.getElementById('btn-stop').onclick = stopRecord;
    document.getElementById('sim-speed').oninput = setSimSpeed;
    document.getElementById('btn-play').onclick = playVideo;

    // 数据管理相关
    document.getElementById('btn-trim').onclick = trimData;
    document.getElementById('btn-export').onclick = exportData;
    document.getElementById('data-list').onchange = handleDataSelection;
    
    // 时间轴预览相关
    const timelineContainer = document.getElementById('timeline-container');
    timelineContainer.addEventListener('mousemove', handleTimelineHover);
    timelineContainer.addEventListener('mouseleave', resetTimelinePreview);
    timelineContainer.addEventListener('mousedown', startTimelineDrag);
    document.addEventListener('mousemove', handleTimelineDrag);
    document.addEventListener('mouseup', stopTimelineDrag);

    // 键盘遥控
    window.addEventListener('keydown', function(e) {
        if (e.key === 'ArrowUp') sendKey('up');
        if (e.key === 'ArrowDown') sendKey('down');
        if (e.key === 'ArrowLeft') sendKey('left');
        if (e.key === 'ArrowRight') sendKey('right');
        if (e.key === ' ') togglePause();
        if (e.key === 'r' || e.key === 'R') {
            if (!isRecording) startRecord(); else stopRecord();
        }
    });

    // 监听仿真状态
    socket.on('sim_state', function(data) {
        // 画面
        if (!isPreviewingRecord && data.image) {
            document.getElementById('sim-img').src = 'data:image/png;base64,' + data.image;
        }
        // 状态栏
        if (!isPreviewingRecord && data.state) updateStatus(data.state);
        // 记录数据（仅在记录中时）
        if (isRecording) {
            recordCurrentState(data.state);
        }
    });

    // Gamepad 支持：定时轮询左摇杆，变化时发送到后端
    let lastGamepad = {x: 0, y: 0};
    setInterval(() => {
        const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
        if (gamepads[0]) {
            const gp = gamepads[0];
            const x = gp.axes[0] || 0;
            const y = -gp.axes[1] || 0; // 取反：上为正
            if (Math.abs(x - lastGamepad.x) > 0.01 || Math.abs(y - lastGamepad.y) > 0.01) {
                socket.emit('gamepad_control', {x, y});
                lastGamepad = {x, y};
            }
        }
    }, 50);
};

function sendKey(key) {
    socket.emit('key_press', {key: key});
}

function togglePause() {
    const btn = document.getElementById('btn-pause');
    if (btn.textContent.includes('暂停')) {
        socket.emit('pause_sim');
        btn.textContent = '继续仿真';
    } else {
        socket.emit('resume_sim');
        btn.textContent = '暂停仿真';
    }
}

function setSimSpeed() {
    const val = parseInt(document.getElementById('sim-speed').value);
    // 右边为最快，slowdown=1，左边最慢，slowdown=20
    const slowdown = 21 - val;
    socket.emit('set_sim_speed', {slowdown: slowdown});
    document.getElementById('sim-speed-label').textContent = slowdown === 1 ? '最快' : `1/${slowdown}`;
}

// 数据记录与管理
let currentRecording = [];
let recordingStartTime = 0;

function startRecord() {
    socket.emit('start_recording');
    isRecording = true;
    currentRecording = [];
    recordingStartTime = Date.now();
    document.getElementById('btn-record').disabled = true;
    document.getElementById('btn-stop').disabled = false;
    document.getElementById('record-status').textContent = '记录中';
}

function stopRecord() {
    socket.emit('stop_recording');
    isRecording = false;
    isPreviewingRecord = true;
    document.getElementById('btn-record').disabled = false;
    document.getElementById('btn-stop').disabled = true;
    document.getElementById('record-status').textContent = '未记录';
    // 保存本次数据集
    if (currentRecording.length > 0) {
        const duration = (currentRecording.at(-1).timestamp - currentRecording[0].timestamp);
        const name = `记录${recordedDatasets.length + 1}`;
        recordedDatasets.push({
            name,
            data: currentRecording.slice(),
            duration,
        });
        updateDataList();
    }
}

function recordCurrentState(state) {
    if (!state) return;
    const t0 = currentRecording.length > 0 ? currentRecording[0].timestamp : state.timestamp;
    // 获取当前画面
    const img = document.getElementById('sim-img').src;
    currentRecording.push({
        ...state,
        timestamp: state.timestamp - t0,
        image: img
    });
}

function updateDataList() {
    const dataList = document.getElementById('data-list');
    dataList.innerHTML = '';
    recordedDatasets.forEach((ds, idx) => {
        const opt = document.createElement('option');
        opt.value = idx;
        opt.textContent = `${ds.name} (${ds.data.length}点, ${ds.duration.toFixed(2)}s)`;
        dataList.appendChild(opt);
    });
    const btnPlay = document.getElementById('btn-play');
    if (recordedDatasets.length > 0) {
        dataList.selectedIndex = 0;
        currentDatasetIndex = 0;
        updateTrimRange();
        btnPlay.disabled = false;
    } else {
        currentDatasetIndex = -1;
        btnPlay.disabled = true;
    }
}

function handleDataSelection() {
    const dataList = document.getElementById('data-list');
    currentDatasetIndex = parseInt(dataList.value);
    updateTrimRange();
    document.getElementById('btn-play').disabled = (currentDatasetIndex < 0);
    // 新增：根据选中记录调整时间轴长度
    updateTimelineLength();
}

let trimStart = 0;
let trimEnd = 1;
let isDraggingTrimStart = false;
let isDraggingTrimEnd = false;

function updateTrimPointers() {
    const timelineContainer = document.getElementById('timeline-container');
    const startPointer = document.getElementById('trim-start-pointer');
    const endPointer = document.getElementById('trim-end-pointer');
    if (currentDatasetIndex < 0) return;
    const ds = recordedDatasets[currentDatasetIndex];
    if (!ds.data || ds.data.length === 0) return;
    const rect = timelineContainer.getBoundingClientRect();
    const startX = trimStart * rect.width;
    const endX = trimEnd * rect.width;
    startPointer.style.left = `${startX}px`;
    endPointer.style.left = `${endX - endPointer.offsetWidth}px`;
}

document.getElementById('trim-start-pointer').addEventListener('mousedown', function(e) {
    isDraggingTrimStart = true;
    e.stopPropagation();
});
document.getElementById('trim-end-pointer').addEventListener('mousedown', function(e) {
    isDraggingTrimEnd = true;
    e.stopPropagation();
});
document.addEventListener('mousemove', function(e) {
    if (!isDraggingTrimStart && !isDraggingTrimEnd) return;
    const timelineContainer = document.getElementById('timeline-container');
    const rect = timelineContainer.getBoundingClientRect();
    let x = Math.max(0, Math.min(e.clientX - rect.left, rect.width));
    let percent = x / rect.width;
    if (isDraggingTrimStart) {
        trimStart = Math.max(0, Math.min(percent, trimEnd - 0.01));
    }
    if (isDraggingTrimEnd) {
        trimEnd = Math.min(1, Math.max(percent, trimStart + 0.01));
    }
    updateTrimPointers();
});
document.addEventListener('mouseup', function(e) {
    isDraggingTrimStart = false;
    isDraggingTrimEnd = false;
});

function updateTimelineLength() {
    const timelineContainer = document.getElementById('timeline-container');
    if (currentDatasetIndex < 0) return;
    const ds = recordedDatasets[currentDatasetIndex];
    if (!ds.data || ds.data.length === 0) return;
    timelineContainer.setAttribute('data-duration', ds.duration);
    document.getElementById('timeline-time').textContent = `0.00s / ${ds.duration.toFixed(2)}s`;
    trimStart = 0;
    trimEnd = 1;
    updateTrimPointers();
}

function trimData() {
    if (currentDatasetIndex < 0) return;
    const ds = recordedDatasets[currentDatasetIndex];
    if (!ds.data || ds.data.length === 0) return;
    const startTime = trimStart * ds.duration;
    const endTime = trimEnd * ds.duration;
    const trimmed = ds.data.filter(d => d.timestamp >= startTime && d.timestamp <= endTime);
    // 可根据需求将trimmed数据集保存或导出
    alert(`已裁剪区间: ${startTime.toFixed(2)}s ~ ${endTime.toFixed(2)}s, 共${trimmed.length}帧`);
}

function exportData() {
    if (currentDatasetIndex < 0) return;
    const ds = recordedDatasets[currentDatasetIndex];
    // 字段选择
    const fields = [];
    if (document.getElementById('export-pos').checked) fields.push('position');
    if (document.getElementById('export-vel').checked) fields.push('velocity');
    if (document.getElementById('export-angle').checked) fields.push('tilt_angle');
    if (document.getElementById('export-motor').checked) fields.push('left_motor', 'right_motor');
    if (document.getElementById('export-wheel').checked) fields.push('left_wheel_angle', 'right_wheel_angle', 'left_wheel_velocity', 'right_wheel_velocity');
    // 导出数据
    const exportArr = ds.data.map(item => {
        const obj = { timestamp: item.timestamp };
        fields.forEach(f => { if (f in item) obj[f] = item[f]; });
        return obj;
    });
    const blob = new Blob([JSON.stringify(exportArr, null, 2)], {type: 'application/json'});
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `${ds.name.replace(/\s+/g, '_')}.json`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
}

// 仿真视频回放
let isPlaying = false;
function playVideo() {
    if (currentDatasetIndex < 0 || isPlaying) return;
    const ds = recordedDatasets[currentDatasetIndex];
    if (!ds.data || ds.data.length === 0) return;
    isPlaying = true;
    // 禁用交互
    document.getElementById('btn-play').disabled = true;
    document.getElementById('btn-record').disabled = true;
    document.getElementById('btn-stop').disabled = true;
    let idx = 0;
    function showFrame() {
        if (!isPlaying) return;
        const frame = ds.data[idx];
        if (frame && frame.image) {
            document.getElementById('sim-img').src = frame.image;
        }
        idx++;
        if (idx < ds.data.length) {
            setTimeout(showFrame, 50); // 20fps
        } else {
            isPlaying = false;
            document.getElementById('btn-play').disabled = false;
            document.getElementById('btn-record').disabled = false;
            document.getElementById('btn-stop').disabled = !isRecording;
        }
    }
    showFrame();
}

function updateStatus(state) {
    document.getElementById('stick').textContent = `(${state.left_stick_y?.toFixed(2) ?? 0}, ${state.left_stick_x?.toFixed(2) ?? 0})`;
    document.getElementById('motor').textContent = `L=${state.left_motor?.toFixed(2) ?? 0}, R=${state.right_motor?.toFixed(2) ?? 0}`;
    document.getElementById('angle').textContent = `L=${state.left_wheel_angle?.toFixed(2) ?? 0}, R=${state.right_wheel_angle?.toFixed(2) ?? 0}`;
    document.getElementById('vel').textContent = `L=${state.left_wheel_velocity?.toFixed(2) ?? 0}, R=${state.right_wheel_velocity?.toFixed(2) ?? 0}`;
    document.getElementById('speed').textContent = `1/${state.sim_slowdown?.toFixed(1) ?? 0}`;
    document.getElementById('pos').textContent = `x=${state.position?.toFixed(2) ?? 0}`;
    document.getElementById('v').textContent = `v=${state.velocity?.toFixed(2) ?? 0}`;
    document.getElementById('theta').textContent = `θ=${state.tilt_angle?.toFixed(2) ?? 0}`;
}
let isTimelineDragging = false;
function startTimelineDrag(event) {
    isTimelineDragging = true;
    handleTimelineHover(event);
}
function handleTimelineDrag(event) {
    if (!isTimelineDragging) return;
    const timelineContainer = document.getElementById('timeline-container');
    const rect = timelineContainer.getBoundingClientRect();
    if (event.clientX < rect.left || event.clientX > rect.right) return;
    handleTimelineHover(event);
}
function handleTimelineHover(event) {
    if (currentDatasetIndex < 0) return;
    const ds = recordedDatasets[currentDatasetIndex];
    if (!ds.data || ds.data.length === 0) return;
    isPreviewingRecord = true;
    const container = document.getElementById('timeline-container');
    const cursor = document.getElementById('timeline-cursor');
    const rect = container.getBoundingClientRect();
    const x = Math.max(0, Math.min(event.clientX - rect.left, rect.width));
    const percent = x / rect.width;
    const time = percent * ds.duration;
    cursor.style.left = `${x}px`;
    document.getElementById('timeline-time').textContent = `${time.toFixed(2)}s / ${ds.duration.toFixed(2)}s`;
    // 找到最接近该时间点的帧
    let frameIdx = 0;
    for (let i = 0; i < ds.data.length; i++) {
        if (ds.data[i].timestamp >= time) {
            frameIdx = i;
            break;
        }
    }
    const frame = ds.data[frameIdx];
    if (frame && frame.image) {
        document.getElementById('sim-img').src = frame.image;
        updateStatus(frame);
    }
}
function resetTimelinePreview() {
    isPreviewingRecord = false;
    // 可根据需要重置主显示区或保持最后预览帧
}
function stopTimelineDrag(event) {
    if (isTimelineDragging) {
        isTimelineDragging = false;
        handleTimelineHover(event);
    }
}