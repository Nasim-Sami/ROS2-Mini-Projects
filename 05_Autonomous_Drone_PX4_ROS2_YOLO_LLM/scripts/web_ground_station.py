#!/usr/bin/env python3
"""
Autonomous Drone Web Ground Control Station (GCS)
--------------------------------------------------
Provides an interactive Browser Ground Control Station:
- Dual Input: [Type] vs [Speak] with real-time Speech Recognition (EN & BN)
- Unmistakable visual and audio recording state feedback (pulsing indicator, soundwave animation, chimes)
- Editable transcription before execution
- Direct integration with LLM Intent Parser, Physics Safety Gatekeeper, and PX4 Flight Node
- Live telemetry streaming (Altitude, Mode, Safety Status, Progress)
"""

import sys
import os
import json
import time
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse

from drone_intent_parser import parse_user_command
from drone_safety_validator import DronePhysicalState, validate_and_resolve_mission
from trajectory_generator import generate_timed_trajectory
import subprocess

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

CAPTURES_DIR = "/workspace/captures"
os.makedirs(CAPTURES_DIR, exist_ok=True)

# Global Shared Telemetry State
telemetry_state = {
    "altitude_m": 0.0,
    "target_altitude_m": 2.5,
    "flight_mode": "Offboard",
    "is_airborne": False,
    "is_armed": False,
    "mission_status": "Ready (Grounded)",
    "last_goal": "Awaiting natural command...",
    "logs": ["Ground Control Station initialized."],
    "safety_status": "NORMAL",
    "obstacle_ahead": False,
    "obstacle_type": None,
    "obstacle_dist": None,
    "latest_capture": None
}

ros2_node_instance = None


HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Autonomous Drone AI Ground Control Station</title>
<style>
  :root {
    --bg: #070b13;
    --card-bg: rgba(14, 23, 42, 0.85);
    --border: rgba(30, 41, 59, 0.8);
    --cyan: #00f0ff;
    --emerald: #10b981;
    --rose: #f43f5e;
    --amber: #f59e0b;
    --text: #f8fafc;
    --text-dim: #94a3b8;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Segoe UI', system-ui, sans-serif; }
  body { background: var(--bg); color: var(--text); min-height: 100vh; padding: 24px; display: flex; flex-direction: column; align-items: center; }
  .container { width: 100%; max-width: 1050px; display: flex; flex-direction: column; gap: 20px; }
  
  /* Header */
  .header { display: flex; justify-content: space-between; align-items: center; background: var(--card-bg); border: 1px solid var(--border); padding: 18px 24px; border-radius: 16px; backdrop-filter: blur(12px); box-shadow: 0 8px 32px rgba(0,0,0,0.4); }
  .logo-box { display: flex; align-items: center; gap: 14px; }
  .logo-icon { width: 44px; height: 44px; background: linear-gradient(135deg, var(--cyan), #3b82f6); border-radius: 12px; display: flex; align-items: center; justify-content: center; font-size: 24px; box-shadow: 0 0 20px rgba(0,240,255,0.4); }
  .title-box h1 { font-size: 20px; font-weight: 700; letter-spacing: -0.5px; }
  .title-box p { font-size: 13px; color: var(--text-dim); }
  .status-badge { display: flex; align-items: center; gap: 8px; background: rgba(16, 185, 129, 0.15); border: 1px solid var(--emerald); color: var(--emerald); padding: 6px 14px; border-radius: 20px; font-size: 13px; font-weight: 600; }
  .pulse-dot { width: 8px; height: 8px; background: var(--emerald); border-radius: 50%; box-shadow: 0 0 10px var(--emerald); animation: pulse 2s infinite; }
  @keyframes pulse { 0%, 100% { opacity: 1; transform: scale(1); } 50% { opacity: 0.4; transform: scale(0.8); } }

  /* Telemetry Bar */
  .telemetry-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 14px; }
  .telem-card { background: var(--card-bg); border: 1px solid var(--border); border-radius: 14px; padding: 16px 20px; display: flex; flex-direction: column; gap: 6px; }
  .telem-label { font-size: 12px; color: var(--text-dim); text-transform: uppercase; letter-spacing: 0.8px; font-weight: 600; }
  .telem-val { font-size: 26px; font-weight: 800; color: var(--cyan); letter-spacing: -0.5px; }
  .telem-sub { font-size: 12px; color: var(--emerald); }

  /* Command Console */
  .command-panel { background: var(--card-bg); border: 1px solid var(--border); border-radius: 20px; padding: 26px; display: flex; flex-direction: column; gap: 20px; box-shadow: 0 12px 40px rgba(0,0,0,0.5); }
  
  /* Mode Selector Tabs */
  .tab-row { display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid var(--border); padding-bottom: 16px; }
  .tabs { display: flex; gap: 10px; background: rgba(15, 23, 42, 0.8); padding: 4px; border-radius: 12px; border: 1px solid var(--border); }
  .tab-btn { background: transparent; border: none; color: var(--text-dim); padding: 8px 20px; font-size: 14px; font-weight: 600; border-radius: 8px; cursor: pointer; transition: all 0.2s; display: flex; align-items: center; gap: 8px; }
  .tab-btn.active { background: var(--cyan); color: #000; box-shadow: 0 0 16px rgba(0,240,255,0.4); }
  .lang-select { background: rgba(15, 23, 42, 0.8); border: 1px solid var(--border); color: var(--text); padding: 6px 12px; border-radius: 8px; font-size: 13px; font-weight: 500; cursor: pointer; }

  /* Voice Recording Section */
  .voice-zone { display: none; flex-direction: column; align-items: center; gap: 16px; padding: 20px; background: rgba(15, 23, 42, 0.5); border-radius: 14px; border: 1px dashed var(--border); }
  .voice-zone.active { display: flex; }
  .mic-button { width: 72px; height: 72px; border-radius: 50%; background: linear-gradient(135deg, #1e293b, #0f172a); border: 2px solid var(--cyan); color: var(--cyan); font-size: 30px; display: flex; align-items: center; justify-content: center; cursor: pointer; transition: all 0.3s; position: relative; }
  .mic-button:hover { transform: scale(1.05); box-shadow: 0 0 25px rgba(0,240,255,0.4); }
  .mic-button.recording { background: linear-gradient(135deg, #e11d48, var(--rose)); border-color: #fff; color: #fff; box-shadow: 0 0 35px var(--rose); animation: record-pulse 1.2s infinite; }
  @keyframes record-pulse { 0% { box-shadow: 0 0 0 0 rgba(244, 63, 94, 0.7); } 70% { box-shadow: 0 0 0 22px rgba(244, 63, 94, 0); } 100% { box-shadow: 0 0 0 0 rgba(244, 63, 94, 0); } }
  .record-status-text { font-size: 15px; font-weight: 600; color: var(--text-dim); text-align: center; }
  .record-status-text.recording { color: var(--rose); font-weight: 700; }
  .soundwave { display: flex; align-items: center; gap: 4px; height: 24px; }
  .wave-bar { width: 4px; background: var(--rose); border-radius: 2px; animation: wave 1s ease-in-out infinite; }
  .wave-bar:nth-child(2) { animation-delay: 0.2s; }
  .wave-bar:nth-child(3) { animation-delay: 0.4s; }
  .wave-bar:nth-child(4) { animation-delay: 0.1s; }
  .wave-bar:nth-child(5) { animation-delay: 0.3s; }
  @keyframes wave { 0%, 100% { height: 6px; } 50% { height: 24px; } }

  /* Command Text Box */
  .input-label-row { display: flex; justify-content: space-between; align-items: center; }
  .input-label { font-size: 13px; font-weight: 600; color: var(--text-dim); }
  .edit-notice { font-size: 12px; color: var(--cyan); background: rgba(0,240,255,0.1); padding: 3px 10px; border-radius: 12px; border: 1px solid rgba(0,240,255,0.3); }
  .command-textarea { width: 100%; min-height: 90px; background: rgba(10, 15, 29, 0.9); border: 1px solid var(--border); border-radius: 12px; padding: 14px 18px; color: #fff; font-size: 16px; resize: vertical; outline: none; transition: all 0.2s; }
  .command-textarea:focus { border-color: var(--cyan); box-shadow: 0 0 18px rgba(0,240,255,0.25); }

  /* Buttons */
  .action-row { display: flex; gap: 14px; }
  .btn-submit { flex: 1; background: linear-gradient(135deg, var(--cyan), #2563eb); border: none; color: #000; font-size: 15px; font-weight: 700; padding: 14px 24px; border-radius: 12px; cursor: pointer; transition: all 0.2s; display: flex; align-items: center; justify-content: center; gap: 10px; box-shadow: 0 4px 20px rgba(0,240,255,0.3); }
  .btn-submit:hover { opacity: 0.95; transform: translateY(-1px); box-shadow: 0 6px 26px rgba(0,240,255,0.5); }
  .btn-land { background: rgba(244, 63, 94, 0.15); border: 1px solid var(--rose); color: var(--rose); font-size: 15px; font-weight: 700; padding: 14px 26px; border-radius: 12px; cursor: pointer; transition: all 0.2s; }
  .btn-land:hover { background: var(--rose); color: #fff; box-shadow: 0 0 20px rgba(244, 63, 94, 0.4); }

  /* Quick Prompts */
  .chip-row { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; }
  .chip-label { font-size: 12px; color: var(--text-dim); margin-right: 4px; }
  .chip { background: rgba(30, 41, 59, 0.6); border: 1px solid var(--border); color: #cbd5e1; font-size: 12px; padding: 6px 12px; border-radius: 20px; cursor: pointer; transition: all 0.2s; }
  .chip:hover { border-color: var(--cyan); color: var(--cyan); background: rgba(0,240,255,0.08); }

  /* Pipeline Log View */
  .log-panel { background: var(--card-bg); border: 1px solid var(--border); border-radius: 16px; padding: 20px; display: flex; flex-direction: column; gap: 12px; }
  .log-title { font-size: 14px; font-weight: 700; color: var(--text-dim); text-transform: uppercase; letter-spacing: 0.8px; display: flex; justify-content: space-between; }
  .log-box { background: rgba(8, 12, 22, 0.95); border: 1px solid var(--border); border-radius: 10px; padding: 14px; min-height: 110px; max-height: 200px; overflow-y: auto; font-family: monospace; font-size: 13px; line-height: 1.6; color: #a5f3fc; }
  .log-entry { margin-bottom: 4px; }
</style>
</head>
<body>

<div class="container">
  <!-- Header -->
  <div class="header">
    <div class="logo-box">
      <div class="logo-icon">🚁</div>
      <div class="title-box">
        <h1>Autonomous Drone Ground Control Station</h1>
        <p>Natural Language Voice & Text Interface (Qwen 2.5 1.5B + Safety Gatekeeper)</p>
      </div>
    </div>
    <div class="status-badge">
      <div class="pulse-dot"></div>
      <span id="system-status">SITL Flight Bridge Connected</span>
    </div>
  </div>

  <!-- Telemetry Cards -->
  <div class="telemetry-grid">
    <div class="telem-card">
      <span class="telem-label">Live Altitude (Z)</span>
      <span class="telem-val" id="val-altitude">0.00 m</span>
      <span class="telem-sub">Safe Ceiling: 150.0 m</span>
    </div>
    <div class="telem-card">
      <span class="telem-label">Flight State</span>
      <span class="telem-val" id="val-state" style="font-size:20px; color:#10b981;">GROUNDED</span>
      <span class="telem-sub" id="val-mode">Mode: Offboard</span>
    </div>
    <div class="telem-card">
      <span class="telem-label">Safety Gatekeeper</span>
      <span class="telem-val" id="val-safety" style="font-size:18px; color:#10b981;">ARMED & READY</span>
      <span class="telem-sub">Geofence: 300.0 m Radius</span>
    </div>
    <div class="telem-card">
      <span class="telem-label">YOLOv8 Vision Radar</span>
      <span class="telem-val" id="val-vision" style="font-size:18px; color:#10b981;">PATH CLEAR</span>
      <span class="telem-sub" id="val-vision-sub">No Obstacle Ahead</span>
    </div>
    <div class="telem-card">
      <span class="telem-label">Target Heading</span>
      <span class="telem-val" id="val-heading">0.0° North</span>
      <span class="telem-sub">QGroundControl Synced</span>
    </div>
  </div>

  <!-- Command Panel -->
  <div class="command-panel">
    <div class="tab-row">
      <div class="tabs">
        <button class="tab-btn active" id="tab-write" onclick="setMode('write')">⌨️ Write Command</button>
        <button class="tab-btn" id="tab-speak" onclick="setMode('speak')">🎙️ Speak Command</button>
      </div>
      <div>
        <select class="lang-select" id="lang-select">
          <option value="en-US">English (US)</option>
          <option value="bn-BD">বাংলা (Bangladesh)</option>
        </select>
      </div>
    </div>

    <!-- Voice Recording Sub-Zone -->
    <div class="voice-zone" id="voice-zone">
      <button class="mic-button" id="mic-btn" onclick="toggleRecording()">🎙️</button>
      <div class="record-status-text" id="record-text">Click the microphone to start speaking</div>
      <div class="soundwave" id="soundwave" style="display:none;">
        <div class="wave-bar"></div><div class="wave-bar"></div><div class="wave-bar"></div><div class="wave-bar"></div><div class="wave-bar"></div>
      </div>
    </div>

    <!-- Text Review & Edit Area -->
    <div class="input-label-row">
      <span class="input-label">Flight Mission Command</span>
      <span class="edit-notice" id="edit-notice">✏️ Edit text below if needed before executing</span>
    </div>

    <textarea class="command-textarea" id="cmd-input" placeholder="Type or speak flight command (e.g. 'take off and then go above in a spiral path', 'dance in celebration', 'উপরে ৩ মিটার ওঠো')..."></textarea>

    <!-- Quick Chips -->
    <div class="chip-row">
      <span class="chip-label">Quick Suggestions:</span>
      <span class="chip" onclick="setCommand('সামনে ৫ মিটার যাও')">⚠️ সামনে ৫ মিটার যাও (বাধা টেস্ট)</span>
      <span class="chip" onclick="setCommand('ডানে ৯০ ডিগ্রি ঘুরে সামনে ৫ মিটার যাও')">🔄 ডানে ঘুরে ৫ মিটার যাও</span>
      <span class="chip" onclick="setCommand('ডানে ৯০ ডিগ্রি ঘুরে ছবি তোলো')">📸 ডানে ঘুরে ছবি তোলো</span>
      <span class="chip" onclick="setCommand('take off and then go above in a spiral path')">🌀 Spiral Ascend</span>
      <span class="chip" onclick="setCommand('dance a little bit in celebration')">💃 Aerial Dance</span>
      <span class="chip" onclick="setCommand('safe landing')">🛬 Land Now</span>
    </div>

    <!-- Actions -->
    <div class="action-row">
      <button class="btn-submit" onclick="executeCommand()">
        <span>🚀 Execute Mission</span>
        <span style="font-size:12px; opacity:0.8;">(Enter)</span>
      </button>
      <button class="btn-land" onclick="setCommand('land'); executeCommand();">
        🛑 Emergency Land
      </button>
    </div>
  </div>

  <!-- Vision Reconnaissance Display -->
  <div class="command-panel" id="vision-panel" style="display:none; padding:18px;">
    <div class="input-label-row">
      <span class="input-label">📸 Aerial Reconnaissance Snapshot (YOLOv8 Detection Feed)</span>
      <span class="edit-notice" id="snap-time">Latest Capture</span>
    </div>
    <div style="margin-top:12px; display:flex; justify-content:center; background:#000; border-radius:10px; overflow:hidden; max-height:360px;">
      <img id="snap-img" src="" style="width:100%; max-height:360px; object-fit:contain;" alt="Aerial Snapshot"/>
    </div>
  </div>

  <!-- Pipeline Logs -->
  <div class="log-panel">
    <div class="log-title">
      <span>AI Pipeline & Mission Telemetry Logs</span>
      <span style="color:var(--cyan); cursor:pointer;" onclick="clearLogs()">Clear</span>
    </div>
    <div class="log-box" id="log-box">
      <div class="log-entry">🤖 System Ready: Qwen 2.5 1.5B LLM & Trajectory Gatekeeper active.</div>
    </div>
  </div>
</div>

<script>
let currentMode = 'write';
let recognition = null;
let isRecording = false;

// Audio Chimes via Web Audio API (Synthesized)
const audioCtx = new (window.AudioContext || window.webkitAudioContext)();
function playChime(type) {
  try {
    const osc = audioCtx.createOscillator();
    const gain = audioCtx.createGain();
    osc.connect(gain);
    gain.connect(audioCtx.destination);
    if (type === 'start') {
      osc.frequency.setValueAtTime(587.33, audioCtx.currentTime); // D5
      osc.frequency.exponentialRampToValueAtTime(880.00, audioCtx.currentTime + 0.15); // A5
      gain.gain.setValueAtTime(0.3, audioCtx.currentTime);
      gain.gain.linearRampToValueAtTime(0.01, audioCtx.currentTime + 0.3);
      osc.start(); osc.stop(audioCtx.currentTime + 0.3);
    } else if (type === 'stop') {
      osc.frequency.setValueAtTime(880.00, audioCtx.currentTime);
      osc.frequency.exponentialRampToValueAtTime(587.33, audioCtx.currentTime + 0.15);
      gain.gain.setValueAtTime(0.3, audioCtx.currentTime);
      gain.gain.linearRampToValueAtTime(0.01, audioCtx.currentTime + 0.3);
      osc.start(); osc.stop(audioCtx.currentTime + 0.3);
    }
  } catch(e) {}
}

function setMode(mode) {
  currentMode = mode;
  document.getElementById('tab-write').classList.toggle('active', mode === 'write');
  document.getElementById('tab-speak').classList.toggle('active', mode === 'speak');
  const vz = document.getElementById('voice-zone');
  if (mode === 'speak') {
    vz.classList.add('active');
  } else {
    vz.classList.remove('active');
    if (isRecording) stopRecording();
  }
}

// Web Speech API Initialization
function initSpeech() {
  const SpeechRec = window.SpeechRecognition || window.webkitSpeechRecognition;
  if (!SpeechRec) {
    alert("Speech recognition is not supported in this browser. Please use Chrome, Edge, or Brave.");
    return false;
  }
  recognition = new SpeechRec();
  recognition.continuous = false;
  recognition.interimResults = true;

  recognition.onstart = function() {
    isRecording = true;
    playChime('start');
    const btn = document.getElementById('mic-btn');
    const txt = document.getElementById('record-text');
    const wave = document.getElementById('soundwave');
    btn.classList.add('recording');
    btn.innerHTML = '🔴';
    txt.className = 'record-status-text recording';
    txt.innerText = '🔴 Listening... Speak now! (শুনছি... কথা বলুন...)';
    wave.style.display = 'flex';
  };

  recognition.onresult = function(event) {
    let transcript = '';
    for (let i = event.resultIndex; i < event.results.length; ++i) {
      transcript += event.results[i][0].transcript;
    }
    document.getElementById('cmd-input').value = transcript;
  };

  recognition.onend = function() {
    isRecording = false;
    playChime('stop');
    const btn = document.getElementById('mic-btn');
    const txt = document.getElementById('record-text');
    const wave = document.getElementById('soundwave');
    btn.classList.remove('recording');
    btn.innerHTML = '🎙️';
    txt.className = 'record-status-text';
    txt.innerText = '✅ Speech Captured! Review or edit below, then click Execute Mission:';
    wave.style.display = 'none';
    document.getElementById('cmd-input').focus();
  };

  recognition.onerror = function(event) {
    isRecording = false;
    const btn = document.getElementById('mic-btn');
    const txt = document.getElementById('record-text');
    const wave = document.getElementById('soundwave');
    btn.classList.remove('recording');
    btn.innerHTML = '🎙️';
    txt.className = 'record-status-text';
    txt.innerText = '⚠️ Microphone error (' + event.error + '). Click to retry.';
    wave.style.display = 'none';
  };

  return true;
}

function toggleRecording() {
  if (!recognition && !initSpeech()) return;
  if (isRecording) {
    recognition.stop();
  } else {
    recognition.lang = document.getElementById('lang-select').value;
    try {
      recognition.start();
    } catch(e) {
      recognition.stop();
    }
  }
}

function setCommand(text) {
  document.getElementById('cmd-input').value = text;
  document.getElementById('cmd-input').focus();
}

function appendLog(msg) {
  const box = document.getElementById('log-box');
  const d = document.createElement('div');
  d.className = 'log-entry';
  d.innerText = msg;
  box.appendChild(d);
  box.scrollTop = box.scrollHeight;
}

function clearLogs() {
  document.getElementById('log-box').innerHTML = '';
}

async function executeCommand() {
  const text = document.getElementById('cmd-input').value.trim();
  if (!text) {
    alert("Please enter or speak a command first!");
    return;
  }

  appendLog("🗣️ Sending command: \\"" + text + "\\"");

  try {
    const res = await fetch('/api/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ command: text })
    });
    const data = await res.json();

    if (data.status === 'approved') {
      appendLog("✅ [APPROVED]: " + data.mission_goal);
      data.logs.forEach(l => appendLog("   " + l));
      appendLog("🚀 Flying " + data.points_count + " trajectory setpoints...");
    } else {
      appendLog("🛑 [REJECTED]: " + (data.reason || "Safety violation"));
    }
  } catch(e) {
    appendLog("❌ Error sending command: " + e.message);
  }
}

// Enter key shortcut
document.getElementById('cmd-input').addEventListener('keydown', function(e) {
  if (e.key === 'Enter' && !e.shiftKey) {
    e.preventDefault();
    executeCommand();
  }
});

// Telemetry Poller (every 1s)
setInterval(async function() {
  try {
    const res = await fetch('/api/telemetry');
    const data = await res.json();
    document.getElementById('val-altitude').innerText = data.altitude_m.toFixed(2) + ' m';
    document.getElementById('val-state').innerText = data.is_airborne ? 'AIRBORNE' : 'GROUNDED';
    document.getElementById('val-state').style.color = data.is_airborne ? '#00f0ff' : '#10b981';
    document.getElementById('val-mode').innerText = 'Mode: ' + data.flight_mode;
    document.getElementById('val-safety').innerText = data.safety_status;
    document.getElementById('val-safety').style.color = (data.safety_status === 'NORMAL') ? '#10b981' : '#f43f5e';

    // Vision Radar Update
    const vVal = document.getElementById('val-vision');
    const vSub = document.getElementById('val-vision-sub');
    if (vVal && vSub) {
      if (data.obstacle_ahead) {
        vVal.innerText = '🚨 ' + (data.obstacle_type ? data.obstacle_type.toUpperCase() : 'OBSTACLE');
        vVal.style.color = '#f43f5e';
        vSub.innerText = 'Proximity: ~' + (data.obstacle_dist ? data.obstacle_dist.toFixed(1) + 'm' : '<4m') + ' Ahead!';
        vSub.style.color = '#f43f5e';
      } else {
        vVal.innerText = 'PATH CLEAR';
        vVal.style.color = '#10b981';
        vSub.innerText = 'No Obstacle Ahead';
        vSub.style.color = '#10b981';
      }
    }

    // Aerial Snapshot Display
    if (data.latest_capture) {
      const snapPanel = document.getElementById('vision-panel');
      const snapImg = document.getElementById('snap-img');
      if (snapPanel && snapImg) {
        snapPanel.style.display = 'flex';
        snapImg.src = data.latest_capture;
      }
    }
  } catch(e) {}
}, 1000);
</script>

</body>
</html>
"""


class WebGCSHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # Silence default request logging to keep console clean
        return

    def do_GET(self):
        if self.path in ["/", "/index.html"]:
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode("utf-8"))

        elif self.path == "/api/telemetry":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(telemetry_state).encode("utf-8"))

        elif self.path.startswith("/captures/"):
            fname = os.path.basename(self.path.split("?")[0])
            fpath = os.path.join(CAPTURES_DIR, fname)
            if os.path.exists(fpath):
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.end_headers()
                with open(fpath, "rb") as f:
                    self.wfile.write(f.read())
            else:
                self.send_response(404)
                self.end_headers()

        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == "/api/command":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length).decode("utf-8")
            try:
                data = json.loads(body)
                command_text = data.get("command", "").strip()
            except Exception:
                command_text = ""

            if not command_text:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "reason": "Empty command"}).encode("utf-8"))
                return

            print(f"\n[Web GCS] 🗣️ Received Mission Request: \"{command_text}\"")

            # 1. Parse with Qwen 2.5 1.5B
            parsed = parse_user_command(command_text)

            # 2. Safety Gatekeeper with Vision Obstacle Awareness
            current_alt = telemetry_state.get("altitude_m", 0.0)
            is_air = telemetry_state.get("is_airborne", False)
            cur_x = ros2_node_instance.current_ned_x if ros2_node_instance else 0.0
            cur_y = ros2_node_instance.current_ned_y if ros2_node_instance else 0.0
            cur_yaw = ros2_node_instance.current_yaw if ros2_node_instance else 0.0

            state = DronePhysicalState(is_airborne=is_air, x=cur_x, y=cur_y, z=current_alt, yaw=cur_yaw)
            is_safe, resolved_steps, simulated_waypoints, logs = validate_and_resolve_mission(
                parsed,
                state,
                obstacle_ahead=telemetry_state.get("obstacle_ahead", False),
                obstacle_type=telemetry_state.get("obstacle_type"),
                obstacle_dist=telemetry_state.get("obstacle_dist")
            )

            response_data = {}
            if is_safe:
                print(f"[Web GCS] 🛡️ Gatekeeper: [ APPROVED ✅ ] Goal: {parsed.get('mission_goal')}")
                telemetry_state["safety_status"] = "NORMAL"
                telemetry_state["last_goal"] = parsed.get("mission_goal")

                # Trigger photo if requested in steps
                if any(s.get("action") in ["take_photo", "search_object"] for s in resolved_steps):
                    if ros2_node_instance:
                        ros2_node_instance.trigger_photo("mission_snapshot")

                cur_x = ros2_node_instance.current_ned_x if ros2_node_instance else 0.0
                cur_y = ros2_node_instance.current_ned_y if ros2_node_instance else 0.0
                cur_z = ros2_node_instance.current_ned_z if ros2_node_instance else -current_alt
                cur_yaw = ros2_node_instance.current_yaw if ros2_node_instance else 0.0

                # Generate 3D timed trajectory
                trajectory = generate_timed_trajectory(
                    resolved_steps,
                    current_x=cur_x,
                    current_y=cur_y,
                    current_z=cur_z,
                    current_yaw=cur_yaw,
                    rate_hz=10.0
                )

                has_land = any(s.get("action") == "land" for s in resolved_steps)

                # Direct flight launch via ROS 2
                if ros2_node_instance:
                    ros2_node_instance.execute_trajectory(trajectory, has_land)

                response_data = {
                    "status": "approved",
                    "mission_goal": parsed.get("mission_goal", "Mission Approved"),
                    "steps": resolved_steps,
                    "points_count": len(trajectory),
                    "logs": logs
                }
            else:
                reason = logs[-1] if logs else "Violated aviation safety boundary"
                print(f"[Web GCS] 🛑 Gatekeeper: [ REJECTED ❌ ] {reason}")
                telemetry_state["safety_status"] = "VIOLATION REJECTED"
                response_data = {
                    "status": "rejected",
                    "reason": reason,
                    "logs": logs
                }

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(response_data).encode("utf-8"))
        else:
            self.send_response(404)
            self.end_headers()


# Full Autonomous Flight Node inside Web GCS
if ROS2_AVAILABLE:
    class GCSBridgeNode(Node):
        def __init__(self):
            super().__init__('web_gcs_flight_node')

            # 1. Disable GCS & RC loss failsafe checks in PX4 SITL
            try:
                subprocess.run(['/opt/px4-gazebo/bin/px4-param', 'set', 'NAV_RCL_ACT', '0'], capture_output=True)
                subprocess.run(['/opt/px4-gazebo/bin/px4-param', 'set', 'NAV_DLL_ACT', '0'], capture_output=True)
                subprocess.run(['/opt/px4-gazebo/bin/px4-param', 'set', 'COM_RCL_EXST', '0'], capture_output=True)
                print("[Web GCS] ✅ PX4 Failsafe parameters updated: NAV_RCL_ACT=0, NAV_DLL_ACT=0, COM_RCL_EXST=0")
            except Exception as e:
                print(f"[Web GCS] Param warning: {e}")

            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )

            # Publishers
            self.offboard_control_mode_pub = self.create_publisher(
                OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
            )
            self.trajectory_setpoint_pub = self.create_publisher(
                TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile
            )
            self.vehicle_command_pub = self.create_publisher(
                VehicleCommand, '/fmu/in/vehicle_command', qos_profile
            )

            # Subscribers (Support both PX4 v1.15 vehicle_status_v4 and standard vehicle_status)
            self.vehicle_status_sub = self.create_subscription(
                VehicleStatus, '/fmu/out/vehicle_status_v4', self.vehicle_status_callback, qos_profile
            )
            self.vehicle_status_legacy_sub = self.create_subscription(
                VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
            )
            self.vehicle_local_pos_sub = self.create_subscription(
                VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_pos_callback, qos_profile
            )

            # Vision Perception & Photo Trigger
            self.vision_sub = self.create_subscription(
                String, '/drone/vision_perception', self.vision_callback, 10
            )
            self.photo_trigger_pub = self.create_publisher(
                String, '/drone/trigger_photo', 10
            )

            # State
            self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
            self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
            self.current_ned_x = 0.0
            self.current_ned_y = 0.0
            self.current_ned_z = 0.0
            self.current_yaw = 0.0
            self.is_airborne = False

            # Trajectory execution buffer
            self.active_trajectory = []
            self.trajectory_index = 0
            self.current_target = [0.0, 0.0, 0.0]
            self.current_target_yaw = 0.0
            self.is_executing = False
            self.has_land_at_end = False

            # 10Hz Heartbeat Timer
            self.timer = self.create_timer(0.1, self.timer_callback)

        def vehicle_status_callback(self, msg: VehicleStatus):
            self.arming_state = msg.arming_state
            self.nav_state = msg.nav_state
            telemetry_state["is_armed"] = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)

        def local_pos_callback(self, msg: VehicleLocalPosition):
            self.current_ned_x = msg.x
            self.current_ned_y = msg.y
            self.current_ned_z = msg.z
            self.current_yaw = msg.heading
            self.is_airborne = (-msg.z > 0.3)
            telemetry_state["altitude_m"] = max(0.0, -msg.z)
            telemetry_state["is_airborne"] = self.is_airborne

        def vision_callback(self, msg: String):
            try:
                data = json.loads(msg.data)
                obs = data.get("obstacle_ahead", False)
                telemetry_state["obstacle_ahead"] = obs
                telemetry_state["obstacle_type"] = data.get("obstacle_type")
                telemetry_state["obstacle_dist"] = data.get("estimated_dist_m")
                if obs:
                    telemetry_state["safety_status"] = f"OBSTACLE: {data.get('obstacle_type', '').upper()}"
                elif telemetry_state["safety_status"].startswith("OBSTACLE"):
                    telemetry_state["safety_status"] = "NORMAL"
            except Exception:
                pass

        def trigger_photo(self, label="aerial_capture"):
            try:
                msg = String()
                msg.data = label
                self.photo_trigger_pub.publish(msg)
                telemetry_state["latest_capture"] = f"/captures/latest.jpg?t={time.time()}"
                print(f"[Web GCS] 📸 Camera Trigger dispatched via /drone/trigger_photo (label: {label})")
            except Exception as e:
                print(f"[Web GCS] Camera trigger warning: {e}")

        def timer_callback(self):
            self.publish_offboard_control_mode()

            if self.is_executing:
                # 1. Ensure vehicle is armed and in offboard mode before moving
                if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                    self.engage_offboard_mode()
                    self.arm()
                    telemetry_state["mission_status"] = "Arming motors..."
                    self.publish_trajectory_setpoint()
                    return

                # 2. Advance trajectory waypoints once vehicle is confirmed armed
                if self.trajectory_index < len(self.active_trajectory):
                    pt = self.active_trajectory[self.trajectory_index]
                    self.current_target = [pt[0], pt[1], pt[2]]
                    self.current_target_yaw = pt[3]
                    self.trajectory_index += 1

                    pct = (self.trajectory_index / len(self.active_trajectory)) * 100.0
                    telemetry_state["mission_status"] = f"Executing: {pct:.0f}%"

                elif self.trajectory_index >= len(self.active_trajectory):
                    self.is_executing = False
                    telemetry_state["mission_status"] = "Hovering (Holding Position)"
                    if self.has_land_at_end:
                        self.land()
                        telemetry_state["mission_status"] = "Landing..."

            self.publish_trajectory_setpoint()

        def execute_trajectory(self, trajectory, has_land=False):
            self.engage_offboard_mode()
            self.arm()
            self.active_trajectory = trajectory
            self.trajectory_index = 0
            self.has_land_at_end = has_land
            self.is_executing = True
            print(f"[Web GCS] 🚀 Direct Flight Execution Started: {len(trajectory)} waypoints (~{len(trajectory)/10.0:.1f}s)")

        def publish_offboard_control_mode(self):
            msg = OffboardControlMode()
            msg.position = True
            msg.velocity = False
            msg.acceleration = False
            msg.attitude = False
            msg.body_rate = False
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.offboard_control_mode_pub.publish(msg)

        def publish_trajectory_setpoint(self):
            msg = TrajectorySetpoint()
            msg.position = [float(x) for x in self.current_target]
            nan = float('nan')
            msg.velocity = [nan, nan, nan]
            msg.acceleration = [nan, nan, nan]
            msg.jerk = [nan, nan, nan]
            msg.yaw = float(self.current_target_yaw)
            msg.yawspeed = nan
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_setpoint_pub.publish(msg)

        def arm(self):
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

        def engage_offboard_mode(self):
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

        def land(self):
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

        def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
            msg = VehicleCommand()
            msg.param1 = float(param1)
            msg.param2 = float(param2)
            msg.command = command
            msg.target_system = 1
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.vehicle_command_pub.publish(msg)


def start_server(port=8080):
    global ros2_node_instance

    if ROS2_AVAILABLE:
        try:
            rclpy.init()
            ros2_node_instance = GCSBridgeNode()
            ros_thread = threading.Thread(target=lambda: rclpy.spin(ros2_node_instance), daemon=True)
            ros_thread.start()
        except Exception as e:
            print(f"ROS 2 note: {e}")

    HTTPServer.allow_reuse_address = True
    server = HTTPServer(('0.0.0.0', port), WebGCSHandler)
    print("=" * 70)
    print(f"🚁 AUTONOMOUS DRONE WEB GROUND CONTROL STATION ACTIVE!")
    print(f"🌐 Access Dashboard in your Browser: http://localhost:{port}")
    print("=" * 70)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping Web GCS server...")
        server.server_close()


if __name__ == '__main__':
    port = 8080
    if len(sys.argv) > 1:
        port = int(sys.argv[1])
    start_server(port)
