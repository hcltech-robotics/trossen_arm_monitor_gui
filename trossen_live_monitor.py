#!/usr/bin/env python3
"""
Copyright 2026 HCLTech, Robotics, РШ, Inc.  All rights reserved.
Trossen Arm Live Data Monitor + Recorder + Gravity Mode

Two tabs:

- LIVE DATA (read-only) -- every signal the SDK exposes: per-joint position/
  velocity/acceleration/effort/external effort/compensation effort, driver +
  rotor temperatures, cartesian pose, controller error state, modes, joint
  limits, versions. Temperatures are color coded green/amber/red with a popup
  alert on entering the critical range. Record any subset to CSV at a chosen
  interval.

- GRAVITY MODE (commands the arm) -- puts the arm in gravity compensation so
  joints can be moved by hand while you watch the data. Free all arm joints at
  once, or one joint at a time with the rest held in idle. The gripper keeps its
  own mode and is never freed. Exits are Hold Position (locks the current pose)
  and Idle Arm (damped hold).

SAFETY: the Live Data tab cannot affect the arm, but the Gravity Mode tab makes
the arm back-drivable -- support it before enabling and keep the workspace clear.
Enabling and holding each need a typed confirmation. On disconnect, Ctrl+C or
SIGTERM the arm is locked at its current pose first if it is still free.

This tool opens its own driver session, and whether the controller accepts two
sessions at once is untested, so run it on its own rather than alongside
trossen_web_gui.py / trossen_arm_service_gui.py.

    python3 trossen_live_monitor.py            # real arm
    python3 trossen_live_monitor.py --demo     # simulated, no hardware needed

Then open http://127.0.0.1:5001
"""
import argparse
import atexit
import csv
import datetime
import os
import signal
import threading
import time

from flask import Flask, jsonify, request, render_template_string

from trossen_live_data import (
    DEFAULT_EE,
    DEFAULT_IP,
    ArmDataSource,
    DemoDataSource,
)


PORT = 5001
LOG_DIR = "live_logs"
ALERT_LOG = os.path.join(LOG_DIR, "alerts.csv")
MIN_INTERVAL_S = 0.05

app = Flask(__name__)

source = ArmDataSource()
lock = threading.Lock()

# Editable at runtime from the Thresholds panel. The SDK documents no numeric
# temperature limit, so these are conservative starting points.
THRESHOLDS = {
    "driver_warn": 60.0,
    "driver_crit": 75.0,
    "rotor_warn": 70.0,
    "rotor_crit": 85.0,
}

recorder = None
static_info = {}


HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Trossen Arm Live Data Monitor By HCLTech (РШ) </title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: #111827;
            color: #f9fafb;
            margin: 0;
            padding: 20px;
        }
        h1 { margin-bottom: 4px; }
        h2 { font-size: 17px; margin: 0 0 12px 0; }
        .subtitle { color: #d1d5db; margin-bottom: 18px; font-size: 14px; }
        .panel {
            background: #1f2937;
            border-radius: 12px;
            padding: 16px;
            margin-bottom: 16px;
        }
        .row {
            display: flex;
            gap: 14px;
            flex-wrap: wrap;
            align-items: flex-end;
        }
        label { display: block; font-size: 13px; color: #d1d5db; margin-bottom: 4px; }
        input[type=text], input[type=number], select {
            background: #374151;
            color: #f9fafb;
            border: 1px solid #4b5563;
            border-radius: 8px;
            padding: 8px 10px;
            font-size: 15px;
        }
        input[type=text] { width: 150px; }
        input[type=number] { width: 100px; }
        button {
            border: 0;
            border-radius: 10px;
            padding: 10px 16px;
            font-size: 15px;
            cursor: pointer;
            color: white;
            background: #4b5563;
        }
        button.primary { background: #22c55e; }
        button.danger { background: #ef4444; }
        button.info { background: #3b82f6; }
        button:disabled { opacity: 0.45; cursor: not-allowed; }
        .pill {
            display: inline-block;
            padding: 6px 12px;
            border-radius: 999px;
            font-weight: bold;
            font-size: 14px;
            background: #374151;
        }
        .pill.live { background: #14532d; color: #bbf7d0; }
        .pill.off  { background: #374151; color: #d1d5db; }
        .pill.err  { background: #7f1d1d; color: #fecaca; }
        table { border-collapse: collapse; width: 100%; font-size: 14px; }
        th, td {
            text-align: right;
            padding: 7px 9px;
            border-bottom: 1px solid #374151;
            font-variant-numeric: tabular-nums;
        }
        th { color: #9ca3af; font-weight: normal; white-space: nowrap; }
        td.name, th.name { text-align: left; font-weight: bold; white-space: nowrap; }
        td.ok   { background: #14532d; color: #bbf7d0; font-weight: bold; }
        td.warn { background: #78350f; color: #fde68a; font-weight: bold; }
        td.crit { background: #b91c1c; color: #ffffff; font-weight: bold; }
        .scroll { overflow-x: auto; }
        .banner {
            background: #7f1d1d;
            color: white;
            padding: 12px 14px;
            border-radius: 10px;
            margin-bottom: 16px;
            font-weight: bold;
            display: none;
        }
        .grid2 { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }

        /* Tab bar. */
        .tabs {
            display: flex;
            gap: 8px;
            margin-bottom: 16px;
            border-bottom: 2px solid #374151;
        }
        .tabs button {
            border-radius: 10px 10px 0 0;
            background: #1f2937;
            color: #9ca3af;
            font-weight: bold;
            margin-bottom: -2px;
            border-bottom: 2px solid transparent;
        }
        .tabs button.active {
            background: #374151;
            color: #f9fafb;
            border-bottom-color: #3b82f6;
        }
        .tabs button.armed { color: #fca5a5; }

        /* Main content + dedicated temperature sidebar. */
        .layout {
            display: grid;
            grid-template-columns: minmax(0, 1fr) 320px;
            gap: 16px;
            align-items: start;
        }
        .sideCol { position: sticky; top: 20px; }
        .tempPanel { margin-bottom: 0; }
        @media (max-width: 1100px) {
            .layout { grid-template-columns: minmax(0, 1fr); }
            .sideCol { position: static; }
        }

        .servo {
            display: flex;
            justify-content: space-between;
            align-items: center;
            gap: 10px;
            padding: 9px 12px;
            border-radius: 10px;
            margin-bottom: 7px;
            border-left: 6px solid transparent;
            background: #111827;
        }
        .servo > div:first-child { min-width: 0; }
        .servo .lbl { font-family: monospace; font-size: 14px; font-weight: bold; }
        .servo .big {
            font-size: 21px;
            font-weight: bold;
            font-variant-numeric: tabular-nums;
            white-space: nowrap;
            flex: 0 0 auto;
        }
        .servo .big .deg { font-size: 13px; font-weight: normal; }
        .servo .sub {
            font-size: 11px;
            color: #9ca3af;
            font-family: monospace;
            white-space: nowrap;
        }
        .servo.ok   { background: #052e16; border-left-color: #22c55e; }
        .servo.ok   .big { color: #86efac; }
        .servo.warn { background: #451a03; border-left-color: #f59e0b; }
        .servo.warn .big { color: #fcd34d; }
        .servo.crit { background: #7f1d1d; border-left-color: #ef4444; }
        .servo.crit .big { color: #ffffff; }
        .servo.crit .sub { color: #fecaca; }
        .servo.crit { animation: flash 1s steps(1) infinite; }
        @keyframes flash { 50% { background: #b91c1c; } }

        /* Gravity mode. */
        .danger-box {
            background: #7f1d1d;
            border: 2px solid #ef4444;
            border-radius: 10px;
            padding: 14px;
            margin-bottom: 16px;
        }
        .danger-box h3 { margin: 0 0 8px 0; }
        .danger-box ul { margin: 0; padding-left: 22px; line-height: 1.6; }
        .state-strip {
            padding: 12px 14px;
            border-radius: 10px;
            margin-bottom: 16px;
            font-weight: bold;
            background: #1f2937;
            border-left: 6px solid #4b5563;
        }
        .state-strip.free {
            background: #7f1d1d;
            border-left-color: #ef4444;
            animation: flash 1.4s steps(1) infinite;
        }
        .state-strip.held { background: #052e16; border-left-color: #22c55e; }
        td.mode-external_effort { background: #7f1d1d; color: #fff; font-weight: bold; }
        td.mode-position { background: #052e16; color: #86efac; }
        td.mode-idle { color: #9ca3af; }
        .confirm-input { width: 110px; text-transform: uppercase; }
        .fieldgroup {
            background: #111827;
            border: 1px solid #374151;
            border-radius: 10px;
            padding: 10px;
            margin-bottom: 10px;
        }
        .fieldgroup .head {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 8px;
        }
        .fieldgroup .head b { font-size: 14px; }
        .checks {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(190px, 1fr));
            gap: 2px 10px;
            max-height: 190px;
            overflow-y: auto;
            font-size: 13px;
        }
        .checks label {
            display: flex;
            gap: 6px;
            align-items: center;
            margin: 0;
            color: #e5e7eb;
            font-family: monospace;
        }
        .tiny { color: #9ca3af; font-size: 13px; }
        .kv { font-family: monospace; font-size: 13px; line-height: 1.7; }
        .modal-back {
            display: none;
            position: fixed;
            inset: 0;
            background: rgba(0,0,0,0.65);
            z-index: 50;
        }
        .modal {
            background: #1f2937;
            border: 3px solid #ef4444;
            border-radius: 14px;
            padding: 22px;
            max-width: 520px;
            margin: 12vh auto 0 auto;
        }
        .modal h3 { margin: 0 0 10px 0; color: #fecaca; }
        .modal ul { margin: 0 0 16px 0; padding-left: 20px; font-family: monospace; }
    </style>
</head>
<body>

<h1>Trossen Arm Live Data Monitor By HCLTech (РШ)</h1>
<p class="subtitle">
    Live data is read-only. <b>The Gravity Mode tab commands the arm</b> — it makes
    the joints back-drivable, so support the arm before enabling it.
    Temperatures: <span style="color:#bbf7d0">green = good</span>,
    <span style="color:#fde68a">amber = warning</span>,
    <span style="color:#fca5a5">red = critical + alert</span>.
</p>

<div class="banner" id="banner"></div>

<div class="panel">
    <h2>Connection</h2>
    <div class="row">
        <div>
            <label>Arm IP</label>
            <input type="text" id="ip" value="{{ default_ip }}">
        </div>
        <div>
            <label>End effector</label>
            <select id="ee">
                <option value="follower">follower</option>
                <option value="leader">leader</option>
                <option value="base">base</option>
                <option value="no_gripper">no_gripper</option>
            </select>
        </div>
        <div>
            <label>Poll interval</label>
            <select id="poll" onchange="restartPolling()">
                <option value="100">100 ms</option>
                <option value="250" selected>250 ms</option>
                <option value="500">500 ms</option>
                <option value="1000">1000 ms</option>
            </select>
        </div>
        <div>
            <label><input type="checkbox" id="clear_error"> clear error on connect</label>
        </div>
        <div>
            <button class="primary" id="btn_connect" onclick="connectArm()">Connect</button>
            <button class="danger" id="btn_disconnect" onclick="disconnectArm()">Disconnect</button>
        </div>
        <div>
            <span class="pill off" id="status">not connected</span>
        </div>
    </div>
    <div class="tiny" id="conn_msg" style="margin-top:10px"></div>
</div>

<div class="tabs">
    <button id="tabbtn_live" class="active" onclick="showTab('live')">Live Data</button>
    <button id="tabbtn_gravity" onclick="showTab('gravity')">Gravity Mode</button>
</div>

<div class="layout">
<div class="mainCol">

<div id="tab_live">

<div class="panel">
    <h2>Joint live data</h2>
    <div class="scroll">
        <table id="joint_table">
            <thead>
                <tr>
                    <th class="name">Joint</th>
                    <th>Position<br>(rad)</th>
                    <th>Velocity<br>(rad/s)</th>
                    <th>Accel<br>(rad/s²)</th>
                    <th>Effort<br>(Nm)</th>
                    <th>Ext effort<br>(Nm)</th>
                    <th>Comp effort<br>(Nm)</th>
                    <th>Driver temp<br>(°C)</th>
                    <th>Rotor temp<br>(°C)</th>
                </tr>
            </thead>
            <tbody id="joint_body">
                <tr><td class="name">---</td><td colspan="8">connect to see live data</td></tr>
            </tbody>
        </table>
    </div>
</div>

<div class="grid2">
    <div class="panel">
        <h2>Cartesian (end effector)</h2>
        <div class="scroll">
            <table>
                <thead>
                    <tr>
                        <th class="name">Axis</th>
                        <th>Position</th>
                        <th>Velocity</th>
                        <th>Accel</th>
                        <th>Ext effort</th>
                    </tr>
                </thead>
                <tbody id="cart_body">
                    <tr><td class="name">---</td><td colspan="4">no data</td></tr>
                </tbody>
            </table>
        </div>
    </div>

    <div class="panel">
        <h2>Temperature thresholds (°C)</h2>
        <div class="row">
            <div>
                <label>Driver warn</label>
                <input type="number" step="1" id="driver_warn" value="{{ th.driver_warn }}">
            </div>
            <div>
                <label>Driver critical</label>
                <input type="number" step="1" id="driver_crit" value="{{ th.driver_crit }}">
            </div>
            <div>
                <label>Rotor warn</label>
                <input type="number" step="1" id="rotor_warn" value="{{ th.rotor_warn }}">
            </div>
            <div>
                <label>Rotor critical</label>
                <input type="number" step="1" id="rotor_crit" value="{{ th.rotor_crit }}">
            </div>
            <div>
                <button class="info" onclick="applyThresholds()">Apply</button>
            </div>
        </div>
        <div class="tiny" style="margin-top:10px">
            The SDK documents no numeric temperature limit, so these are conservative
            defaults. Lower them temporarily to test the red / alert path on a cold arm.
            Alerts are also appended to <code>{{ alert_log }}</code>.
        </div>
        <div class="tiny" id="th_msg" style="margin-top:8px"></div>
    </div>
</div>

<div class="panel">
    <h2>Record live data to CSV</h2>
    <div class="row">
        <div>
            <label>Sample interval (seconds)</label>
            <input type="number" step="0.05" min="0.05" id="interval" value="1.0">
        </div>
        <div>
            <label>File name prefix</label>
            <input type="text" id="fname" value="live">
        </div>
        <div>
            <button class="primary" id="btn_rec" onclick="startRecording()">Start Recording</button>
            <button class="danger" id="btn_rec_stop" onclick="stopRecording()">Stop Recording</button>
        </div>
        <div>
            <span class="pill off" id="rec_status">not recording</span>
        </div>
    </div>
    <div class="tiny" id="rec_msg" style="margin-top:10px"></div>
    <div id="field_groups" style="margin-top:14px">
        <div class="tiny">Connect to load the recordable fields.</div>
    </div>
</div>

<div class="panel">
    <h2>Health &amp; configuration</h2>
    <div class="grid2">
        <div class="kv" id="health_left">no data</div>
        <div class="scroll">
            <table>
                <thead>
                    <tr>
                        <th class="name">Joint</th>
                        <th>Mode</th>
                        <th>Pos min</th>
                        <th>Pos max</th>
                        <th>Vel max</th>
                        <th>Effort max</th>
                    </tr>
                </thead>
                <tbody id="config_body">
                    <tr><td class="name">---</td><td colspan="5">no data</td></tr>
                </tbody>
            </table>
        </div>
    </div>
</div>

</div><!-- /tab_live -->

<div id="tab_gravity" hidden>

    <div class="danger-box">
        <h3>⚠ This tab moves the arm</h3>
        <ul>
            <li><b>Support the arm by hand before enabling.</b> Freed joints become
                back-drivable and the arm can swing under its own weight.</li>
            <li>Clear the workspace and keep the power / emergency stop reachable.</li>
            <li>Start with <b>one joint</b> in a low pose rather than all at once.</li>
            <li>The <b>gripper is never freed</b> — it keeps its own mode and holds
                whatever it is gripping.</li>
        </ul>
    </div>

    <div class="state-strip" id="grav_state">Not connected.</div>

    <div class="panel">
        <h2>Enable gravity compensation</h2>
        <div class="tiny" style="margin-bottom:12px">
            Sets <code>external_effort</code> mode with zero external effort. The
            controller compensates gravity and friction itself, so no control loop
            runs here — if this GUI stops, the arm stays compensated, and the
            controller falls back to idle if the connection drops.
        </div>

        <div class="row" style="margin-bottom:12px">
            <div>
                <label><input type="radio" name="gmode" value="joint" checked
                    onchange="syncGravityForm()"> One joint at a time (safer)</label>
                <label><input type="radio" name="gmode" value="all"
                    onchange="syncGravityForm()"> All arm joints at once</label>
            </div>
            <div>
                <label>Joint to free</label>
                <select id="grav_joint"></select>
            </div>
        </div>

        <div class="row">
            <div>
                <label>Type FREE to confirm</label>
                <input type="text" class="confirm-input" id="grav_confirm"
                    placeholder="FREE" oninput="syncGravityForm()">
            </div>
            <div>
                <button class="danger" id="btn_grav_enable" disabled
                    onclick="enableGravity()">Enable Gravity Mode</button>
            </div>
        </div>
        <div class="tiny" id="grav_msg" style="margin-top:10px"></div>
    </div>

    <div class="panel">
        <h2>Exit</h2>
        <div class="row">
            <div>
                <label>Type HOLD to confirm</label>
                <input type="text" class="confirm-input" id="hold_confirm"
                    placeholder="HOLD" oninput="syncGravityForm()">
            </div>
            <div>
                <button class="primary" id="btn_grav_hold" disabled
                    onclick="holdPosition()">Hold Position</button>
            </div>
            <div>
                <button class="info" onclick="idleArm()">Idle Arm</button>
            </div>
        </div>
        <div class="tiny" style="margin-top:10px">
            <b>Hold Position</b> locks the arm at the pose it is in right now
            (<code>position</code> mode). <b>Idle Arm</b> leaves it in
            <code>idle</code> — a damped hold with no position target, not a limp
            release. Disconnecting or pressing Ctrl+C while joints are free holds
            the pose automatically.
        </div>
    </div>

    <div class="panel">
        <h2>Joint check</h2>
        <div class="tiny" style="margin-bottom:10px">
            Travel is the range each joint has covered since gravity mode was
            enabled — move a joint through its range and watch it grow.
            <button onclick="resetTravel()" style="padding:4px 10px;font-size:13px">
                Reset travel</button>
        </div>
        <div class="scroll">
            <table>
                <thead>
                    <tr>
                        <th class="name">Joint</th>
                        <th>Mode</th>
                        <th>Position<br>(rad)</th>
                        <th>Velocity<br>(rad/s)</th>
                        <th>Effort<br>(Nm)</th>
                        <th>Ext effort<br>(Nm)</th>
                        <th>Travel min<br>(rad)</th>
                        <th>Travel max<br>(rad)</th>
                        <th>Range<br>(rad)</th>
                    </tr>
                </thead>
                <tbody id="grav_body">
                    <tr><td class="name">---</td><td colspan="8">connect to see joints</td></tr>
                </tbody>
            </table>
        </div>
    </div>

</div><!-- /tab_gravity -->

</div><!-- /mainCol -->

<aside class="sideCol">
    <div class="panel tempPanel">
        <h2>Servo Temperatures</h2>
        <div class="tiny" style="margin-bottom:10px">
            Big number is the hotter of the two sensors in that servo.
            <b>J1 = SDK joint 0.</b>
        </div>
        <div id="servo_temps">
            <div class="tiny">connect to see servo temperatures</div>
        </div>
        <div class="tiny" id="temp_summary" style="margin-top:12px"></div>
    </div>
</aside>

</div><!-- /layout -->

<div class="modal-back" id="modal_back">
    <div class="modal">
        <h3>⚠ Temperature Alert</h3>
        <ul id="modal_list"></ul>
        <button class="danger" onclick="closeModal()">Acknowledge</button>
    </div>
</div>

<script>
const AXES = ["x", "y", "z", "rx", "ry", "rz"];
const JOINT_COLS = ["pos", "vel", "accel", "effort", "ext_effort", "comp_effort"];

let jointLabels = [];
let pollTimer = null;
let pollMs = 250;
// Latch: one popup per excursion, re-armed only after the field returns to ok.
let alerted = new Set();
let audioCtx = null;
let numArmJoints = 0;
let lastMotion = null;
// Travel per joint since gravity mode was enabled: {label: {min, max}}.
// Client-side only, so it resets on page reload.
let travel = {};

function fmt(v, digits) {
    if (v === undefined || v === null || v === "") return "---";
    const n = Number(v);
    return isNaN(n) ? String(v) : n.toFixed(digits === undefined ? 4 : digits);
}

async function postJSON(url, body) {
    const res = await fetch(url, {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(body || {})
    });
    return await res.json();
}

function setStatus(id, text, cls) {
    const el = document.getElementById(id);
    el.innerText = text;
    el.className = "pill " + cls;
}

/* ------------------------------------------------------------------- tabs */

let currentTab = "live";

function showTab(name) {
    currentTab = name;
    document.getElementById("tab_live").hidden = (name !== "live");
    document.getElementById("tab_gravity").hidden = (name !== "gravity");
    document.getElementById("tabbtn_live").className =
        (name === "live" ? "active" : "");
    document.getElementById("tabbtn_gravity").className =
        (name === "gravity" ? "active" : "") +
        (lastMotion && lastMotion.gravity_active ? " armed" : "");
    if (location.hash !== "#" + name) history.replaceState(null, "", "#" + name);
}

/* #gravity in the URL opens that tab directly, so the page can be bookmarked
   or reloaded straight into it. */
function initialTab() {
    return location.hash === "#gravity" ? "gravity" : "live";
}

/* ---------------------------------------------------------- gravity mode */

function buildGravityUI(numJoints) {
    numArmJoints = Math.max(numJoints - 1, 0);

    // Only arm joints can be freed; the gripper is deliberately excluded.
    const sel = document.getElementById("grav_joint");
    sel.innerHTML = "";
    for (let i = 0; i < numArmJoints; i++) {
        const opt = document.createElement("option");
        opt.value = i;
        opt.text = servoName(i, numJoints) + "  (SDK joint " + i + ")";
        sel.appendChild(opt);
    }

    const body = document.getElementById("grav_body");
    body.innerHTML = "";
    for (let i = 0; i < numJoints; i++) {
        const label = jointLabels[i] || ("joint" + i);
        const isGripper = (i === numJoints - 1);
        const tr = document.createElement("tr");
        tr.innerHTML =
            '<td class="name">' +
                (isGripper ? "Gripper" : servoName(i, numJoints)) +
                ' <span class="tiny">(' + i + ")</span></td>" +
            '<td id="gm_mode_' + label + '">---</td>' +
            '<td id="gm_pos_' + label + '">---</td>' +
            '<td id="gm_vel_' + label + '">---</td>' +
            '<td id="gm_effort_' + label + '">---</td>' +
            '<td id="gm_ext_' + label + '">---</td>' +
            '<td id="gm_tmin_' + label + '">---</td>' +
            '<td id="gm_tmax_' + label + '">---</td>' +
            '<td id="gm_trange_' + label + '">---</td>';
        body.appendChild(tr);
    }

    resetTravel();
    syncGravityForm();
}

function syncGravityForm() {
    const mode = document.querySelector('input[name="gmode"]:checked');
    const isJoint = mode && mode.value === "joint";
    document.getElementById("grav_joint").disabled = !isJoint;

    const connected = !!(lastMotion && lastMotion.connected);
    document.getElementById("btn_grav_enable").disabled = !connected ||
        document.getElementById("grav_confirm").value.trim().toUpperCase() !== "FREE";
    document.getElementById("btn_grav_hold").disabled = !connected ||
        document.getElementById("hold_confirm").value.trim().toUpperCase() !== "HOLD";
}

function resetTravel() {
    travel = {};
    for (const label of jointLabels) travel[label] = null;
}

let autoShownGravity = false;

function renderGravity(motion, d) {
    lastMotion = motion;

    // If the page loads while the arm is already free (a reload mid-session),
    // surface the tab that says so rather than leaving it hidden.
    if (motion && motion.gravity_active && !autoShownGravity) {
        autoShownGravity = true;
        showTab("gravity");
    } else if (motion && !motion.gravity_active) {
        autoShownGravity = false;
        if (currentTab === "gravity") showTab("gravity");  // refresh armed class
    }

    const strip = document.getElementById("grav_state");
    if (!motion || !motion.connected) {
        strip.className = "state-strip";
        strip.innerText = "Not connected.";
    } else if (motion.gravity_active) {
        const names = motion.free_joints
            .map(j => servoName(j, jointLabels.length)).join(", ");
        strip.className = "state-strip free";
        strip.innerText = "⚠ ARM IS FREE — " + names +
            " back-drivable. Support the arm. Use Hold Position when done.";
    } else {
        strip.className = "state-strip held";
        strip.innerText = "Arm is not free. Joints are holding.";
    }

    for (const m of (motion && motion.modes) || []) {
        const label = m.label;
        const cell = document.getElementById("gm_mode_" + label);
        if (cell) {
            cell.innerText = m.mode;
            cell.className = "mode-" + m.mode;
        }
    }

    if (!d) return;

    for (const label of jointLabels) {
        const pos = Number(d[label + "_pos"]);
        setText("gm_pos_" + label, fmt(d[label + "_pos"]));
        setText("gm_vel_" + label, fmt(d[label + "_vel"]));
        setText("gm_effort_" + label, fmt(d[label + "_effort"]));
        setText("gm_ext_" + label, fmt(d[label + "_ext_effort"]));

        if (!isNaN(pos)) {
            const t = travel[label];
            if (!t) {
                travel[label] = {min: pos, max: pos};
            } else {
                if (pos < t.min) t.min = pos;
                if (pos > t.max) t.max = pos;
            }
            const cur = travel[label];
            setText("gm_tmin_" + label, fmt(cur.min));
            setText("gm_tmax_" + label, fmt(cur.max));
            setText("gm_trange_" + label, fmt(cur.max - cur.min));
        }
    }
}

function setText(id, text) {
    const el = document.getElementById(id);
    if (el) el.innerText = text;
}

async function enableGravity() {
    const mode = document.querySelector('input[name="gmode"]:checked').value;
    const body = {
        mode: mode,
        confirm: document.getElementById("grav_confirm").value.trim().toUpperCase()
    };
    if (mode === "joint") {
        body.joint = Number(document.getElementById("grav_joint").value);
    }

    const data = await postJSON("/gravity/enable", body);
    document.getElementById("grav_msg").innerText =
        data.ok ? data.message : "Failed: " + data.error;

    if (data.ok) {
        document.getElementById("grav_confirm").value = "";
        resetTravel();
        renderGravity(data.motion, null);
        showTab("gravity");
        beep();
    }
    syncGravityForm();
}

async function holdPosition() {
    const data = await postJSON("/gravity/hold", {
        confirm: document.getElementById("hold_confirm").value.trim().toUpperCase()
    });
    document.getElementById("grav_msg").innerText =
        data.ok ? data.message : "Failed: " + data.error;

    if (data.ok) {
        document.getElementById("hold_confirm").value = "";
        renderGravity(data.motion, null);
    }
    syncGravityForm();
}

async function idleArm() {
    const data = await postJSON("/gravity/idle", {});
    document.getElementById("grav_msg").innerText =
        data.ok ? data.message : "Failed: " + data.error;
    if (data.ok) renderGravity(data.motion, null);
    syncGravityForm();
}

/* ------------------------------------------------------------- connection */

async function connectArm() {
    const btn = document.getElementById("btn_connect");
    btn.disabled = true;
    setStatus("status", "connecting...", "off");
    document.getElementById("conn_msg").innerText = "";

    try {
        const data = await postJSON("/connect", {
            ip: document.getElementById("ip").value.trim(),
            ee: document.getElementById("ee").value,
            clear_error: document.getElementById("clear_error").checked
        });

        if (!data.ok) {
            setStatus("status", "connect failed", "err");
            document.getElementById("conn_msg").innerText = data.error || "unknown error";
            return;
        }

        jointLabels = data.static.modes.map(m => m.label);
        buildJointTable(data.static.num_joints);
        buildServoTemps(data.static.num_joints);
        buildGravityUI(data.static.num_joints);
        renderConfig(data.static);
        await loadFields();
        alerted = new Set();
        setStatus("status", "connected", "live");
        document.getElementById("conn_msg").innerText =
            "Connected to " + data.static.ip + " (" + data.static.ee + "), " +
            data.static.num_joints + " joints including gripper.";
        restartPolling();
    } catch (err) {
        setStatus("status", "error", "err");
        document.getElementById("conn_msg").innerText = String(err);
    } finally {
        btn.disabled = false;
    }
}

async function disconnectArm() {
    if (pollTimer) { clearInterval(pollTimer); pollTimer = null; }
    await postJSON("/disconnect", {});
    setStatus("status", "not connected", "off");
    document.getElementById("conn_msg").innerText = "Disconnected.";
    hideBanner();
}

function restartPolling() {
    pollMs = Number(document.getElementById("poll").value);
    if (pollTimer) clearInterval(pollTimer);
    pollTimer = setInterval(poll, pollMs);
    poll();
}

/* -------------------------------------------------- servo temp sidebar */

/* The sidebar is 1-indexed (J1..J6 + Gripper) as asked for; the rest of the UI
   keeps the SDK's 0-based joint numbers, so J1 = SDK joint 0. */
function servoName(index, numJoints) {
    return index === numJoints - 1 ? "Gripper_servo" : "J" + (index + 1) + "_servo";
}

function buildServoTemps(numJoints) {
    const host = document.getElementById("servo_temps");
    host.innerHTML = "";

    for (let i = 0; i < numJoints; i++) {
        const label = jointLabels[i] || ("joint" + i);
        const div = document.createElement("div");
        div.className = "servo";
        div.id = "servo_" + label;
        div.innerHTML =
            '<div><div class="lbl">' + servoName(i, numJoints) + "</div>" +
            '<div class="sub" id="servo_sub_' + label + '">driver --- / rotor ---</div></div>' +
            '<div class="big" id="servo_big_' + label + '">---<span class="deg"> °C</span></div>';
        host.appendChild(div);
    }
}

const STATUS_RANK = {ok: 0, warn: 1, crit: 2};

function updateServoTemps(temperatures, numJoints) {
    let hottest = null;
    let worst = "ok";

    for (let i = 0; i < numJoints; i++) {
        const label = jointLabels[i] || ("joint" + i);
        const drv = temperatures[label + "_driver_temp"];
        const rot = temperatures[label + "_rotor_temp"];

        const row = document.getElementById("servo_" + label);
        if (!row) continue;

        const present = [drv, rot].filter(x => x !== undefined);
        if (!present.length) continue;

        // Headline the hotter sensor, and color by the worse of the two.
        const hotter = present.reduce((a, b) => (b.value > a.value ? b : a));
        const status = present.reduce(
            (s, x) => (STATUS_RANK[x.status] > STATUS_RANK[s] ? x.status : s), "ok");

        row.className = "servo " + status;
        document.getElementById("servo_big_" + label).innerHTML =
            fmt(hotter.value, 1) + '<span class="deg"> °C</span>';
        document.getElementById("servo_sub_" + label).innerText =
            "drv " + (drv ? fmt(drv.value, 1) : "---") +
            " / rot " + (rot ? fmt(rot.value, 1) : "---");

        if (!hottest || hotter.value > hottest.value) {
            hottest = {name: servoName(i, numJoints), value: hotter.value};
        }
        if (STATUS_RANK[status] > STATUS_RANK[worst]) worst = status;
    }

    const summary = document.getElementById("temp_summary");
    if (hottest) {
        summary.innerHTML =
            "Hottest: <b>" + hottest.name + " " + fmt(hottest.value, 1) + " °C</b>" +
            " — all servos " +
            (worst === "ok" ? "in the good range."
                : worst === "warn" ? "OK except a warning." : "— CRITICAL, see alert.");
    } else {
        summary.innerText = "";
    }
}

/* ------------------------------------------------------------ live tables */

function buildJointTable(numJoints) {
    const body = document.getElementById("joint_body");
    body.innerHTML = "";

    for (let i = 0; i < numJoints; i++) {
        const label = jointLabels[i] || ("joint" + i);
        const tr = document.createElement("tr");
        const name = label === "gripper"
            ? "Gripper"
            : "Joint " + i + ' <span class="tiny">(J' + (i + 1) + ")</span>";
        let cells = '<td class="name">' + name + "</td>";
        for (const c of JOINT_COLS) {
            cells += '<td id="v_' + label + "_" + c + '">---</td>';
        }
        cells += '<td id="v_' + label + '_driver_temp">---</td>';
        cells += '<td id="v_' + label + '_rotor_temp">---</td>';
        tr.innerHTML = cells;
        body.appendChild(tr);
    }

    const cart = document.getElementById("cart_body");
    cart.innerHTML = "";
    for (const axis of AXES) {
        const tr = document.createElement("tr");
        tr.innerHTML =
            '<td class="name">' + axis + "</td>" +
            '<td id="v_cart_pos_' + axis + '">---</td>' +
            '<td id="v_cart_vel_' + axis + '">---</td>' +
            '<td id="v_cart_accel_' + axis + '">---</td>' +
            '<td id="v_cart_ext_effort_' + axis + '">---</td>';
        cart.appendChild(tr);
    }
}

function setCell(key, value, digits) {
    const el = document.getElementById("v_" + key);
    if (el) el.innerText = fmt(value, digits);
    return el;
}

async function poll() {
    let data;
    try {
        const res = await fetch("/live");
        data = await res.json();
    } catch (err) {
        setStatus("status", "poll error", "err");
        return;
    }

    if (!data.ok) {
        setStatus("status", data.connected ? "read error" : "not connected", "err");
        document.getElementById("conn_msg").innerText = data.error || "";
        return;
    }

    setStatus("status", "live", "live");
    const d = data.data;

    for (const label of jointLabels) {
        for (const c of JOINT_COLS) setCell(label + "_" + c, d[label + "_" + c]);
    }
    for (const axis of AXES) {
        setCell("cart_pos_" + axis, d["cart_pos_" + axis]);
        setCell("cart_vel_" + axis, d["cart_vel_" + axis]);
        setCell("cart_accel_" + axis, d["cart_accel_" + axis]);
        setCell("cart_ext_effort_" + axis, d["cart_ext_effort_" + axis]);
    }

    // Temperatures: color the cell, then latch-alert on entering critical.
    const entered = [];
    for (const [key, info] of Object.entries(data.temperatures)) {
        const el = setCell(key, info.value, 1);
        if (el) el.className = info.status;

        if (info.status === "crit") {
            if (!alerted.has(key)) {
                alerted.add(key);
                entered.push(key + " = " + fmt(info.value, 1) + " °C (limit " + info.crit + ")");
            }
        } else if (info.status === "ok") {
            alerted.delete(key);
        }
    }

    updateServoTemps(data.temperatures, jointLabels.length);
    renderGravity(data.motion, d);
    renderBanner(data);
    if (entered.length) popAlert(entered);

    document.getElementById("health_left").innerHTML =
        "<b>Controller error:</b> " + (d.error_information || "---") + "<br>" +
        "<b>Output id:</b> " + (d.header_id !== undefined ? d.header_id : "---") + "<br>" +
        "<b>Output timestamp:</b> " + fmt(d.header_timestamp, 4) + " s<br>" +
        "<b>Read latency:</b> " + fmt(d.read_latency_ms, 2) + " ms<br>" +
        "<b>Poll interval:</b> " + pollMs + " ms<br>" +
        "<b>Driver version:</b> " + (data.static.driver_version || "---") + "<br>" +
        "<b>Controller version:</b> " + (data.static.controller_version || "---");

    if (data.recording) renderRecording(data.recording);
}

function renderBanner(data) {
    const msgs = [];

    const crit = Object.entries(data.temperatures)
        .filter(([, i]) => i.status === "crit")
        .map(([k, i]) => k + " " + fmt(i.value, 1) + "°C");
    if (crit.length) msgs.push("TEMPERATURE CRITICAL: " + crit.join(", "));

    const warn = Object.entries(data.temperatures)
        .filter(([, i]) => i.status === "warn")
        .map(([k, i]) => k + " " + fmt(i.value, 1) + "°C");
    if (warn.length) msgs.push("Temperature warning: " + warn.join(", "));

    const err = data.data.error_information;
    if (err && err !== "No error") msgs.push("CONTROLLER: " + err);

    const banner = document.getElementById("banner");
    if (msgs.length) {
        banner.innerHTML = msgs.join("<br>");
        banner.style.display = "block";
    } else {
        hideBanner();
    }
}

function hideBanner() {
    document.getElementById("banner").style.display = "none";
}

/* A modal, not alert(), because alert() blocks the polling timer. */
function popAlert(lines) {
    const list = document.getElementById("modal_list");
    list.innerHTML = lines.map(l => "<li>" + l + "</li>").join("");
    document.getElementById("modal_back").style.display = "block";
    beep();
}

function closeModal() {
    document.getElementById("modal_back").style.display = "none";
}

function beep() {
    try {
        audioCtx = audioCtx || new (window.AudioContext || window.webkitAudioContext)();
        const osc = audioCtx.createOscillator();
        const gain = audioCtx.createGain();
        osc.frequency.value = 880;
        gain.gain.value = 0.15;
        osc.connect(gain).connect(audioCtx.destination);
        osc.start();
        osc.stop(audioCtx.currentTime + 0.35);
    } catch (err) { /* audio is a nicety, never fatal */ }
}

function renderConfig(st) {
    const body = document.getElementById("config_body");
    body.innerHTML = "";

    const modeByJoint = {};
    for (const m of st.modes) modeByJoint[m.joint] = m.mode;

    for (const lim of st.limits) {
        const tr = document.createElement("tr");
        tr.innerHTML =
            '<td class="name">' + (lim.label === "gripper" ? "Gripper" : "Joint " + lim.joint) + "</td>" +
            "<td>" + (modeByJoint[lim.joint] || "---") + "</td>" +
            "<td>" + fmt(lim.position_min, 3) + "</td>" +
            "<td>" + fmt(lim.position_max, 3) + "</td>" +
            "<td>" + fmt(lim.velocity_max, 3) + "</td>" +
            "<td>" + fmt(lim.effort_max, 3) + "</td>";
        body.appendChild(tr);
    }
}

/* -------------------------------------------------------------- thresholds */

async function applyThresholds() {
    const data = await postJSON("/thresholds", {
        driver_warn: Number(document.getElementById("driver_warn").value),
        driver_crit: Number(document.getElementById("driver_crit").value),
        rotor_warn: Number(document.getElementById("rotor_warn").value),
        rotor_crit: Number(document.getElementById("rotor_crit").value)
    });

    document.getElementById("th_msg").innerText = data.ok
        ? "Applied: driver " + data.thresholds.driver_warn + "/" + data.thresholds.driver_crit +
          ", rotor " + data.thresholds.rotor_warn + "/" + data.thresholds.rotor_crit
        : "Failed: " + data.error;

    if (data.ok) alerted = new Set();
}

/* --------------------------------------------------------------- recording */

const DEFAULT_ON = ["per_joint", "temperatures"];

async function loadFields() {
    const res = await fetch("/fields");
    const data = await res.json();
    const host = document.getElementById("field_groups");
    host.innerHTML = "";

    if (!data.ok) {
        host.innerHTML = '<div class="tiny">' + (data.error || "no fields") + "</div>";
        return;
    }

    for (const group of data.groups) {
        if (!group.fields.length) continue;

        const div = document.createElement("div");
        div.className = "fieldgroup";

        const checks = group.fields.map(f => {
            // Positions, velocities and temperatures are the usual suspects.
            const on = DEFAULT_ON.includes(group.name) &&
                (group.name === "temperatures" ||
                 f.key.endsWith("_pos") || f.key.endsWith("_vel"));
            return '<label><input type="checkbox" class="fchk" value="' + f.key + '"' +
                   (on ? " checked" : "") + "> " + f.key +
                   (f.unit ? ' <span class="tiny">(' + f.unit + ")</span>" : "") + "</label>";
        }).join("");

        div.innerHTML =
            '<div class="head"><b>' + group.title + "</b>" +
            '<span><button class="btn-all">all</button> ' +
            '<button class="btn-none">none</button></span></div>' +
            '<div class="checks" data-group="' + group.name + '">' + checks + "</div>";

        // Listeners rather than inline onclick, to keep quoting simple.
        div.querySelector(".btn-all").onclick = () => toggleGroup(group.name, true);
        div.querySelector(".btn-none").onclick = () => toggleGroup(group.name, false);

        host.appendChild(div);
    }
}

function toggleGroup(name, on) {
    document.querySelectorAll('.checks[data-group="' + name + '"] .fchk')
        .forEach(c => { c.checked = on; });
}

function selectedFields() {
    return Array.from(document.querySelectorAll(".fchk"))
        .filter(c => c.checked)
        .map(c => c.value);
}

async function startRecording() {
    const fields = selectedFields();
    if (!fields.length) {
        document.getElementById("rec_msg").innerText =
            "Pick at least one field to record.";
        return;
    }

    const data = await postJSON("/record/start", {
        interval_s: Number(document.getElementById("interval").value),
        fields: fields,
        filename: document.getElementById("fname").value.trim()
    });

    if (!data.ok) {
        document.getElementById("rec_msg").innerText = "Failed: " + data.error;
        return;
    }

    document.getElementById("rec_msg").innerText =
        "Recording " + fields.length + " fields every " +
        data.recording.interval_s + "s to " + data.recording.path;
    renderRecording(data.recording);
}

async function stopRecording() {
    const data = await postJSON("/record/stop", {});
    if (!data.ok) {
        document.getElementById("rec_msg").innerText = "Failed: " + data.error;
        return;
    }
    setStatus("rec_status", "not recording", "off");
    document.getElementById("rec_msg").innerText =
        "Stopped. " + data.recording.rows + " rows written to " + data.recording.path;
}

function renderRecording(rec) {
    if (rec.running) {
        setStatus("rec_status",
            "recording " + rec.rows + " rows / " + fmt(rec.elapsed_s, 1) + "s", "live");
        if (rec.error) {
            document.getElementById("rec_msg").innerText = "Recorder error: " + rec.error;
        }
    } else {
        setStatus("rec_status", "not recording", "off");
    }
}

/* ---------------------------------------------------------------- startup */

(async function init() {
    showTab(initialTab());

    const res = await fetch("/live");
    const data = await res.json();
    if (data.ok) {
        // Server already had a live connection (e.g. page reloaded).
        const st = await (await fetch("/static_info")).json();
        if (st.ok) {
            jointLabels = st.static.modes.map(m => m.label);
            buildJointTable(st.static.num_joints);
            buildServoTemps(st.static.num_joints);
            buildGravityUI(st.static.num_joints);
            renderConfig(st.static);
            await loadFields();
            restartPolling();
        }
    }
})();
</script>
</body>
</html>
"""


# --------------------------------------------------------------- temperatures

def classify_temperatures(data):
    """Tag every *_temp field ok / warn / crit against the current thresholds."""
    out = {}

    for key, value in data.items():
        if key.endswith("_driver_temp"):
            warn, crit = THRESHOLDS["driver_warn"], THRESHOLDS["driver_crit"]
        elif key.endswith("_rotor_temp"):
            warn, crit = THRESHOLDS["rotor_warn"], THRESHOLDS["rotor_crit"]
        else:
            continue

        try:
            temp = float(value)
        except (TypeError, ValueError):
            continue

        if temp >= crit:
            status = "crit"
        elif temp >= warn:
            status = "warn"
        else:
            status = "ok"

        out[key] = {"value": temp, "status": status, "warn": warn, "crit": crit}

    return out


_alert_latch = set()


def log_alerts(temperatures):
    """Append newly-critical fields to live_logs/alerts.csv (one row per excursion)."""
    new_rows = []

    for key, info in temperatures.items():
        if info["status"] == "crit":
            if key not in _alert_latch:
                _alert_latch.add(key)
                new_rows.append((key, info))
        elif info["status"] == "ok":
            _alert_latch.discard(key)

    if not new_rows:
        return

    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        write_header = not os.path.exists(ALERT_LOG)

        with open(ALERT_LOG, "a", newline="") as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow(["timestamp", "ip", "field", "value_c", "crit_limit_c"])
            stamp = datetime.datetime.now().isoformat(timespec="seconds")
            for key, info in new_rows:
                writer.writerow([stamp, source.ip, key, info["value"], info["crit"]])
    except Exception as e:
        print(f"[alert log] {e}")


# ------------------------------------------------------------------- recorder

class Recorder(threading.Thread):
    """Samples selected fields into a CSV at a fixed interval."""

    def __init__(self, path, fields, interval_s):
        super().__init__(daemon=True)
        self.path = path
        self.fields = fields
        self.interval_s = interval_s
        self.stop_event = threading.Event()
        self.rows = 0
        self.error = None
        self.started_at = time.monotonic()

    def status(self):
        return {
            "running": self.is_alive() and not self.stop_event.is_set(),
            "path": self.path,
            "fields": len(self.fields),
            "interval_s": self.interval_s,
            "rows": self.rows,
            "elapsed_s": round(time.monotonic() - self.started_at, 2),
            "error": self.error,
        }

    def run(self):
        try:
            with open(self.path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp", "elapsed_s"] + self.fields)
                f.flush()

                # Absolute deadlines, so the interval does not drift with the
                # time spent reading and writing each row.
                deadline = time.monotonic()

                while not self.stop_event.is_set():
                    try:
                        with lock:
                            data = source.snapshot()
                    except Exception as e:
                        self.error = str(e)
                        if self.stop_event.wait(self.interval_s):
                            break
                        deadline = time.monotonic()
                        continue

                    self.error = None
                    stamp = datetime.datetime.now(datetime.timezone.utc).isoformat()
                    elapsed = round(time.monotonic() - self.started_at, 4)
                    writer.writerow(
                        [stamp, elapsed] + [data.get(k, "") for k in self.fields]
                    )
                    f.flush()
                    self.rows += 1

                    deadline += self.interval_s
                    wait = deadline - time.monotonic()
                    if wait <= 0:
                        # Fell behind; resync rather than spin.
                        deadline = time.monotonic()
                        continue
                    if self.stop_event.wait(wait):
                        break
        except Exception as e:
            self.error = str(e)


# --------------------------------------------------------------------- routes

@app.route("/")
def index():
    return render_template_string(
        HTML_PAGE,
        default_ip=DEFAULT_IP,
        default_ee=DEFAULT_EE,
        th=THRESHOLDS,
        alert_log=ALERT_LOG,
    )


@app.route("/connect", methods=["POST"])
def connect_route():
    global static_info

    data = request.get_json(silent=True) or {}
    ip = (data.get("ip") or DEFAULT_IP).strip()
    ee = data.get("ee") or DEFAULT_EE
    clear_error = bool(data.get("clear_error"))

    try:
        with lock:
            static_info = source.connect(ip, ee, clear_error)
        _alert_latch.clear()
        return jsonify({"ok": True, "static": static_info})
    except Exception as e:
        return jsonify({"ok": False, "error": f"{type(e).__name__}: {e}"})


@app.route("/disconnect", methods=["POST"])
def disconnect_route():
    global static_info

    stop_recorder()
    with lock:
        source.disconnect()
    static_info = {}
    _alert_latch.clear()
    return jsonify({"ok": True})


@app.route("/static_info")
def static_info_route():
    if not static_info:
        return jsonify({"ok": False, "error": "not connected"})
    return jsonify({"ok": True, "static": static_info})


@app.route("/live")
def live_route():
    try:
        with lock:
            data = source.snapshot()
    except Exception as e:
        return jsonify({
            "ok": False,
            "connected": source.connected,
            "error": f"{type(e).__name__}: {e}",
        })

    temperatures = classify_temperatures(data)
    log_alerts(temperatures)

    # Motion state rides along on the poll that is already running, so the
    # Gravity Mode tab needs no timer of its own.
    try:
        with lock:
            motion = source.motion_state()
    except Exception as e:
        motion = {"connected": True, "gravity_active": False,
                  "free_joints": [], "modes": [], "error": str(e)}

    return jsonify({
        "ok": True,
        "connected": True,
        "data": data,
        "temperatures": temperatures,
        "thresholds": THRESHOLDS,
        "static": static_info,
        "motion": motion,
        "recording": recorder.status() if recorder is not None else {"running": False},
    })


@app.route("/fields")
def fields_route():
    try:
        with lock:
            source.require_driver()
            groups = source.field_groups()
        return jsonify({"ok": True, "groups": groups})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)})


@app.route("/thresholds", methods=["POST"])
def thresholds_route():
    data = request.get_json(silent=True) or {}

    try:
        updated = {}
        for key in THRESHOLDS:
            if key in data:
                updated[key] = float(data[key])

        merged = dict(THRESHOLDS)
        merged.update(updated)

        if merged["driver_warn"] > merged["driver_crit"]:
            raise ValueError("driver warn must be <= driver critical")
        if merged["rotor_warn"] > merged["rotor_crit"]:
            raise ValueError("rotor warn must be <= rotor critical")

        THRESHOLDS.update(merged)
        _alert_latch.clear()
        return jsonify({"ok": True, "thresholds": THRESHOLDS})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)})


# ---------------------------------------------------------------- gravity mode
#
# The only routes that command the arm. Each requires a typed confirmation,
# matching the teach_free / position_hold convention in
# trossen_arm_service_gui.py.

@app.route("/gravity/status")
def gravity_status_route():
    try:
        with lock:
            return jsonify({"ok": True, "motion": source.motion_state()})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)})


@app.route("/gravity/enable", methods=["POST"])
def gravity_enable_route():
    data = request.get_json(silent=True) or {}

    try:
        if (data.get("confirm") or "").strip().upper() != "FREE":
            raise ValueError('Type FREE to confirm enabling gravity mode.')

        mode = (data.get("mode") or "all").strip().lower()

        with lock:
            source.require_driver()
            if mode == "all":
                motion = source.gravity_all()
                msg = "All arm joints are free. Support the arm."
            elif mode == "joint":
                if data.get("joint") is None:
                    raise ValueError("No joint selected.")
                index = int(data["joint"])
                motion = source.gravity_joint(index)
                msg = (f"Joint {index} is free; the other arm joints are held "
                       f"in idle.")
            else:
                raise ValueError(f"Unknown mode '{mode}'. Use 'all' or 'joint'.")

        print(f"[gravity] ENABLED ({mode}): free joints {motion['free_joints']}")
        return jsonify({"ok": True, "motion": motion, "message": msg})
    except Exception as e:
        return jsonify({"ok": False, "error": f"{type(e).__name__}: {e}"})


@app.route("/gravity/hold", methods=["POST"])
def gravity_hold_route():
    data = request.get_json(silent=True) or {}

    try:
        if (data.get("confirm") or "").strip().upper() != "HOLD":
            raise ValueError("Type HOLD to confirm locking the current pose.")

        with lock:
            source.require_driver()
            motion = source.hold_position()

        print("[gravity] HOLD: arm locked at its current pose")
        return jsonify({
            "ok": True,
            "motion": motion,
            "message": "Arm is holding its current pose in position mode.",
        })
    except Exception as e:
        return jsonify({"ok": False, "error": f"{type(e).__name__}: {e}"})


@app.route("/gravity/idle", methods=["POST"])
def gravity_idle_route():
    try:
        with lock:
            source.require_driver()
            motion = source.idle_arm()

        print("[gravity] IDLE: arm joints set to idle (damped hold)")
        return jsonify({
            "ok": True,
            "motion": motion,
            "message": "Arm joints are idle -- damped hold, no position target.",
        })
    except Exception as e:
        return jsonify({"ok": False, "error": f"{type(e).__name__}: {e}"})


@app.route("/record/start", methods=["POST"])
def record_start_route():
    global recorder

    data = request.get_json(silent=True) or {}

    try:
        if recorder is not None and recorder.is_alive():
            raise RuntimeError("Recording already in progress.")

        with lock:
            source.require_driver()

        interval_s = float(data.get("interval_s", 1.0))
        if interval_s < MIN_INTERVAL_S:
            raise ValueError(f"Interval must be at least {MIN_INTERVAL_S} s")

        fields = [str(f) for f in (data.get("fields") or [])]
        if not fields:
            raise ValueError("No fields selected.")

        prefix = (data.get("filename") or "live").strip() or "live"
        prefix = "".join(c for c in prefix if c.isalnum() or c in "-_")

        os.makedirs(LOG_DIR, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(LOG_DIR, f"{prefix}_{stamp}.csv")

        recorder = Recorder(path, fields, interval_s)
        recorder.start()

        return jsonify({"ok": True, "recording": recorder.status()})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)})


def stop_recorder():
    global recorder

    if recorder is None:
        return None

    recorder.stop_event.set()
    recorder.join(timeout=5)
    status = recorder.status()
    status["running"] = False
    recorder = None
    return status


@app.route("/record/stop", methods=["POST"])
def record_stop_route():
    status = stop_recorder()
    if status is None:
        return jsonify({"ok": False, "error": "Not recording."})
    return jsonify({"ok": True, "recording": status})


@app.route("/record/status")
def record_status_route():
    if recorder is None:
        return jsonify({"ok": True, "recording": {"running": False}})
    return jsonify({"ok": True, "recording": recorder.status()})


def hold_if_free():
    """Never leave a floating arm behind on the way out.

    Runs on normal exit and on Ctrl+C / SIGTERM. If gravity mode is active the
    arm is locked at its current pose first; otherwise this does nothing.
    """
    try:
        if source.connected and source.gravity_active:
            print("\n[gravity] exiting while arm is free -- holding position first")
            with lock:
                source.hold_position()
            print("[gravity] arm held.")
    except Exception as e:
        print(f"[gravity] could not hold on exit: {e}")
        print("[gravity] SUPPORT THE ARM. The controller falls back to idle "
              "(a damped hold) when the connection drops.")


def _signal_exit(signum, _frame):
    # atexit runs hold_if_free(); raising SystemExit gets us there cleanly.
    print(f"\nReceived signal {signum}, shutting down...")
    raise SystemExit(0)


def main():
    global source

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Use simulated data; no arm required. Temperatures spike periodically "
             "so the red/alert path can be verified."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    if args.demo:
        source = DemoDataSource()
        print("DEMO MODE: simulated data, no arm connection.")

    atexit.register(hold_if_free)
    signal.signal(signal.SIGINT, _signal_exit)
    signal.signal(signal.SIGTERM, _signal_exit)

    print(f"Live data is read-only; the Gravity Mode tab commands the arm.")
    print(f"Open http://{args.host}:{args.port}")
    print(f"Recordings and alerts go to ./{LOG_DIR}/")

    try:
        app.run(host=args.host, port=args.port, debug=False, threaded=True)
    except SystemExit:
        raise
    finally:
        hold_if_free()


if __name__ == "__main__":
    main()
