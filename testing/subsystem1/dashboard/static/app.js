"use strict";

const state = {
  config: null,
  commandToken: "",
  events: [],
  telemetry: [],
  lastSequence: 0,
  lastTelemetryHostTime: null,
};

const byId = (id) => document.getElementById(id);
const eventRows = byId("eventRows");
const chart = byId("distanceChart");

function setClass(element, className) {
  element.classList.remove("good", "danger", "warning");
  if (className) element.classList.add(className);
}

function formatEventDetail(event) {
  const data = event.data;
  if (event.kind === "telemetry") {
    const distance = data.distance_valid ? `${data.distance_mm} mm` : "distance invalid";
    const gap = data.gap ? ` · GAP ${data.robot_delta_ms} ms` : ` · Δ ${data.robot_delta_ms ?? "—"} ms`;
    return `${distance} · ${data.stopped ? "STOPPED" : "MOVING"} · flags 0x${data.flags.toString(16).padStart(2, "0")}${gap}`;
  }
  if (event.kind === "heartbeat") return `${data.topic} · ${data.payload}`;
  if (event.kind === "command") return `${data.command} → ${data.topic}`;
  return data.message || JSON.stringify(data);
}

function addEvent(event) {
  state.events.push(event);
  if (state.events.length > 10000) state.events.shift();

  if (event.kind === "telemetry") {
    state.lastTelemetryHostTime = Date.parse(event.host_time);
    state.telemetry.push({ hostTime: event.host_time, ...event.data });
    if (state.telemetry.length > 240) state.telemetry.shift();
    updateTelemetry(event.data);
    drawChart();
  }
  renderEventRows();
}

function updateTelemetry(data) {
  byId("distanceValue").textContent = data.distance_valid ? data.distance_mm : "—";
  byId("distanceValidity").textContent = data.distance_valid ? "VALID SNAPSHOT" : "DISTANCE INVALID";
  byId("clockValue").textContent = data.captured_ms.toLocaleString();
  byId("deltaValue").textContent = `DELTA ${data.robot_delta_ms ?? "—"} ms`;
  byId("rawFrame").textContent = data.raw_hex;

  const stopElement = byId("stopValue");
  stopElement.textContent = data.stopped ? "STOPPED" : "MOVING";
  setClass(stopElement, data.stopped ? "good" : "warning");

  const health = byId("healthValue");
  if (!data.version_supported || data.unknown_flags) {
    health.textContent = "BAD FRAME";
    setClass(health, "danger");
  } else if (data.gap) {
    health.textContent = "GAP FOUND";
    setClass(health, "warning");
  } else {
    health.textContent = "FRAME OK";
    setClass(health, "good");
  }
}

function renderEventRows() {
  const filter = byId("eventFilter").value;
  const visible = state.events.filter((event) => {
    if (filter === "all") return true;
    if (filter === "errors") return event.severity !== "info";
    return event.kind === filter;
  }).slice(-350).reverse();

  eventRows.replaceChildren(...visible.map((event) => {
    const row = document.createElement("tr");
    row.className = event.severity;
    const values = [
      event.sequence,
      event.host_time.replace("T", " "),
      event.kind.toUpperCase(),
      formatEventDetail(event),
    ];
    for (const value of values) {
      const cell = document.createElement("td");
      cell.textContent = value;
      row.appendChild(cell);
    }
    return row;
  }));
}

function drawChart() {
  const context = chart.getContext("2d");
  const ratio = window.devicePixelRatio || 1;
  const width = chart.clientWidth;
  const height = chart.clientHeight;
  chart.width = Math.floor(width * ratio);
  chart.height = Math.floor(height * ratio);
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, width, height);

  context.strokeStyle = "#26302d";
  context.lineWidth = 1;
  for (let y = 35; y < height; y += 45) {
    context.beginPath();
    context.moveTo(0, y + .5);
    context.lineTo(width, y + .5);
    context.stroke();
  }
  for (let x = 50; x < width; x += 90) {
    context.beginPath();
    context.moveTo(x + .5, 0);
    context.lineTo(x + .5, height);
    context.stroke();
  }

  const samples = state.telemetry.filter((sample) => sample.distance_valid);
  if (samples.length < 2) return;
  const distances = samples.map((sample) => sample.distance_mm);
  const min = Math.min(...distances);
  const max = Math.max(...distances);
  const span = Math.max(10, max - min);
  const padding = 24;

  context.strokeStyle = "#b7ff38";
  context.lineWidth = 2;
  context.beginPath();
  samples.forEach((sample, index) => {
    const x = padding + (index / (samples.length - 1)) * (width - padding * 2);
    const y = height - padding - ((sample.distance_mm - min) / span) * (height - padding * 2);
    if (index === 0) context.moveTo(x, y); else context.lineTo(x, y);
  });
  context.stroke();

  samples.forEach((sample, index) => {
    if (!sample.gap) return;
    const x = padding + (index / (samples.length - 1)) * (width - padding * 2);
    context.fillStyle = "#ffb82e";
    context.fillRect(x - 2, 5, 4, height - 10);
  });

  context.fillStyle = "#929b91";
  context.font = "11px Cascadia Mono, monospace";
  context.fillText(`${max} mm`, 10, 16);
  context.fillText(`${min} mm`, 10, height - 10);
}

async function loadConfig() {
  const response = await fetch("/api/config", { cache: "no-store" });
  state.config = await response.json();
  state.commandToken = state.config.command_token;
  byId("brokerValue").textContent = `${state.config.broker}:${state.config.broker_port}`;
  byId("publishedFields").replaceChildren(...state.config.schema.published.map(listItem));
  byId("missingFields").replaceChildren(...state.config.schema.not_published.map(listItem));
}

function listItem(text) {
  const item = document.createElement("li");
  item.textContent = text;
  return item;
}

async function pollEvents() {
  try {
    const response = await fetch(`/api/events?after=${state.lastSequence}`, { cache: "no-store" });
    const payload = await response.json();
    for (const event of payload.events) {
      state.lastSequence = Math.max(state.lastSequence, event.sequence);
      addEvent(event);
    }
  } catch (error) {
    byId("statusText").textContent = "DASHBOARD API OFFLINE";
  }
}

async function pollStatus() {
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    const status = await response.json();
    const lamp = byId("statusLamp");
    lamp.classList.toggle("online", Boolean(status.connected));
    lamp.classList.toggle("offline", !status.connected);
    byId("statusText").textContent = status.connected ? "MQTT CONNECTED" : "MQTT DISCONNECTED";
    byId("sessionValue").textContent = status.log_path || "—";
    byId("frameCount").textContent = status.telemetry_frames;
    byId("gapCount").textContent = status.gap_count;
    byId("missingCount").textContent = status.estimated_missing;
    byId("invalidCount").textContent = status.invalid_frames;
    byId("heartbeatCount").textContent = status.heartbeat_frames;
    byId("reconnectCount").textContent = status.connection_count;
  } catch (error) {
    byId("statusLamp").className = "status-lamp offline";
    byId("statusText").textContent = "DASHBOARD API OFFLINE";
  }
}

function updateAge() {
  if (!state.lastTelemetryHostTime) {
    byId("ageValue").textContent = "AGE —";
    return;
  }
  const ageMs = Math.max(0, Date.now() - state.lastTelemetryHostTime);
  byId("ageValue").textContent = `AGE ${(ageMs / 1000).toFixed(1)} s`;
  if (ageMs > 3000) {
    byId("healthValue").textContent = "STALE";
    setClass(byId("healthValue"), "danger");
  }
}

async function sendCommand(command) {
  const risky = command !== "STOP";
  if (risky && !byId("armCommands").checked) return;
  if (risky && !window.confirm(`${command} can change robot state. Confirm the physical test area is safe and publish ${command}?`)) return;

  byId("commandResult").textContent = `Publishing ${command}…`;
  try {
    const response = await fetch("/api/command", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "X-Dashboard-Token": state.commandToken,
      },
      body: JSON.stringify({ command, confirmed: risky }),
    });
    const payload = await response.json();
    byId("commandResult").textContent = response.ok ? `${command}: ${payload.message}` : `${command}: ${payload.error || payload.message}`;
  } catch (error) {
    byId("commandResult").textContent = `${command}: dashboard API unavailable`;
  }
}

function download(name, content, type) {
  const link = document.createElement("a");
  link.href = URL.createObjectURL(new Blob([content], { type }));
  link.download = name;
  link.click();
  URL.revokeObjectURL(link.href);
}

function exportJson() {
  download(`p2-08-dashboard-${Date.now()}.json`, JSON.stringify(state.events, null, 2), "application/json");
}

function csvCell(value) {
  const text = String(value ?? "");
  return `"${text.replaceAll('"', '""')}"`;
}

function exportCsv() {
  const header = ["sequence", "host_time", "kind", "severity", "captured_ms", "distance_mm", "distance_valid", "stopped", "link_up", "robot_delta_ms", "gap", "estimated_missing", "raw_hex", "detail"];
  const rows = state.events.map((event) => {
    const data = event.data || {};
    return [event.sequence, event.host_time, event.kind, event.severity, data.captured_ms, data.distance_mm, data.distance_valid, data.stopped, data.link_up, data.robot_delta_ms, data.gap, data.estimated_missing, data.raw_hex, formatEventDetail(event)].map(csvCell).join(",");
  });
  download(`p2-08-dashboard-${Date.now()}.csv`, [header.join(","), ...rows].join("\n"), "text/csv");
}

byId("eventFilter").addEventListener("change", renderEventRows);
byId("clearView").addEventListener("click", () => { state.events = []; state.telemetry = []; renderEventRows(); drawChart(); });
byId("exportJson").addEventListener("click", exportJson);
byId("exportCsv").addEventListener("click", exportCsv);
byId("armCommands").addEventListener("change", (event) => {
  document.querySelectorAll("[data-command='START'], [data-command='RESET']").forEach((button) => { button.disabled = !event.target.checked; });
});
document.querySelectorAll("[data-command]").forEach((button) => button.addEventListener("click", () => sendCommand(button.dataset.command)));
window.addEventListener("resize", drawChart);

loadConfig().then(() => {
  pollEvents();
  pollStatus();
  setInterval(pollEvents, 500);
  setInterval(pollStatus, 1000);
  setInterval(updateAge, 250);
}).catch(() => {
  byId("statusText").textContent = "DASHBOARD API OFFLINE";
  byId("statusLamp").className = "status-lamp offline";
});

