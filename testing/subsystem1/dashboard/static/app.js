// Polls /api/state and renders it. Values whose validity flag is clear in
// the frame are shown as "—" rather than as a misleading zero.
"use strict";

const $ = (id) => document.getElementById(id);
const DASH = "—";
const STALE_S = 3;            // telemetry is published about once a second
let after = 0;
let history = [];             // [host seconds, left, right], from the server

function set(id, text, cls) {
  const el = $(id);
  el.textContent = text;
  if (cls !== undefined) el.className = el.className.split(" ")
    .filter((c) => !["on", "off", "warn"].includes(c)).concat(cls ? [cls] : [])
    .join(" ");
}

function age(seconds) {
  if (seconds === null) return "never";
  return seconds < 1 ? "now" : `${seconds.toFixed(0)} s ago`;
}

function render(s) {
  set("link", s.connected ? "broker: connected" : "broker: disconnected",
      s.connected ? "on" : "off");
  const t = s.telemetry;
  const fresh = t && s.telemetry_age !== null && s.telemetry_age < STALE_S;
  set("age", t ? `last frame ${age(s.telemetry_age)}` : "no telemetry yet",
      fresh ? "" : "warn");

  if (t) {
    set("state", t.state_name,
        t.state_name === "FAULT" ? "off" : t.state_name === "RUNNING" ? "on" : "");
    set("mode", t.mode_name);
    // Version 5 adds what Motion Control's fault was (stall, task...).
    set("fault", t.motion_fault_name ? `${t.fault_name} (${t.motion_fault_name})`
        : t.fault_name, t.fault === 0 ? "" : "off");
    set("latched", t.stop_latched ? "LATCHED" : "clear",
        t.stop_latched ? "off" : "on");
    set("cause", t.stop_cause_name ? t.stop_cause_name : "");

    // The count is 4 bits on the wire, so it wraps at 16.
    set("barcode", t.barcode_valid ? t.barcode_text : DASH);
    set("command", t.barcode_valid ? t.command_name : DASH);
    set("barcount", t.barcode_valid ? `${t.barcode_count} (mod 16)` : DASH);
    // Version 5: the command waiting for the next junction, and junctions
    // confirmed (2 bits on the wire, so it wraps at 4).
    set("held", t.version >= 5 ? (t.held_name ? t.held_name : "none") : DASH);
    set("junctions", t.version >= 5 ? `${t.junction_count} (mod 4)` : DASH);

    set("left", t.motion_valid ? `${t.left_mm_s} mm/s` : DASH);
    set("right", t.motion_valid ? `${t.right_mm_s} mm/s` : DASH);
    set("dist", t.distance_valid ? `${t.distance_mm} mm` : DASH);
    set("heading", t.motion_valid ? `${(t.heading_cdeg / 100).toFixed(1)}°` : DASH);
    // Version 6: S4's motion class and the encoder-estimated yaw rate
    // (2 deg/s steps; no gyroscope on the car).
    set("motionclass", t.version >= 6 && t.motion_class_name
        ? t.motion_class_name : DASH);
    set("yaw", t.version >= 6 && t.motion_class_name
        ? `${t.yaw_deg_s}°/s` : DASH);

    const line = t.line_fresh ? t.line_name : null;
    $("sl").classList.toggle("black", line === "LEFT" || line === "BOTH");
    $("sr").classList.toggle("black", line === "RIGHT" || line === "BOTH");
    set("linetext", line === null ? "no fresh line report" : line);

    set("hump", t.hump_valid ? `${t.hump_max_mm} mm` : DASH);
    set("humplast", t.hump_last_valid ? `${t.hump_last_mm} mm` : DASH);
    // Older frames (version 3) have no count.
    set("humpcount", t.version >= 4 ? `${t.hump_count}` : DASH);

    set("fwd", t.forward_valid ? `${t.forward_mm} mm` : DASH);
    set("scan", t.scan_result ?
        `${t.scan_name}${t.bypass_side ? ` (bypass ${t.side_name})` : ""}` : "none");
    set("near", t.profile_valid ? `${t.nearest_mm} mm` : DASH);
    // A raw width is shown even though an edge is missing; it overstates.
    set("width", t.shape_valid ? `${t.width_mm} mm`
        : t.width_raw ? `${t.width_mm} mm (raw, edge missing)` : DASH);
    set("centre", t.shape_valid ? `${t.centre_offset_mm} mm` : DASH);
  }
  history = s.speeds;
  drawSpeeds(s.now);

  renderLetters(s);

  set("frames", `${s.frames} good, ${s.bad_frames} bad`);
  set("hb", s.heartbeat ? `${s.heartbeat} (${age(s.heartbeat_age)})` : "none yet");
  set("stats", s.stats || "none yet");
  set("log", s.log);

  const list = $("events");
  for (const e of s.events) {
    const li = document.createElement("li");
    const when = document.createElement("time");
    when.textContent = new Date(e.host * 1000).toLocaleTimeString();
    li.append(when, e.text);
    list.prepend(li);
    after = Math.max(after, e.seq);
  }
  while (list.children.length > 300) list.lastChild.remove();
}

function drawSpeeds(now) {
  const c = $("speeds");
  const w = c.clientWidth, h = c.clientHeight, dpr = window.devicePixelRatio || 1;
  if (c.width !== w * dpr) { c.width = w * dpr; c.height = h * dpr; }
  const g = c.getContext("2d");
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, w, h);
  const css = getComputedStyle(document.documentElement);
  let max = 50;
  for (const [, l, r] of history) max = Math.max(max, Math.abs(l), Math.abs(r));
  const y = (v) => h / 2 - (v / max) * (h / 2 - 4);
  const x = (t) => w - ((now - t) / 60) * w;
  g.strokeStyle = css.getPropertyValue("--line");
  g.beginPath(); g.moveTo(0, h / 2); g.lineTo(w, h / 2); g.stroke();
  for (const [col, key] of [["--left", 1], ["--right", 2]]) {
    g.strokeStyle = css.getPropertyValue(col);
    g.lineWidth = 2;
    g.beginPath();
    history.forEach((p, i) => (i ? g.lineTo(x(p[0]), y(p[key]))
                                 : g.moveTo(x(p[0]), y(p[key]))));
    g.stroke();
  }
  g.fillStyle = css.getPropertyValue("--muted");
  g.font = "11px system-ui";
  g.fillText(`±${max} mm/s`, 4, 12);
}

async function poll() {
  try {
    const r = await fetch(`/api/state?after=${after}`, { cache: "no-store" });
    render(await r.json());
  } catch (err) {
    set("link", "dashboard server not reachable", "off");
  }
  setTimeout(poll, 500);
}

document.querySelectorAll("button[data-cmd]").forEach((b) => {
  b.addEventListener("click", async () => {
    b.disabled = true;
    try {
      const r = await fetch("/api/command", {
        method: "POST", headers: { "X-Dashboard": "1" }, body: b.dataset.cmd,
      });
      const j = await r.json();
      set("cmdmsg", `${b.dataset.cmd}: ${j.message}`, j.ok ? "" : "off");
    } catch (err) {
      set("cmdmsg", `${b.dataset.cmd}: server not reachable`, "off");
    } finally {
      b.disabled = false;
    }
  });
});

// Barcode letters: the page edits a draft; the server holds the table it
// last sent and re-sends it when the car restarts (S1 decision 6).
const letterSelects = [...document.querySelectorAll("select[data-route]")];
const BOOT = { L: "A", R: "B", S: "C", U: "D" };
let lettersEdited = false;     // keep the user's draft while polling

for (const sel of letterSelects) {
  for (let c = 65; c <= 90; c++) sel.add(new Option(String.fromCharCode(c)));
  sel.addEventListener("change", () => { lettersEdited = true; checkLetters(); });
}

function draftLetters() {
  return Object.fromEntries(letterSelects.map((s) => [s.dataset.route, s.value]));
}

function showLetters(table) {
  for (const sel of letterSelects) sel.value = table[sel.dataset.route];
  checkLetters();
}

function checkLetters() {
  const values = letterSelects.map((s) => s.value);
  let dup = false;
  for (const sel of letterSelects) {
    const clash = values.filter((v) => v === sel.value).length > 1;
    sel.classList.toggle("dup", clash);
    dup = dup || clash;
  }
  $("sendletters").disabled = dup;
  if (dup) set("lettermsg", "Each command needs a different letter.", "off");
  return !dup;
}

function renderLetters(s) {
  if (!lettersEdited) showLetters(s.letters);
  if (!lettersEdited && s.letters_note) {
    set("lettermsg", s.letters_note, s.letters_note.includes("NOT") ? "off" : "");
  }
}

$("bootletters").addEventListener("click", () => {
  lettersEdited = true;
  showLetters(BOOT);
});

$("sendletters").addEventListener("click", async () => {
  if (!checkLetters()) return;
  const b = $("sendletters");
  b.disabled = true;
  try {
    const r = await fetch("/api/letters", {
      method: "POST", headers: { "X-Dashboard": "1" },
      body: JSON.stringify(draftLetters()),
    });
    const j = await r.json();
    set("lettermsg", j.message, j.ok ? "" : "off");
    if (j.ok) lettersEdited = false;
  } catch (err) {
    set("lettermsg", "server not reachable", "off");
  } finally {
    b.disabled = false;
  }
});

poll();
