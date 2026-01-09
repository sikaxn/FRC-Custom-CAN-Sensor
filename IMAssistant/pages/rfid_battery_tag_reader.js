const btnBack = document.getElementById("btnBack");
const btnReadTag = document.getElementById("btnReadTag");
const btnMockRobot = document.getElementById("btnMockRobot");
const btnCharged = document.getElementById("btnCharged");
const btnSetStatus = document.getElementById("btnSetStatus");
const btnInitNew = document.getElementById("btnInitNew");
const btnWriteTag = document.getElementById("btnWriteTag");
const btnLoadJson = document.getElementById("btnLoadJson");
const btnSaveJson = document.getElementById("btnSaveJson");
const btnPrint = document.getElementById("btnPrint");
const btnOpenJsonEditor = document.getElementById("btnOpenJsonEditor");
const btnToggleAdvanced = document.getElementById("btnToggleAdvanced");
const advancedBlock = document.getElementById("advancedBlock");
const readerStatus = document.getElementById("readerStatus");
const uidStatus = document.getElementById("uidStatus");
const opStatus = document.getElementById("opStatus");
const progressText = document.getElementById("progressText");
const jsonText = document.getElementById("jsonText");
const metaSn = document.getElementById("metaSn");
const metaFu = document.getElementById("metaFu");
const metaCc = document.getElementById("metaCc");
const metaNote = document.getElementById("metaNote");
const metaUid = document.getElementById("metaUid");
const noteBadge = document.getElementById("noteBadge");
const usageBody = document.getElementById("usageBody");
const initDialog = document.getElementById("initDialog");
const initCancel = document.getElementById("initCancel");
const initConfirm = document.getElementById("initConfirm");
const manualBlock = document.getElementById("manualBlock");
const bestBlock = document.getElementById("bestBlock");
const manualSn = document.getElementById("manualSn");
const teamNumber = document.getElementById("teamNumber");
const batteryType = document.getElementById("batteryType");
const batteryIdBlock = document.getElementById("batteryIdBlock");
const batteryIdLabel = document.getElementById("batteryIdLabel");
const batteryId = document.getElementById("batteryId");
const specialNote = document.getElementById("specialNote");
const previewText = document.getElementById("previewText");
const statusDialog = document.getElementById("statusDialog");
const statusCancel = document.getElementById("statusCancel");
const INIT_TEAM_KEY = "rfid_init_team";

const openFileInput = document.createElement("input");
openFileInput.type = "file";
openFileInput.accept = ".json,.txt";

const MAX_USAGE = 14;
let currentDoc = null;
let lastUid = "";
let rawFallback = "";

const NOTE_LABELS = {
  0: { label: "Normal", bg: "#dcfce7", fg: "#166534" },
  1: { label: "Practice", bg: "#fef9c3", fg: "#78350f" },
  2: { label: "Scrap", bg: "#fee2e2", fg: "#991b1b" },
  3: { label: "Other", bg: "#dbeafe", fg: "#1e3a8a" },
};

const CHIME_FILES = {
  0: "../assets/audio/chime_normal.mp3",
  1: "../assets/audio/chime_practice.mp3",
  2: "../assets/audio/chime_scrap.mp3",
  3: "../assets/audio/chime_other.mp3",
};
const chimeCache = {};

const USAGE_LABELS = {
  1: { label: "Robot", bg: "#e0f2fe", fg: "#0369a1" },
  2: { label: "Charger", bg: "#dcfce7", fg: "#166534" },
};

btnBack?.addEventListener("click", () => {
  if (window.nav?.back) {
    window.nav.back();
  } else {
    window.location.href = "home.html";
  }
});

function setStatus(el, text) {
  if (el) el.textContent = text || "";
}

function logEvent(type, op, data) {
  if (!window.log?.append) return;
  window.log.append({ type, op, data });
}

function playChime(noteValue) {
  const key = Number.isFinite(noteValue) ? noteValue : 0;
  const src = CHIME_FILES[key] || CHIME_FILES[0];
  if (!chimeCache[src]) {
    chimeCache[src] = new Audio(src);
  }
  const audio = chimeCache[src];
  try {
    audio.currentTime = 0;
    void audio.play();
  } catch {}
}

function setReading(isReading, message = "") {
  document.body.classList.toggle("reading", isReading);
  if (progressText) progressText.textContent = isReading ? message : "";
}

function formatUsageTime(tstr) {
  if (!tstr || tstr === "0000000000") return "Date not available";
  try {
    const yy = tstr.slice(0, 2);
    const mo = tstr.slice(2, 4);
    const da = tstr.slice(4, 6);
    const hh = tstr.slice(6, 8);
    const mm = tstr.slice(8, 10);
    return `20${yy}-${mo}-${da} ${hh}:${mm}`;
  } catch {
    return tstr;
  }
}

function nowYyMMddHHmmUtc() {
  const d = new Date();
  const yy = String(d.getUTCFullYear() % 100).padStart(2, "0");
  const mo = String(d.getUTCMonth() + 1).padStart(2, "0");
  const da = String(d.getUTCDate()).padStart(2, "0");
  const hh = String(d.getUTCHours()).padStart(2, "0");
  const mm = String(d.getUTCMinutes()).padStart(2, "0");
  return `${yy}${mo}${da}${hh}${mm}`;
}

function formatPrintDate(tstr) {
  if (!tstr || tstr === "0000000000") return "Date not available";
  if (tstr.length < 10) return tstr;
  const yy = tstr.slice(0, 2);
  const mo = tstr.slice(2, 4);
  const da = tstr.slice(4, 6);
  const hh = tstr.slice(6, 8);
  const mm = tstr.slice(8, 10);
  return `20${yy}-${mo}-${da} ${hh}:${mm}`;
}

function escapeHtml(value) {
  return String(value || "")
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;");
}

function formatNowLocal() {
  const d = new Date();
  const yyyy = d.getFullYear();
  const mo = String(d.getMonth() + 1).padStart(2, "0");
  const da = String(d.getDate()).padStart(2, "0");
  const hh = String(d.getHours()).padStart(2, "0");
  const mm = String(d.getMinutes()).padStart(2, "0");
  const ss = String(d.getSeconds()).padStart(2, "0");
  return `${yyyy}-${mo}-${da} ${hh}:${mm}:${ss}`;
}

function renderUsageRows(usage, maxRows = 36) {
  const rows = Array.isArray(usage) ? [...usage] : [];
  rows.sort((a, b) => Number(b.i || 0) - Number(a.i || 0));
  let visible = rows;
  let extra = 0;
  if (rows.length > maxRows) {
    extra = rows.length - maxRows;
    visible = rows.slice(0, maxRows);
  }
  const parts = visible.map((ent) => {
    const i = ent?.i ?? 0;
    const t = formatPrintDate(String(ent?.t || ""));
    const d = Number(ent?.d || 0);
    const device = d === 1 ? "Robot" : (d === 2 ? "Charger" : "Unknown");
    const e = ent?.e ?? 0;
    const v = ent?.v ?? 0;
    const rowClass = d === 2 ? " class='charger-row'" : "";
    return (
      `<tr${rowClass}>` +
      `<td class='num'>${escapeHtml(i)}</td>` +
      `<td>${escapeHtml(t)}</td>` +
      `<td>${escapeHtml(device)}</td>` +
      `<td class='num'>${escapeHtml(e)}</td>` +
      `<td class='num'>${escapeHtml(v)}</td>` +
      `</tr>`
    );
  });
  if (!parts.length) {
    parts.push("<tr><td colspan='5' class='muted'>No usage records.</td></tr>");
  }
  if (extra) {
    parts.push(`<tr><td colspan='5' class='muted'>(+${extra} more not shown)</td></tr>`);
  }
  return parts.join("\n");
}

function generatePrintHtml(doc, uid) {
  const sn = String(doc?.sn || "");
  const fu = formatPrintDate(String(doc?.fu || "0000000000"));
  const cc = Number(doc?.cc || 0);
  const n = Number(doc?.n || 0);
  const noteLabel = (NOTE_LABELS[n]?.label) || "Normal";
  const usage = Array.isArray(doc?.u) ? doc.u : [];
  const totalReads = usage.filter((ent) => Number(ent?.d || 0) === 1).length;
  const totalCharges = usage.filter((ent) => Number(ent?.d || 0) === 2).length;
  const now = formatNowLocal();
  return `<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8" />
<title>Battery Report ${escapeHtml(sn)}</title>
<meta name="viewport" content="width=device-width, initial-scale=1" />
<style>
  :root {
    --fg:#000;
    --muted:#222;
    --line:#000;
    --bg:#fff;
  }
  * { box-sizing:border-box; }
  html, body { background:var(--bg); color:var(--fg); }
  body {
    margin:24px;
    font:12px/1.35 system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif;
  }
  h1 { margin:0 0 6px; font-size:16px; font-weight:700; }
  h2 { margin:14px 0 6px; font-size:13px; font-weight:700; }
  .grid {
    display:grid;
    grid-template-columns: 160px 1fr 120px 1fr;
    gap:6px 10px;
    padding:8px;
    border:1px solid var(--line);
  }
  .k { color:var(--muted); text-align:right; }
  .code { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }
  table { width:100%; border-collapse:collapse; }
  th, td { border-top:1px solid var(--line); padding:4px 6px; vertical-align:top; }
  thead th { text-align:left; border-top:none; font-size:11px; font-weight:700; }
  .num { text-align:right; font-variant-numeric: tabular-nums; }
  .muted { color:#444; }
  .foot { margin-top:8px; font-size:11px; color:#111; }
  .charger-row { background:#000; color:#fff;}
  @page {
    size: letter;
    margin: 0.5in;
  }
  @media print {
    body { margin:0; }
    * { -webkit-print-color-adjust: exact; print-color-adjust: exact; }
  }
</style>
<script>
  window.addEventListener('load', function() {
    setTimeout(function() { window.print(); }, 100);
  });
</script>
</head>
<body>
  <h1>Battery Report</h1>
  <div class="grid">
    <div class="k">Serial Number (sn):</div><div class="code">${escapeHtml(sn)}</div>
    <div class="k">UID:</div><div class="code">${escapeHtml(uid || "UNKNOWN")}</div>
    <div class="k">First Use (fu):</div><div>${escapeHtml(fu)}</div>
    <div class="k">Cycle Count (cc):</div><div class="code">${escapeHtml(cc)}</div>
    <div class="k">Note (n):</div><div class="code">${escapeHtml(n)} бк ${escapeHtml(noteLabel)}</div>
  </div>

  <h2>Usage</h2>
  <table>
    <thead>
      <tr>
        <th style="width:48px">#</th>
        <th style="width:160px">Time</th>
        <th style="width:110px">Device</th>
        <th class="num" style="width:90px">e</th>
        <th class="num" style="width:90px">v</th>
      </tr>
    </thead>
    <tbody>
      ${renderUsageRows(usage, 36)}
    </tbody>
  </table>

  <h2>Stats</h2>
  <div class="grid" style="grid-template-columns: 160px 1fr 160px 1fr;">
    <div class="k">Robot records:</div><div class="code">${escapeHtml(totalReads)}</div>
    <div class="k">Charger records:</div><div class="code">${escapeHtml(totalCharges)}</div>
    <div class="k">Total records (u):</div><div class="code">${escapeHtml(usage.length)}</div>
    <div class="k">Generated:</div><div>${escapeHtml(now)}</div>
  </div>

  <div class="foot">
    This report only represents the data currently stored on the NFC tag.
  </div>
</body>
</html>`;
}

function ensureSchema(obj) {
  const base = {
    sn: "",
    fu: "0000000000",
    cc: 0,
    n: 0,
    u: [],
  };
  const doc = { ...base, ...(obj || {}) };
  doc.sn = String(doc.sn || "");
  doc.fu = String(doc.fu || "0000000000");
  doc.cc = Number(doc.cc || 0);
  doc.n = Number(doc.n || 0);
  const usage = Array.isArray(doc.u) ? doc.u : [];
  const normalized = usage
    .map((ent) => ({
      i: Number(ent?.i || 0),
      t: String(ent?.t || "0000000000"),
      d: Number(ent?.d || 0),
      e: Number(ent?.e || 0),
      v: Number(ent?.v || 0),
    }))
    .sort((a, b) => a.i - b.i);
  doc.u = normalized.slice(-MAX_USAGE);
  return doc;
}

function addUsage(doc, d) {
  const next = ensureSchema(doc);
  const usage = [...next.u];
  const maxId = usage.length ? Math.max(...usage.map((u) => u.i)) : 0;
  usage.push({
    i: maxId + 1,
    t: nowYyMMddHHmmUtc(),
    d: Number(d),
    e: 0,
    v: 0,
  });
  next.u = usage.slice(-MAX_USAGE);
  return next;
}

async function refreshUid() {
  if (!window.smartcard?.getStatus) return;
  try {
    const status = await window.smartcard.getStatus();
    lastUid = status?.uid || "";
    setStatus(readerStatus, status?.readers?.length
      ? `Readers: ${status.readers.join(", ")}`
      : "Readers: none");
    setStatus(uidStatus, lastUid ? `UID: ${lastUid}` : "UID: not available");
  } catch (err) {
    setStatus(readerStatus, `Reader error: ${err}`);
  }
}

function renderDoc(doc, rawText = "") {
  currentDoc = doc;
  rawFallback = rawText || "";
  if (doc) {
    if (jsonText) jsonText.value = JSON.stringify(doc, null, 2);
    setStatus(opStatus, "JSON loaded.");
  } else {
    if (jsonText) jsonText.value = rawText;
    setStatus(opStatus, "Loaded raw text.");
  }
  renderMetaAndUsage();
}

function renderMetaAndUsage() {
  if (metaUid) metaUid.textContent = lastUid || "--";
  if (!currentDoc) {
    if (metaSn) metaSn.value = "";
    if (metaFu) metaFu.value = "";
    if (metaCc) metaCc.value = "";
    if (metaNote) metaNote.value = "";
    if (noteBadge) {
      noteBadge.textContent = "Note: -";
      noteBadge.style.background = "#e5e7eb";
      noteBadge.style.color = "#111827";
    }
    if (usageBody) usageBody.innerHTML = "";
    return;
  }

  if (metaSn) metaSn.value = currentDoc.sn || "";
  if (metaFu) metaFu.value = currentDoc.fu || "0000000000";
  if (metaCc) metaCc.value = String(currentDoc.cc ?? 0);
  if (metaNote) metaNote.value = String(currentDoc.n ?? 0);

  const noteInfo = NOTE_LABELS[Number(currentDoc.n)] || NOTE_LABELS[0];
  if (noteBadge) {
    noteBadge.textContent = `Note: ${noteInfo.label} (${currentDoc.n ?? 0})`;
    noteBadge.style.background = noteInfo.bg;
    noteBadge.style.color = noteInfo.fg;
  }

  if (usageBody) {
    const usage = Array.isArray(currentDoc.u) ? [...currentDoc.u] : [];
    usage.sort((a, b) => (b.i || 0) - (a.i || 0));
    usageBody.innerHTML = "";
    usage.forEach((ent) => {
      const tr = document.createElement("tr");
      const dev = USAGE_LABELS[Number(ent.d)] || { label: "Unknown", bg: "#fff", fg: "#000" };
      tr.innerHTML = `
        <td style="padding: 6px;">${ent.i ?? ""}</td>
        <td style="padding: 6px;">${formatUsageTime(ent.t || "")}</td>
        <td style="padding: 6px; background:${dev.bg}; color:${dev.fg}; border-radius: 4px;">${dev.label}</td>
        <td style="padding: 6px;">e=${ent.e ?? 0}, v=${ent.v ?? 0}</td>
      `;
      usageBody.appendChild(tr);
    });
  }
}

async function readTag() {
  if (!window.smartcard?.readNdefText) {
    setStatus(opStatus, "Smartcard API not available.");
    return;
  }
  setStatus(opStatus, "Reading tag...");
  setReading(true, "Reading...");
  await refreshUid();
  try {
    const result = await window.smartcard.readNdefText();
    if (result?.ok) {
      const text = result.text || "";
      try {
        const parsed = JSON.parse(text);
        const doc = ensureSchema(parsed);
        renderDoc(doc);
        playChime(Number(doc.n));
        logEvent("read", "read", doc);
      } catch {
        renderDoc(null, text);
        logEvent("read", "read", { msg: text, time: new Date().toISOString() });
      }
    } else {
      setStatus(opStatus, result?.error || "Read failed.");
    }
  } catch (err) {
    setStatus(opStatus, `Read failed: ${err}`);
  } finally {
    setReading(false);
  }
}

async function writeDoc(doc) {
  if (!window.smartcard?.writeNdefText) {
    setStatus(opStatus, "Smartcard API not available.");
    return false;
  }
  const payload = JSON.stringify(ensureSchema(doc));
  try {
    const result = await window.smartcard.writeNdefText(payload);
    if (result?.ok) {
      setStatus(opStatus, "Write OK.");
      return true;
    }
    setStatus(opStatus, result?.error || "Write failed.");
  } catch (err) {
    setStatus(opStatus, `Write failed: ${err}`);
  }
  return false;
}

async function writeAndRefresh(doc, op) {
  const ok = await writeDoc(doc);
  if (ok) {
    logEvent("write", op || "write", ensureSchema(doc));
    await readTag();
  }
}

btnReadTag?.addEventListener("click", readTag);

btnMockRobot?.addEventListener("click", async () => {
  if (!currentDoc) {
    setStatus(opStatus, "Read a valid JSON tag first or load JSON.");
    return;
  }
  const doc = addUsage(currentDoc, 1);
  await writeAndRefresh(doc, "mock_robot");
});

btnCharged?.addEventListener("click", async () => {
  if (!currentDoc) {
    setStatus(opStatus, "Read a valid JSON tag first or load JSON.");
    return;
  }
  let doc = addUsage(currentDoc, 2);
  doc.cc = Number(doc.cc || 0) + 1;
  await writeAndRefresh(doc, "charged");
});

btnSetStatus?.addEventListener("click", async () => {
  if (!currentDoc) {
    setStatus(opStatus, "Read a valid JSON tag first or load JSON.");
    return;
  }
  openStatusDialog();
});

btnInitNew?.addEventListener("click", async () => {
  openInitDialog();
});

btnWriteTag?.addEventListener("click", async () => {
  if (jsonText && jsonText.value.trim()) {
    try {
      const parsed = JSON.parse(jsonText.value);
      await writeAndRefresh(ensureSchema(parsed), "write_tag");
      return;
    } catch {
      setStatus(opStatus, "JSON parse failed. Fix JSON before writing.");
      return;
    }
  }
  if (!currentDoc) {
    setStatus(opStatus, "Nothing to write.");
    return;
  }
  await writeAndRefresh(currentDoc, "write_tag");
});

btnLoadJson?.addEventListener("click", () => {
  openFileInput.value = "";
  openFileInput.click();
});

openFileInput.addEventListener("change", (evt) => {
  const file = evt.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = (e) => {
    try {
      const text = String(e.target.result || "");
      const parsed = JSON.parse(text);
      renderDoc(ensureSchema(parsed));
      setStatus(opStatus, `Loaded JSON: ${file.name}`);
      logEvent("read", "load_json", ensureSchema(parsed));
    } catch (err) {
      setStatus(opStatus, `Invalid JSON: ${err}`);
    }
  };
  reader.readAsText(file);
});

btnSaveJson?.addEventListener("click", () => {
  const payload = currentDoc
    ? JSON.stringify(ensureSchema(currentDoc), null, 2)
    : (jsonText?.value || "");
  if (!payload) {
    setStatus(opStatus, "Nothing to save.");
    return;
  }
  const blob = new Blob([payload], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = "battery.json";
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
  setStatus(opStatus, "Saved JSON to file.");
  try {
    const parsed = JSON.parse(payload);
    logEvent("write", "save_json", ensureSchema(parsed));
  } catch {}
});

btnPrint?.addEventListener("click", () => {
  let doc = null;
  if (currentDoc) {
    doc = ensureSchema(currentDoc);
  } else if (jsonText?.value?.trim()) {
    try {
      doc = ensureSchema(JSON.parse(jsonText.value));
    } catch {
      setStatus(opStatus, "JSON parse failed. Fix JSON before printing.");
      return;
    }
  }
  if (!doc) {
    setStatus(opStatus, "Nothing to print.");
    return;
  }
  if (!window.printWindow?.open) {
    setStatus(opStatus, "Print window not available.");
    return;
  }
  window.printWindow.open(generatePrintHtml(doc, lastUid || "UNKNOWN"));
  logEvent("read", "print", doc);
});

btnOpenJsonEditor?.addEventListener("click", () => {
  const payload = currentDoc
    ? JSON.stringify(ensureSchema(currentDoc))
    : (jsonText?.value || "");
  const encoded = encodeURIComponent(payload || "");
  const target = `best_json_editor.html?payload=${encoded}`;
  window.location.href = target;
});

renderMetaAndUsage();

function setAdvancedVisible(visible) {
  if (advancedBlock) {
    advancedBlock.style.display = visible ? "block" : "none";
  }
  if (btnToggleAdvanced) {
    btnToggleAdvanced.textContent = visible
      ? "Hide JSON & Advanced"
      : "Show JSON & Advanced";
  }
}

btnToggleAdvanced?.addEventListener("click", () => {
  const isVisible = advancedBlock?.style.display !== "none";
  setAdvancedVisible(!isVisible);
});

setAdvancedVisible(false);

function getInitMode() {
  const selected = document.querySelector('input[name="initMode"]:checked');
  return selected?.value || "manual";
}

function filterAscii(value, maxLen) {
  return value
    .split("")
    .filter((ch) => ch.charCodeAt(0) <= 0x7f)
    .join("")
    .slice(0, maxLen);
}

function filterDigits(value, maxLen) {
  return value.replace(/\D/g, "").slice(0, maxLen);
}

function updateInitFields() {
  const mode = getInitMode();
  if (manualBlock) manualBlock.style.display = mode === "manual" ? "block" : "none";
  if (bestBlock) bestBlock.style.display = mode === "best" ? "block" : "none";
  updateBatteryTypeFields();
  updatePreview();
}

function updateBatteryTypeFields() {
  if (!batteryType || !batteryIdBlock || !batteryIdLabel || !specialNote) return;
  const type = batteryType.value;
  if (type === "special") {
    batteryIdBlock.style.display = "none";
    specialNote.style.display = "block";
  } else {
    batteryIdBlock.style.display = "block";
    specialNote.style.display = "none";
    batteryIdLabel.textContent = type === "old"
      ? "Battery ID (00-98)"
      : "Battery ID (000-899)";
  }
}

function generatePreview() {
  const mode = getInitMode();
  if (mode === "manual") {
    const value = filterAscii(manualSn?.value || "", 8);
    if (!value) return { value: null, error: null };
    return { value, error: null };
  }

  const teamRaw = filterDigits(teamNumber?.value || "", 5);
  if (!teamRaw) return { value: null, error: null };
  const teamNum = Number(teamRaw);
  if (!Number.isFinite(teamNum)) return { value: null, error: "Illegal input" };
  const teamStr = String(teamNum);
  const paddedTeam = teamStr + "-".repeat(Math.max(0, 5 - teamStr.length));

  const type = batteryType?.value || "new";
  if (type === "special") {
    return { value: `${paddedTeam}999`, error: null };
  }

  const idRaw = filterDigits(batteryId?.value || "", type === "old" ? 2 : 3);
  if (!idRaw) return { value: null, error: null };
  const id = Number(idRaw);
  if (!Number.isFinite(id)) return { value: null, error: "Illegal input" };

  if (type === "new") {
    if (id < 0 || id > 899) return { value: null, error: "Illegal input" };
    return { value: `${paddedTeam}${String(id).padStart(3, "0")}`, error: null };
  }

  if (id < 0 || id > 98) return { value: null, error: "Illegal input" };
  return { value: `${paddedTeam}9${String(id).padStart(2, "0")}`, error: null };
}

function updatePreview() {
  if (!previewText || !initConfirm) return;
  const { value, error } = generatePreview();
  if (error) {
    previewText.textContent = error;
    previewText.style.color = "#c00";
    initConfirm.disabled = true;
    return;
  }
  if (!value) {
    previewText.textContent = "Incomplete data — preview unavailable";
    previewText.style.color = "#666";
    initConfirm.disabled = true;
    return;
  }
  previewText.textContent = `Preview: ${value}`;
  previewText.style.color = "#111";
  initConfirm.disabled = false;
}

function openInitDialog() {
  if (!initDialog) return;
  if (manualSn) manualSn.value = "";
  if (teamNumber) {
    teamNumber.value = localStorage.getItem(INIT_TEAM_KEY) || "";
  }
  if (batteryId) batteryId.value = "";
  if (batteryType) batteryType.value = "new";
  const defaultMode = document.querySelector('input[name="initMode"][value="best"]');
  if (defaultMode) defaultMode.checked = true;
  updateInitFields();
  initDialog.classList.add("open");
}

function closeInitDialog() {
  initDialog?.classList.remove("open");
}

function openStatusDialog() {
  statusDialog?.classList.add("open");
}

function closeStatusDialog() {
  statusDialog?.classList.remove("open");
}

manualSn?.addEventListener("input", (e) => {
  manualSn.value = filterAscii(e.target.value, 8);
  updatePreview();
});

teamNumber?.addEventListener("input", (e) => {
  teamNumber.value = filterDigits(e.target.value, 5);
  localStorage.setItem(INIT_TEAM_KEY, teamNumber.value);
  updatePreview();
});

batteryId?.addEventListener("input", (e) => {
  const maxLen = batteryType?.value === "old" ? 2 : 3;
  batteryId.value = filterDigits(e.target.value, maxLen);
  updatePreview();
});

batteryType?.addEventListener("change", () => {
  updateBatteryTypeFields();
  const maxLen = batteryType?.value === "old" ? 2 : 3;
  if (batteryId) batteryId.value = filterDigits(batteryId.value, maxLen);
  updatePreview();
});

document.querySelectorAll('input[name="initMode"]').forEach((el) => {
  el.addEventListener("change", updateInitFields);
});

initCancel?.addEventListener("click", closeInitDialog);

initConfirm?.addEventListener("click", async () => {
  const { value, error } = generatePreview();
  if (error || !value) return;
  closeInitDialog();
  const doc = ensureSchema({
    sn: value,
    fu: nowYyMMddHHmmUtc(),
    cc: 0,
    n: 0,
    u: [],
  });
  await writeAndRefresh(doc, "init_new");
});

statusCancel?.addEventListener("click", closeStatusDialog);

statusDialog?.querySelectorAll("[data-status]").forEach((btn) => {
  btn.addEventListener("click", async (event) => {
    if (!currentDoc) {
      closeStatusDialog();
      return;
    }
    const value = Number(event.currentTarget.getAttribute("data-status"));
    if (Number.isNaN(value) || value < 0 || value > 3) return;
    closeStatusDialog();
    const doc = ensureSchema({ ...currentDoc, n: value });
    await writeAndRefresh(doc, "set_status");
  });
});
