const { app, BrowserWindow, ipcMain, shell, Menu } = require("electron");
const path = require("path");
const { spawn } = require("child_process");
const fs = require("fs");
const fsp = fs.promises;
const crypto = require("crypto");
const { pathToFileURL } = require("url");
let SerialPort;
try {
  ({ SerialPort } = require("serialport"));
} catch {
  SerialPort = null;
}
let Context;
let SCARD_SHARE_SHARED;
let SCARD_PROTOCOL_T0;
let SCARD_PROTOCOL_T1;
let SCARD_LEAVE_CARD;
let smartcardLoadError = "";
try {
  ({
    Context,
    SCARD_SHARE_SHARED,
    SCARD_PROTOCOL_T0,
    SCARD_PROTOCOL_T1,
    SCARD_LEAVE_CARD,
  } = require("smartcard"));
} catch (err) {
  Context = null;
  smartcardLoadError = String(err);
}

let mainWindow;
let serialPrefs = { preferred: null };
let uiPrefs = { autoHideMenuBar: true };
let bestJsonPayload = "";

const DEFAULT_REPO_URL = "https://studenttechsupport.com/customcanespfw/";
const REPO_STATE_FILE = () => path.join(app.getPath("userData"), "repos.json");
const CACHE_ROOT = () => path.join(app.getPath("userData"), "repo-cache");
const SERIAL_PREFS_FILE = () =>
  path.join(app.getPath("userData"), "serial.json");
const UI_PREFS_FILE = () =>
  path.join(app.getPath("userData"), "ui.json");

function normalizeBaseUrl(url) {
  const u = new URL(url);
  if (!u.pathname.endsWith("/")) u.pathname += "/";
  return u.toString();
}

function repoIdFor(url) {
  return crypto.createHash("sha1").update(url).digest("hex").slice(0, 12);
}

function repoCacheDir(repoId) {
  return path.join(CACHE_ROOT(), repoId);
}

function cacheBaseUrl(repoId) {
  const dir = repoCacheDir(repoId) + path.sep;
  return pathToFileURL(dir).toString();
}

function emitRepoFetchLog(message) {
  if (mainWindow?.webContents) {
    mainWindow.webContents.send("repo:fetchLog", message);
  }
}

async function clearRepoCache(repoId) {
  const dir = repoCacheDir(repoId);
  await fsp.rm(dir, { recursive: true, force: true });
}

async function clearAllRepoCaches() {
  await fsp.rm(CACHE_ROOT(), { recursive: true, force: true });
}

async function resetAppDataAndQuit() {
  const userDataDir = app.getPath("userData");
  const scriptPath = path.join(__dirname, "scripts", "reset-userdata.ps1");
  const tempUserData = path.join(app.getPath("temp"), "imassistant-reset");
  const tempScriptPath = path.join(
    app.getPath("temp"),
    "imassistant-reset-userdata.ps1"
  );
  try {
    if (process.platform === "win32") {
      app.setPath("userData", tempUserData);
      try {
        const scriptText = await fsp.readFile(scriptPath, "utf8");
        await fsp.writeFile(tempScriptPath, scriptText);
      } catch (err) {
        console.warn("[reset] failed to stage reset script:", err);
      }
      const child = spawn(
        "powershell.exe",
        [
          "-NoProfile",
          "-ExecutionPolicy",
          "Bypass",
          "-File",
          tempScriptPath,
          "-UserDataDir",
          userDataDir,
          "-Pid",
          String(process.pid),
          "-ExePath",
          process.execPath,
        ],
        { detached: true, stdio: "ignore" }
      );
      child.unref();
    } else {
      await fsp.rm(userDataDir, { recursive: true, force: true });
    }
  } catch (err) {
    console.warn("[reset] failed to clear userData:", err);
  } finally {
    app.exit(0);
  }
}

async function loadRepoState() {
  try {
    const raw = await fsp.readFile(REPO_STATE_FILE(), "utf8");
    const parsed = JSON.parse(raw);
    if (!parsed.repos) parsed.repos = [];
    return parsed;
  } catch {
    return { activeRepoId: "", repos: [] };
  }
}

async function saveRepoState(state) {
  await fsp.mkdir(app.getPath("userData"), { recursive: true });
  await fsp.writeFile(REPO_STATE_FILE(), JSON.stringify(state, null, 2));
}

async function loadSerialPrefs() {
  try {
    const raw = await fsp.readFile(SERIAL_PREFS_FILE(), "utf8");
    serialPrefs = JSON.parse(raw);
  } catch {
    serialPrefs = { preferred: null };
  }
}

async function saveSerialPrefs() {
  await fsp.mkdir(app.getPath("userData"), { recursive: true });
  await fsp.writeFile(SERIAL_PREFS_FILE(), JSON.stringify(serialPrefs, null, 2));
}

async function loadUiPrefs() {
  try {
    const raw = await fsp.readFile(UI_PREFS_FILE(), "utf8");
    uiPrefs = JSON.parse(raw);
  } catch {
    uiPrefs = { autoHideMenuBar: true };
  }
}

async function saveUiPrefs() {
  await fsp.mkdir(app.getPath("userData"), { recursive: true });
  await fsp.writeFile(UI_PREFS_FILE(), JSON.stringify(uiPrefs, null, 2));
}

function portMatches(pref, port) {
  if (!pref) return false;
  if (pref.serialNumber && port.serialNumber) {
    return pref.serialNumber === port.serialNumber;
  }
  if (pref.path) {
    if (port.path && pref.path === port.path) return true;
    if (port.displayName && pref.path === port.displayName) return true;
    if (port.portName && pref.path === port.portName) return true;
  }
  if (pref.displayName && port.displayName && pref.displayName === port.displayName) {
    return true;
  }

  const normalizeId = (val) => {
    if (val === undefined || val === null || val === "") return "";
    if (typeof val === "number") return val.toString(16).toLowerCase();
    const text = String(val).trim().toLowerCase();
    return text.startsWith("0x") ? text.slice(2) : text;
  };

  const prefVid = normalizeId(pref.vendorId || pref.usbVendorId);
  const prefPid = normalizeId(pref.productId || pref.usbProductId);
  const portVid = normalizeId(port.vendorId || port.usbVendorId);
  const portPid = normalizeId(port.productId || port.usbProductId);
  if (prefVid && prefPid && portVid && portPid) {
    if (prefVid === portVid && prefPid === portPid) return true;
    const prefVidNum = parseInt(prefVid, 16);
    const prefPidNum = parseInt(prefPid, 16);
    const portVidNum = parseInt(portVid, 16);
    const portPidNum = parseInt(portPid, 16);
    if (
      Number.isFinite(prefVidNum) &&
      Number.isFinite(prefPidNum) &&
      Number.isFinite(portVidNum) &&
      Number.isFinite(portPidNum)
    ) {
      return prefVidNum === portVidNum && prefPidNum === portPidNum;
    }
  }
  return false;
}

const NDEF_KEY = Buffer.from([0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7]);
const FFFF_KEY = Buffer.from([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]);
const A0_KEY = Buffer.from([0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5]);
const ZERO_KEY = Buffer.from([0x00, 0x00, 0x00, 0x00, 0x00, 0x00]);

const KEYS_TO_LOAD = [
  { slot: 0, key: NDEF_KEY },
  { slot: 1, key: FFFF_KEY },
];

const COMMON_KEYS = [FFFF_KEY, A0_KEY, NDEF_KEY, ZERO_KEY];

const AUTH_ORDER = [
  { keyType: 0x60, slot: 0 },
  { keyType: 0x60, slot: 1 },
  { keyType: 0x61, slot: 0 },
  { keyType: 0x61, slot: 1 },
];

const START_BLOCK = 4;
const END_BLOCK = 63;

function apduLoadKeyToSlot(slot, key6) {
  if (!key6 || key6.length !== 6) throw new Error("Key must be 6 bytes");
  return [0xff, 0x82, 0x00, slot & 0xff, 0x06, ...key6];
}

function apduAuthenticateBlock(blockNumber, keyType, keySlot) {
  return [
    0xff,
    0x86,
    0x00,
    0x00,
    0x05,
    0x01,
    0x00,
    blockNumber & 0xff,
    keyType & 0xff,
    keySlot & 0xff,
  ];
}

function apduReadBlock(blockNumber) {
  return [0xff, 0xb0, 0x00, blockNumber & 0xff, 0x10];
}

function apduUpdateBlock(blockNumber, data16) {
  if (!data16 || data16.length !== 16) throw new Error("Data must be 16 bytes");
  return [0xff, 0xd6, 0x00, blockNumber & 0xff, 0x10, ...data16];
}

function apduGetUid() {
  return [0xff, 0xca, 0x00, 0x00, 0x00];
}

function isSectorTrailer(blockNumber) {
  return blockNumber % 4 === 3;
}

function parseResponse(buffer) {
  if (!buffer || buffer.length < 2) {
    return { data: Buffer.alloc(0), sw1: 0x00, sw2: 0x00 };
  }
  const sw1 = buffer[buffer.length - 2];
  const sw2 = buffer[buffer.length - 1];
  const data = buffer.slice(0, -2);
  return { data, sw1, sw2 };
}

async function transmit(card, apdu) {
  const response = await card.transmit(Buffer.from(apdu), {
    autoGetResponse: true,
  });
  return parseResponse(response);
}

function requireOk(sw1, sw2, step) {
  if (sw1 !== 0x90 || sw2 !== 0x00) {
    throw new Error(`${step} failed, SW=${sw1.toString(16)}${sw2.toString(16)}`);
  }
}

async function loadKeys(card) {
  for (const { slot, key } of KEYS_TO_LOAD) {
    const { sw1, sw2 } = await transmit(card, apduLoadKeyToSlot(slot, key));
    requireOk(sw1, sw2, `LOAD_KEY slot ${slot}`);
  }
}

async function loadKey(card, key, slot = 0x00) {
  const { sw1, sw2 } = await transmit(card, apduLoadKeyToSlot(slot, key));
  return sw1 === 0x90 && sw2 === 0x00;
}

async function authBlock(card, block, keyType, key, slot = 0x00) {
  const loaded = await loadKey(card, key, slot);
  if (!loaded) return false;
  const { sw1, sw2 } = await transmit(card, apduAuthenticateBlock(block, keyType, slot));
  return sw1 === 0x90 && sw2 === 0x00;
}

async function tryAuth(card, block) {
  for (const key of COMMON_KEYS) {
    const ok = await authBlock(card, block, 0x60, key, 0x00);
    if (ok) return { keyType: 0x60, key };
  }
  for (const key of COMMON_KEYS) {
    const ok = await authBlock(card, block, 0x61, key, 0x00);
    if (ok) return { keyType: 0x61, key };
  }
  return { keyType: null, key: null };
}

async function writeBlock(card, block, data16) {
  const { sw1, sw2 } = await transmit(card, apduUpdateBlock(block, data16));
  return sw1 === 0x90 && sw2 === 0x00;
}

function pickReader(readers, hint = "") {
  if (!readers || readers.length === 0) return null;
  const needle = String(hint || "").toLowerCase();
  if (needle) {
    const match = readers.find((r) => r.name.toLowerCase().includes(needle));
    if (match) return match;
  }
  const acr = readers.find((r) => r.name.toLowerCase().includes("acr122"));
  if (acr) return acr;
  const acs = readers.find((r) => r.name.toLowerCase().includes("acs"));
  if (acs) return acs;
  return readers[0];
}

async function withCard(readerHint, handler) {
  if (!Context) throw new Error("smartcard module not available");
  const ctx = new Context();
  if (!ctx.isValid) throw new Error("PC/SC context invalid");
  const readers = ctx.listReaders();
  const reader = pickReader(readers, readerHint);
  if (!reader) {
    ctx.close();
    throw new Error("No PC/SC readers found");
  }

  let card;
  try {
    card = await reader.connect(
      SCARD_SHARE_SHARED,
      SCARD_PROTOCOL_T0 | SCARD_PROTOCOL_T1
    );
    const result = await handler(card, reader);
    card.disconnect(SCARD_LEAVE_CARD);
    return result;
  } finally {
    try {
      if (card?.connected) card.disconnect(SCARD_LEAVE_CARD);
    } catch {}
    ctx.close();
  }
}

async function tryAuthThenRead(card, block) {
  for (const { keyType, slot } of AUTH_ORDER) {
    const auth = await transmit(card, apduAuthenticateBlock(block, keyType, slot));
    if (auth.sw1 === 0x90 && auth.sw2 === 0x00) {
      const read = await transmit(card, apduReadBlock(block));
      if (read.sw1 === 0x90 && read.sw2 === 0x00 && read.data.length === 16) {
        return read.data;
      }
    }
  }
  return null;
}

async function readUserArea(card) {
  const chunks = [];
  for (let blk = START_BLOCK; blk <= END_BLOCK; blk += 1) {
    if (isSectorTrailer(blk)) continue;
    const data = await tryAuthThenRead(card, blk);
    if (!data) throw new Error(`Auth/read failed at block ${blk}`);
    chunks.push(data);
  }
  return Buffer.concat(chunks);
}

function findNdefValue(data) {
  let i = 0;
  while (i < data.length) {
    const t = data[i];
    if (t === 0x00) {
      i += 1;
      continue;
    }
    if (t === 0xfe) break;
    if (i + 1 >= data.length) break;
    let length;
    let vstart;
    let hdr;
    if (data[i + 1] !== 0xff) {
      length = data[i + 1];
      vstart = i + 2;
      hdr = 2;
    } else {
      if (i + 3 >= data.length) break;
      length = (data[i + 2] << 8) | data[i + 3];
      vstart = i + 4;
      hdr = 4;
    }
    if (t === 0x03) {
      return { offset: vstart, length };
    }
    i += hdr + length;
  }
  throw new Error("NDEF TLV (0x03) not found");
}

function parseFirstRecord(ndefValue) {
  if (ndefValue.length < 3) throw new Error("NDEF too short");
  const hdr = ndefValue[0];
  const tnf = hdr & 0x07;
  const sr = (hdr >> 4) & 1;
  const il = (hdr >> 3) & 1;
  let idx = 1;
  const tlen = ndefValue[idx];
  idx += 1;
  let plen;
  if (sr) {
    plen = ndefValue[idx];
    idx += 1;
  } else {
    if (idx + 4 > ndefValue.length) throw new Error("Truncated NDEF length");
    plen = ndefValue.readUInt32BE(idx);
    idx += 4;
  }
  let idlen = 0;
  if (il) {
    if (idx >= ndefValue.length) throw new Error("Truncated NDEF id length");
    idlen = ndefValue[idx];
    idx += 1;
  }
  if (idx + tlen > ndefValue.length) throw new Error("Truncated NDEF type");
  const type = ndefValue.slice(idx, idx + tlen);
  idx += tlen + idlen;
  if (idx + plen > ndefValue.length) throw new Error("Truncated NDEF payload");
  const payload = ndefValue.slice(idx, idx + plen);
  return { tnf, type, payload };
}

function decodeTextRecordPayload(payload) {
  if (!payload || payload.length === 0) return "";
  const status = payload[0];
  const isUtf16 = (status & 0x80) !== 0;
  const langLen = status & 0x3f;
  if (1 + langLen > payload.length) {
    return payload.toString("utf8");
  }
  const textBytes = payload.slice(1 + langLen);
  return textBytes.toString(isUtf16 ? "utf16le" : "utf8");
}

function buildTextRecordPayload(text, lang = "en") {
  const langBytes = Buffer.from(lang, "ascii").slice(0, 32);
  const textBytes = Buffer.from(text, "utf8");
  const status = langBytes.length & 0x3f;
  return Buffer.concat([Buffer.from([status]), langBytes, textBytes]);
}

function buildTextRecord(text, lang = "en") {
  const payload = buildTextRecordPayload(text, lang);
  const type = Buffer.from("T", "ascii");
  if (payload.length >= 256) {
    const header = Buffer.from([0xc1, type.length]);
    const len = Buffer.alloc(4);
    len.writeUInt32BE(payload.length);
    return Buffer.concat([header, len, type, payload]);
  }
  return Buffer.concat([
    Buffer.from([0xd1, type.length, payload.length]),
    type,
    payload,
  ]);
}

function wrapTlv(ndefMessage) {
  if (ndefMessage.length < 0xff) {
    return Buffer.concat([
      Buffer.from([0x03, ndefMessage.length]),
      ndefMessage,
      Buffer.from([0xfe]),
    ]);
  }
  const len = Buffer.alloc(2);
  len.writeUInt16BE(ndefMessage.length);
  return Buffer.concat([
    Buffer.from([0x03, 0xff]),
    len,
    ndefMessage,
    Buffer.from([0xfe]),
  ]);
}

async function writeUserAreaWithTlv(card, tlv) {
  const blocks = [];
  for (let blk = START_BLOCK; blk <= END_BLOCK; blk += 1) {
    if (!isSectorTrailer(blk)) blocks.push(blk);
  }
  const capacity = blocks.length * 16;
  if (tlv.length > capacity) {
    throw new Error(`NDEF too large (${tlv.length} > ${capacity})`);
  }
  const buf = Buffer.concat([tlv, Buffer.alloc(capacity - tlv.length, 0x00)]);
  for (let i = 0; i < blocks.length; i += 1) {
    const blk = blocks[i];
    const chunk = buf.slice(i * 16, i * 16 + 16);
    let wrote = false;
    for (const { keyType, slot } of AUTH_ORDER) {
      const auth = await transmit(card, apduAuthenticateBlock(blk, keyType, slot));
      if (auth.sw1 === 0x90 && auth.sw2 === 0x00) {
        const write = await transmit(card, apduUpdateBlock(blk, chunk));
        if (write.sw1 === 0x90 && write.sw2 === 0x00) {
          wrote = true;
          break;
        }
      }
    }
    if (!wrote) throw new Error(`Write failed at block ${blk}`);
  }
}

function trailerBlock(sector) {
  return sector * 4 + 3;
}

async function wipeSector(card, sector) {
  const tblock = trailerBlock(sector);
  const { keyType, key } = await tryAuth(card, tblock);
  if (!keyType) {
    return { ok: false, error: `Cannot auth sector ${sector}` };
  }

  for (let i = 0; i < 3; i += 1) {
    const blk = sector * 4 + i;
    if (blk === 0) continue;
    const ok = await authBlock(card, blk, keyType, key, 0x00);
    if (!ok) continue;
    await writeBlock(card, blk, Buffer.alloc(16, 0x00));
  }

  const trailer = Buffer.concat([
    NDEF_KEY,
    Buffer.from([0xff, 0x07, 0x80, 0x69]),
    Buffer.alloc(6, 0xff),
  ]);
  const tOk = await authBlock(card, tblock, keyType, key, 0x00);
  if (!tOk) {
    return { ok: false, error: `Cannot auth trailer ${tblock}` };
  }
  const wrote = await writeBlock(card, tblock, trailer);
  if (!wrote) {
    return { ok: false, error: `Failed to reset trailer ${tblock}` };
  }
  return { ok: true };
}

async function rebuildMad(card) {
  const blk1 = Buffer.from([
    0xd3, 0xf7, 0xd3, 0xf7, 0xd3, 0xf7, 0x03, 0xe1,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  ]);
  const blk2 = Buffer.alloc(16, 0x00);
  const pairs = [
    { block: 1, data: blk1 },
    { block: 2, data: blk2 },
  ];
  for (const { block, data } of pairs) {
    const { keyType, key } = await tryAuth(card, block);
    if (!keyType) continue;
    const ok = await authBlock(card, block, keyType, key, 0x00);
    if (!ok) continue;
    await writeBlock(card, block, data);
  }
}

async function writeNdefReady(card) {
  const textPayload = buildTextRecord("ready", "en");
  const tlv = wrapTlv(textPayload);
  if (tlv.length > 48) {
    throw new Error(`Ready TLV too large (${tlv.length} > 48)`);
  }
  const padded = Buffer.concat([tlv, Buffer.alloc(48 - tlv.length, 0x00)]);
  for (let i = 0; i < 3; i += 1) {
    const blk = 4 + i;
    const { keyType, key } = await tryAuth(card, blk);
    if (!keyType) throw new Error(`Auth failed block ${blk}`);
    const ok = await authBlock(card, blk, keyType, key, 0x00);
    if (!ok) throw new Error(`Re-auth failed block ${blk}`);
    const chunk = padded.slice(i * 16, i * 16 + 16);
    const wrote = await writeBlock(card, blk, chunk);
    if (!wrote) throw new Error(`Write failed block ${blk}`);
  }
}

async function ensureRepoState() {
  const state = await loadRepoState();
  if (!state.repos.length) {
    const baseUrl = normalizeBaseUrl(DEFAULT_REPO_URL);
    const id = repoIdFor(baseUrl);
    state.repos.push({
      id,
      name: "Default Repo",
      baseUrl,
      lastFetchedAt: "",
      lastHash: "",
      lastCheckedAt: "",
    });
    state.activeRepoId = id;
    await saveRepoState(state);
  } else if (!state.activeRepoId) {
    state.activeRepoId = state.repos[0].id;
    await saveRepoState(state);
  }
  return state;
}

function safeRelativePath(baseUrl, targetUrl) {
  const base = new URL(baseUrl);
  const target = new URL(targetUrl, base);
  const basePath = base.pathname.endsWith("/")
    ? base.pathname
    : base.pathname + "/";
  const targetPath = target.pathname;
  let rel;
  if (target.origin === base.origin && targetPath.startsWith(basePath)) {
    rel = path.posix.relative(basePath, targetPath);
  } else {
    const host = target.hostname.replace(/[^a-z0-9.-]/gi, "_");
    rel = path.posix.join("_external", host, targetPath.replace(/^\/+/, ""));
  }
  rel = rel.replace(/^\/+/, "");
  if (!rel || rel.startsWith("..")) rel = path.posix.join("_external", "misc");
  return rel;
}

async function fetchTextWithHash(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`Failed to fetch ${url}: ${res.status}`);
  const text = await res.text();
  const hash = crypto.createHash("sha256").update(text).digest("hex");
  return { text, hash };
}

async function hashFile(filePath) {
  const buf = await fsp.readFile(filePath);
  return crypto.createHash("sha256").update(buf).digest("hex");
}

async function downloadToCache(baseUrl, repoDir, targetUrl) {
  const resolved = new URL(targetUrl, baseUrl).toString();
  const relPath = safeRelativePath(baseUrl, resolved);
  const destPath = path.join(repoDir, relPath);
  const resolvedPath = path.resolve(destPath);
  const resolvedRoot = path.resolve(repoDir);
  if (!resolvedPath.startsWith(resolvedRoot)) {
    throw new Error(`Refusing to write outside cache: ${resolvedPath}`);
  }
  await fsp.mkdir(path.dirname(destPath), { recursive: true });
  emitRepoFetchLog(`GET ${resolved}`);
  const res = await fetch(resolved);
  if (!res.ok) throw new Error(`Failed to fetch ${resolved}: ${res.status}`);
  const buf = Buffer.from(await res.arrayBuffer());
  await fsp.writeFile(destPath, buf);
  return { url: resolved, path: destPath, relPath };
}

function extractMarkdownImages(markdownText) {
  const images = [];
  const re = /!\[[^\]]*]\(([^)\s]+)(?:\s+"[^"]*")?\)/g;
  let match;
  while ((match = re.exec(markdownText))) {
    images.push(match[1].replace(/^<|>$/g, ""));
  }
  return images;
}

function extractManifestParts(manifestText, manifestUrl) {
  const urls = [];
  try {
    const data = JSON.parse(manifestText);
    const builds = Array.isArray(data.builds) ? data.builds : [];
    for (const build of builds) {
      const parts = Array.isArray(build.parts) ? build.parts : [];
      for (const part of parts) {
        const partPath = part.path || part.url;
        if (partPath) {
          urls.push(new URL(partPath, manifestUrl).toString());
        }
      }
    }
  } catch {
    return [];
  }
  return urls;
}

async function cacheRepoAssets(repo, repoDir, firmwareDefs) {
  const errors = [];
  const markdownUrls = new Set();
  const manifestUrls = new Set();

  for (const def of firmwareDefs) {
    if (def.markdown_up) markdownUrls.add(def.markdown_up);
    if (def.markdown_down) markdownUrls.add(def.markdown_down);

    if (def.manifest) manifestUrls.add(def.manifest);

    if (def.manifestTemplate) {
      const ids = Array.isArray(def.canIds) && def.canIds.length
        ? def.canIds
        : [1, 2, 3, 4, 5, 6, 7, 8];
      for (const id of ids) {
        manifestUrls.add(def.manifestTemplate.replace("{CAN_ID}", id));
      }
    }
  }

  const downloaded = new Set();
  const downloadText = async (url) => {
    const resolved = new URL(url, repo.baseUrl).toString();
    if (downloaded.has(resolved)) {
      const cachePath = path.join(repoDir, safeRelativePath(repo.baseUrl, resolved));
      return await fsp.readFile(cachePath, "utf8");
    }
    const result = await downloadToCache(repo.baseUrl, repoDir, resolved);
    downloaded.add(result.url);
    return await fsp.readFile(result.path, "utf8");
  };

  for (const url of markdownUrls) {
    try {
      const text = await downloadText(url);
      const imageUrls = extractMarkdownImages(text);
      for (const img of imageUrls) {
        try {
          await downloadToCache(repo.baseUrl, repoDir, img);
        } catch (err) {
          errors.push(String(err));
        }
      }
    } catch (err) {
      errors.push(String(err));
    }
  }

  for (const url of manifestUrls) {
    try {
      const manifestText = await downloadText(url);
      const partUrls = extractManifestParts(
        manifestText,
        new URL(url, repo.baseUrl).toString()
      );
      for (const partUrl of partUrls) {
        try {
          await downloadToCache(repo.baseUrl, repoDir, partUrl);
        } catch (err) {
          errors.push(String(err));
        }
      }
    } catch (err) {
      errors.push(String(err));
    }
  }

  return errors;
}

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 900,
    height: 650,
    autoHideMenuBar: uiPrefs.autoHideMenuBar,
    webPreferences: {
      preload: path.join(__dirname, "preload.js"),
      contextIsolation: true,
      nodeIntegration: false,
    },
  });

  // ================================
  // Web Serial support (Electron)
  // ================================
  const ses = mainWindow.webContents.session;

  // Provide a "port selection" for navigator.serial.requestPort()
  ses.on("select-serial-port", (event, portList, webContents, callback) => {
    event.preventDefault();

    // DEBUG: see what ports Electron sees (shows in your terminal)
    console.log("[select-serial-port] ports:", portList);

    // Prefer a user-selected port if it matches the current list.
    const selected =
      portList.find((port) => portMatches(serialPrefs.preferred, port)) ||
      portList[0];

    if (!selected) {
      console.log("[select-serial-port] no ports available");
      callback(""); // causes requestPort() to reject
      return;
    }

    console.log("[select-serial-port] selected:", selected);
    callback(selected.portId);
  });

  // Auto-grant permission for serial devices
  // (Without this, some Electron builds will still block Web Serial.)
  if (typeof ses.setDevicePermissionHandler === "function") {
    ses.setDevicePermissionHandler((details) => {
      if (details.deviceType === "serial") return true;
      return false;
    });
  }

  // Optional: when devices are added/removed (helpful for debugging)
  ses.on("serial-port-added", (_event, port) => {
    console.log("[serial-port-added]", port);
  });
  ses.on("serial-port-removed", (_event, port) => {
    console.log("[serial-port-removed]", port);
  });

  mainWindow.loadFile(path.join(__dirname, "pages", "home.html"));
}

function buildMenu() {
  const template = [
    {
      label: "IM Assistant",
      submenu: [
        {
          label: "Reload",
          accelerator: "CmdOrCtrl+R",
          click: () => mainWindow?.reload(),
        },
        {
          label: "Exit",
          accelerator: "Alt+F4",
          click: () => app.quit(),
        },
        { type: "separator" },
        {
          label: "Help",
          click: () => shell.openExternal("https://studenttechsupport.com/support"),
        },
        {
          label: "Privacy",
          click: () => shell.openExternal("https://studenttechsupport.com/privacy"),
        },
      ],
    },
    {
      label: "Debug",
      submenu: [
        {
          label: "Dev Tools",
          accelerator: "CmdOrCtrl+Shift+I",
          click: () => mainWindow?.webContents.toggleDevTools(),
        },
        {
          label: "Open AppData Folder",
          click: () => {
            void shell.openPath(app.getPath("userData"));
          },
        },
        {
          label: "Reload",
          accelerator: "CmdOrCtrl+R",
          click: () => mainWindow?.reload(),
        },
        {
          label: "Force Reload",
          accelerator: "CmdOrCtrl+Shift+R",
          click: () => mainWindow?.webContents.reloadIgnoringCache(),
        },
        { type: "separator" },
        {
          label: "Reset and Clear All (Quit)",
          click: () => {
            void resetAppDataAndQuit();
          },
        },
        { type: "separator" },
        {
          label: "Actual Size",
          accelerator: "CmdOrCtrl+0",
          click: () => mainWindow?.webContents.setZoomLevel(0),
        },
      ],
    },
  ];

  Menu.setApplicationMenu(Menu.buildFromTemplate(template));
}

app.whenReady().then(async () => {
  await ensureRepoState();
  await loadSerialPrefs();
  await loadUiPrefs();
  createWindow();
  buildMenu();
});

app.on("window-all-closed", () => {
  if (process.platform !== "darwin") app.quit();
});

ipcMain.handle("app:getVersion", () => {
  return app.getVersion();
});

ipcMain.handle("navigate", (_event, page) => {
  mainWindow.loadFile(path.join(__dirname, "pages", page));
});

ipcMain.handle("go-back", () => {
  mainWindow.loadFile(path.join(__dirname, "pages", "home.html"));
});

ipcMain.handle("repo:getState", async () => {
  const state = await ensureRepoState();
  return state;
});

ipcMain.handle("repo:getActive", async () => {
  const state = await ensureRepoState();
  const active = state.repos.find((r) => r.id === state.activeRepoId) || null;
  if (!active) return null;
  return { ...active, cacheBaseUrl: cacheBaseUrl(active.id) };
});

ipcMain.handle("repo:setActive", async (_event, id) => {
  const state = await ensureRepoState();
  if (!state.repos.find((r) => r.id === id)) return false;
  state.activeRepoId = id;
  await saveRepoState(state);
  return true;
});

ipcMain.handle("repo:add", async (_event, payload) => {
  const baseUrl = normalizeBaseUrl(payload.baseUrl);
  const name = String(payload.name || "").trim();
  const state = await ensureRepoState();
  const existing = state.repos.find((r) => r.baseUrl === baseUrl);
  if (existing) return { ok: true, repo: existing };

  const id = repoIdFor(baseUrl);
  const repo = {
    id,
    name: name || baseUrl,
    baseUrl,
    lastFetchedAt: "",
    lastHash: "",
    lastCheckedAt: "",
  };
  state.repos.push(repo);
  if (!state.activeRepoId) state.activeRepoId = id;
  await saveRepoState(state);
  return { ok: true, repo };
});

ipcMain.handle("repo:remove", async (_event, id) => {
  const state = await ensureRepoState();
  const next = state.repos.filter((r) => r.id !== id);
  if (next.length === state.repos.length) return false;
  state.repos = next;
  if (state.activeRepoId === id) {
    state.activeRepoId = state.repos[0]?.id || "";
  }
  await saveRepoState(state);
  return true;
});

ipcMain.handle("repo:checkForUpdate", async (_event, id) => {
  const state = await ensureRepoState();
  const repo = state.repos.find((r) => r.id === id);
  if (!repo) return { changed: false, error: "Repo not found" };

  const listUrl = new URL("firmwarelist.json", repo.baseUrl).toString();
  try {
    const { hash } = await fetchTextWithHash(listUrl);
    let cachedHash = repo.lastHash;
    if (!cachedHash) {
      const cachedPath = path.join(repoCacheDir(repo.id), "firmwarelist.json");
      try {
        cachedHash = await hashFile(cachedPath);
      } catch {
        cachedHash = "";
      }
    }
    repo.lastCheckedAt = new Date().toISOString();
    await saveRepoState(state);
    return { changed: Boolean(cachedHash && hash && cachedHash !== hash) };
  } catch (err) {
    return { changed: false, error: String(err) };
  }
});

ipcMain.handle("repo:getCachedFirmwareList", async (_event, id) => {
  const state = await ensureRepoState();
  const repo = state.repos.find((r) => r.id === id);
  if (!repo) return null;
  const filePath = path.join(repoCacheDir(repo.id), "firmwarelist.json");
  try {
    const raw = await fsp.readFile(filePath, "utf8");
    const parsed = JSON.parse(raw);
    return {
      repo,
      cacheBaseUrl: cacheBaseUrl(repo.id),
      data: parsed,
    };
  } catch {
    return {
      repo,
      cacheBaseUrl: cacheBaseUrl(repo.id),
      data: null,
    };
  }
});

ipcMain.handle("repo:refresh", async (_event, id) => {
  const state = await ensureRepoState();
  const repo = state.repos.find((r) => r.id === id);
  if (!repo) return { ok: false, error: "Repo not found" };

  const repoDir = repoCacheDir(repo.id);
  await fsp.mkdir(repoDir, { recursive: true });

  try {
    const listUrl = new URL("firmwarelist.json", repo.baseUrl).toString();
    emitRepoFetchLog(`GET ${listUrl}`);
    const { text, hash } = await fetchTextWithHash(listUrl);
    await fsp.writeFile(path.join(repoDir, "firmwarelist.json"), text);

    const parsed = JSON.parse(text);
    const firmwareDefs = Array.isArray(parsed.firmwares) ? parsed.firmwares : [];
    const errors = await cacheRepoAssets(repo, repoDir, firmwareDefs);

    repo.lastFetchedAt = new Date().toISOString();
    repo.lastHash = hash;
    await saveRepoState(state);

    return { ok: true, errors };
  } catch (err) {
    return { ok: false, error: String(err) };
  }
});

ipcMain.handle("repo:clearCache", async (_event, id) => {
  const state = await ensureRepoState();
  const repo = state.repos.find((r) => r.id === id);
  if (!repo) return { ok: false, error: "Repo not found" };
  await clearRepoCache(repo.id);
  return { ok: true };
});

ipcMain.handle("repo:clearAllCaches", async () => {
  await clearAllRepoCaches();
  return { ok: true };
});

ipcMain.handle("serial:listPorts", async () => {
  if (SerialPort?.list) {
    const ports = await SerialPort.list();
    return ports.map((port) => ({
      path: port.path || "",
      serialNumber: port.serialNumber || "",
      vendorId: port.vendorId || "",
      productId: port.productId || "",
      manufacturer: port.manufacturer || "",
      friendlyName: port.friendlyName || "",
      pnpId: port.pnpId || "",
    }));
  }
  return [];
});

ipcMain.handle("serial:getPreferred", async () => {
  return serialPrefs.preferred || null;
});

ipcMain.handle("serial:setPreferred", async (_event, port) => {
  serialPrefs.preferred = port || null;
  await saveSerialPrefs();
  return { ok: true };
});

ipcMain.handle("serial:clearPreferred", async () => {
  serialPrefs.preferred = null;
  await saveSerialPrefs();
  return { ok: true };
});
ipcMain.handle("ui:getPrefs", async () => {
  return uiPrefs;
});

ipcMain.handle("ui:setAutoHideMenuBar", async (_event, value) => {
  uiPrefs.autoHideMenuBar = Boolean(value);
  await saveUiPrefs();
  if (mainWindow) {
    mainWindow.setAutoHideMenuBar(uiPrefs.autoHideMenuBar);
    mainWindow.setMenuBarVisibility(!uiPrefs.autoHideMenuBar);
  }
  return { ok: true };
});
ipcMain.handle("smartcard:getStatus", async () => {
  if (!Context) {
    return {
      available: false,
      readers: [],
      uid: "",
      lastError: smartcardLoadError || "smartcard module not available.",
    };
  }

  const ctx = new Context();
  if (!ctx.isValid) {
    return {
      available: false,
      readers: [],
      uid: "",
      lastError: "PC/SC context invalid.",
    };
  }

  let uid = "";
  let lastError = "";
  let readerNames = [];
  try {
    const readers = ctx.listReaders();
    readerNames = readers.map((r) => r.name);
    const reader = pickReader(readers);
    if (reader) {
      try {
        const card = await reader.connect(
          SCARD_SHARE_SHARED,
          SCARD_PROTOCOL_T0 | SCARD_PROTOCOL_T1
        );
        const { data, sw1, sw2 } = await transmit(card, apduGetUid());
        if (sw1 === 0x90 && sw2 === 0x00) {
          uid = data.toString("hex").toUpperCase();
        }
        card.disconnect(SCARD_LEAVE_CARD);
      } catch (err) {
        lastError = String(err);
      }
    }
  } catch (err) {
    lastError = String(err);
  } finally {
    ctx.close();
  }

  return {
    available: true,
    readers: readerNames,
    uid,
    lastError,
  };
});

ipcMain.handle("smartcard:readNdefText", async (_event, readerHint = "") => {
  return await withCard(readerHint, async (card) => {
    await loadKeys(card);
    const raw = await readUserArea(card);
    const { offset, length } = findNdefValue(raw);
    const value = raw.slice(offset, offset + length);
    const { tnf, type, payload } = parseFirstRecord(value);
    if (tnf === 0x01 && type.equals(Buffer.from("T"))) {
      return { ok: true, text: decodeTextRecordPayload(payload) };
    }
    if (tnf === 0x02 && type.equals(Buffer.from("application/json"))) {
      return { ok: true, text: payload.toString("utf8") };
    }
    return { ok: false, error: "Unsupported NDEF record type." };
  });
});

ipcMain.handle("smartcard:writeNdefText", async (_event, payload, readerHint = "") => {
  const text = String(payload ?? "");
  return await withCard(readerHint, async (card) => {
    await loadKeys(card);
    const ndef = buildTextRecord(text, "en");
    const tlv = wrapTlv(ndef);
    await writeUserAreaWithTlv(card, tlv);
    return { ok: true };
  });
});

ipcMain.handle("smartcard:prepareNewCard", async (_event, readerHint = "") => {
  return await withCard(readerHint, async (card) => {
    for (let sector = 0; sector < 16; sector += 1) {
      await wipeSector(card, sector);
    }
    await rebuildMad(card);
    await writeNdefReady(card);
    return { ok: true };
  });
});

ipcMain.handle("bestjson:setPayload", async (_event, payload) => {
  bestJsonPayload = String(payload ?? "");
  return { ok: true };
});

ipcMain.handle("bestjson:getPayload", async () => {
  return { ok: true, payload: bestJsonPayload };
});
ipcMain.handle("shell:openExternal", async (_event, url) => {
  if (!url) return { ok: false };
  const textUrl = String(url);
  if (!/^https?:/i.test(textUrl)) return { ok: false };
  await shell.openExternal(textUrl);
  return { ok: true };
});
