const { app, BrowserWindow, ipcMain, shell } = require("electron");
const path = require("path");
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

let mainWindow;
let serialPrefs = { preferred: null };

const DEFAULT_REPO_URL = "https://studenttechsupport.com/customcanespfw/";
const REPO_STATE_FILE = () => path.join(app.getPath("userData"), "repos.json");
const CACHE_ROOT = () => path.join(app.getPath("userData"), "repo-cache");
const SERIAL_PREFS_FILE = () =>
  path.join(app.getPath("userData"), "serial.json");

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

async function clearRepoCache(repoId) {
  const dir = repoCacheDir(repoId);
  await fsp.rm(dir, { recursive: true, force: true });
}

async function clearAllRepoCaches() {
  await fsp.rm(CACHE_ROOT(), { recursive: true, force: true });
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
    autoHideMenuBar: true,      // hide toolbar on Windows
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

app.whenReady().then(async () => {
  await ensureRepoState();
  await loadSerialPrefs();
  createWindow();
});

app.on("window-all-closed", () => {
  if (process.platform !== "darwin") app.quit();
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
ipcMain.handle("shell:openExternal", async (_event, url) => {
  if (!url) return { ok: false };
  const textUrl = String(url);
  if (!/^https?:/i.test(textUrl)) return { ok: false };
  await shell.openExternal(textUrl);
  return { ok: true };
});
