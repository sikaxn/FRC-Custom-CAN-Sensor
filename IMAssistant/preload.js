const { contextBridge, ipcRenderer } = require("electron");

contextBridge.exposeInMainWorld("app", {
  getVersion: () => ipcRenderer.invoke("app:getVersion"),
});


contextBridge.exposeInMainWorld("nav", {
  go: (page) => ipcRenderer.invoke("navigate", page),
  back: () => ipcRenderer.invoke("go-back"),
});

contextBridge.exposeInMainWorld("repo", {
  getState: () => ipcRenderer.invoke("repo:getState"),
  getActive: () => ipcRenderer.invoke("repo:getActive"),
  setActive: (id) => ipcRenderer.invoke("repo:setActive", id),
  add: (baseUrl, name) => ipcRenderer.invoke("repo:add", { baseUrl, name }),
  remove: (id) => ipcRenderer.invoke("repo:remove", id),
  refresh: (id) => ipcRenderer.invoke("repo:refresh", id),
  checkForUpdate: (id) => ipcRenderer.invoke("repo:checkForUpdate", id),
  clearCache: (id) => ipcRenderer.invoke("repo:clearCache", id),
  clearAllCaches: () => ipcRenderer.invoke("repo:clearAllCaches"),
  getCachedFirmwareList: (id) =>
    ipcRenderer.invoke("repo:getCachedFirmwareList", id),
  onFetchLog: (handler) => {
    const listener = (_event, message) => handler(message);
    ipcRenderer.on("repo:fetchLog", listener);
    return () => ipcRenderer.removeListener("repo:fetchLog", listener);
  },
});

contextBridge.exposeInMainWorld("serial", {
  listPorts: () => ipcRenderer.invoke("serial:listPorts"),
  getPreferred: () => ipcRenderer.invoke("serial:getPreferred"),
  setPreferred: (port) => ipcRenderer.invoke("serial:setPreferred", port),
  clearPreferred: () => ipcRenderer.invoke("serial:clearPreferred"),
});

contextBridge.exposeInMainWorld("ui", {
  getPrefs: () => ipcRenderer.invoke("ui:getPrefs"),
  setAutoHideMenuBar: (value) =>
    ipcRenderer.invoke("ui:setAutoHideMenuBar", value),
});

contextBridge.exposeInMainWorld("smartcard", {
  getStatus: () => ipcRenderer.invoke("smartcard:getStatus"),
  readNdefText: (readerHint) => ipcRenderer.invoke("smartcard:readNdefText", readerHint),
  writeNdefText: (text, readerHint) =>
    ipcRenderer.invoke("smartcard:writeNdefText", text, readerHint),
  prepareNewCard: (readerHint) =>
    ipcRenderer.invoke("smartcard:prepareNewCard", readerHint),
});

contextBridge.exposeInMainWorld("bestjson", {
  setPayload: (payload) => ipcRenderer.invoke("bestjson:setPayload", payload),
  getPayload: () => ipcRenderer.invoke("bestjson:getPayload"),
});

contextBridge.exposeInMainWorld("log", {
  append: (payload) => ipcRenderer.invoke("log:append", payload),
  openFolder: () => ipcRenderer.invoke("log:openFolder"),
});

contextBridge.exposeInMainWorld("printWindow", {
  open: (html) => ipcRenderer.invoke("print:open", { html }),
});

contextBridge.exposeInMainWorld("shell", {
  openExternal: (url) => ipcRenderer.invoke("shell:openExternal", url),
});

window.addEventListener("DOMContentLoaded", () => {
  document.addEventListener("click", (event) => {
    const link = event.target.closest("a");
    if (!link) return;
    const href = link.getAttribute("href");
    if (!href) return;
    let url;
    try {
      url = new URL(href, window.location.href);
    } catch {
      return;
    }
    if (url.protocol === "http:" || url.protocol === "https:") {
      event.preventDefault();
      ipcRenderer.invoke("shell:openExternal", url.toString());
    }
  });
});
