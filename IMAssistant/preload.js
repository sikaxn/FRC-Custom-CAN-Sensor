const { contextBridge, ipcRenderer } = require("electron");

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
