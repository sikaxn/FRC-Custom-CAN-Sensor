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
