const { contextBridge, ipcRenderer } = require("electron");

contextBridge.exposeInMainWorld("nav", {
  go: (page) => ipcRenderer.invoke("navigate", page),
  back: () => ipcRenderer.invoke("go-back"),
});
