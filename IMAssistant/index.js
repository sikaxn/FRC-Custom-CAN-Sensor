const { app, BrowserWindow, ipcMain } = require("electron");
const path = require("path");

let mainWindow;

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 900,
    height: 650,
    //autoHideMenuBar: true,      // hide toolbar on Windows
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

    // Simple default: pick the first available port.
    // TODO: replace with your own UI, or filter by vendorId/productId/serialNumber.
    const selected = portList[0];

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

app.whenReady().then(createWindow);

app.on("window-all-closed", () => {
  if (process.platform !== "darwin") app.quit();
});

ipcMain.handle("navigate", (_event, page) => {
  mainWindow.loadFile(path.join(__dirname, "pages", page));
});

ipcMain.handle("go-back", () => {
  mainWindow.loadFile(path.join(__dirname, "pages", "home.html"));
});
