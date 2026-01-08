const btnBack = document.getElementById("btnBack");
const btnReaderStatus = document.getElementById("btnReaderStatus");
const btnCopyUid = document.getElementById("btnCopyUid");
const btnReadNdef = document.getElementById("btnReadNdef");
const btnWriteNdef = document.getElementById("btnWriteNdef");
const btnPrepareCard = document.getElementById("btnPrepareCard");
const btnOpenBatteryReader = document.getElementById("btnOpenBatteryReader");
const btnOpenBestJson = document.getElementById("btnOpenBestJson");
const btnOpenFile = document.getElementById("btnOpenFile");
const btnSaveFile = document.getElementById("btnSaveFile");
const readerStatus = document.getElementById("readerStatus");
const ndefText = document.getElementById("ndefText");
const ndefStatus = document.getElementById("ndefStatus");
const progressText = document.getElementById("progressText");
const prepareDialog = document.getElementById("prepareDialog");
const prepareConfirm = document.getElementById("prepareConfirm");
const prepareCancel = document.getElementById("prepareCancel");
let lastUid = "";

const actionButtons = [
  btnReaderStatus,
  btnCopyUid,
  btnReadNdef,
  btnWriteNdef,
  btnPrepareCard,
  btnOpenBatteryReader,
  btnOpenBestJson,
  btnOpenFile,
  btnSaveFile,
];

const openFileInput = document.createElement("input");
openFileInput.type = "file";
openFileInput.accept = ".json,.txt";

function setBusy(isBusy, message = "") {
  document.body.classList.toggle("busy", isBusy);
  actionButtons.forEach((btn) => {
    if (btn) btn.disabled = isBusy;
  });
  if (progressText) {
    progressText.textContent = isBusy ? message : "";
  }
}

function openPrepareDialog() {
  return new Promise((resolve) => {
    if (!prepareDialog || !prepareConfirm || !prepareCancel) {
      resolve(false);
      return;
    }
    prepareDialog.classList.add("open");

    const cleanup = () => {
      prepareDialog.classList.remove("open");
      prepareConfirm.removeEventListener("click", onConfirm);
      prepareCancel.removeEventListener("click", onCancel);
    };

    const onConfirm = () => {
      cleanup();
      resolve(true);
    };
    const onCancel = () => {
      cleanup();
      resolve(false);
    };

    prepareConfirm.addEventListener("click", onConfirm);
    prepareCancel.addEventListener("click", onCancel);
  });
}

btnBack?.addEventListener("click", () => {
  if (window.nav?.back) {
    window.nav.back();
  } else {
    window.location.href = "home.html";
  }
});

function formatStatus(result) {
  if (!result?.available) {
    const err = result?.lastError ? ` (${result.lastError})` : "";
    return `Smartcard not available${err}`;
  }
  const names = Array.isArray(result.readers) ? result.readers : [];
  const list = names.length ? names.join(", ") : "No readers detected.";
  const uid = result.uid ? ` UID: ${result.uid}` : " UID: not available.";
  const err = result.lastError ? ` Last error: ${result.lastError}` : "";
  lastUid = result.uid || "";
  if (btnCopyUid) {
    btnCopyUid.style.display = lastUid ? "inline-block" : "none";
  }
  return `Readers: ${list}.${uid}${err}`;
}

btnReaderStatus?.addEventListener("click", async () => {
  if (!window.smartcard?.getStatus) {
    if (readerStatus) readerStatus.textContent = "Smartcard API not available.";
    return;
  }
  setBusy(true, "Checking reader status...");
  try {
    const result = await window.smartcard.getStatus();
    if (readerStatus) readerStatus.textContent = formatStatus(result);
  } catch (err) {
    if (readerStatus) readerStatus.textContent = `Failed to read status: ${err}`;
  } finally {
    setBusy(false);
  }
});

btnCopyUid?.addEventListener("click", async () => {
  if (!lastUid) {
    if (readerStatus) readerStatus.textContent = "UID not available yet.";
    if (btnCopyUid) btnCopyUid.style.display = "none";
    return;
  }
  try {
    await navigator.clipboard.writeText(lastUid);
    if (readerStatus) readerStatus.textContent = `UID copied: ${lastUid}`;
  } catch (err) {
    if (readerStatus) readerStatus.textContent = `Copy failed: ${err}`;
  }
});

btnReadNdef?.addEventListener("click", async () => {
  if (!window.smartcard?.readNdefText) {
    if (ndefStatus) ndefStatus.textContent = "Smartcard API not available.";
    return;
  }
  if (ndefStatus) ndefStatus.textContent = "Reading...";
  setBusy(true, "Reading NDEF text...");
  try {
    const result = await window.smartcard.readNdefText();
    if (result?.ok) {
      if (ndefText) ndefText.value = result.text || "";
      if (ndefStatus) ndefStatus.textContent = "Read OK.";
    } else {
      if (ndefStatus) ndefStatus.textContent = result?.error || "Read failed.";
    }
  } catch (err) {
    if (ndefStatus) ndefStatus.textContent = `Read failed: ${err}`;
  } finally {
    setBusy(false);
  }
});

btnWriteNdef?.addEventListener("click", async () => {
  if (!window.smartcard?.writeNdefText) {
    if (ndefStatus) ndefStatus.textContent = "Smartcard API not available.";
    return;
  }
  const text = ndefText ? ndefText.value : "";
  if (ndefStatus) ndefStatus.textContent = "Writing...";
  setBusy(true, "Writing NDEF text...");
  try {
    const result = await window.smartcard.writeNdefText(text || "");
    if (result?.ok) {
      if (ndefStatus) ndefStatus.textContent = "Write OK.";
    } else {
      if (ndefStatus) ndefStatus.textContent = result?.error || "Write failed.";
    }
  } catch (err) {
    if (ndefStatus) ndefStatus.textContent = `Write failed: ${err}`;
  } finally {
    setBusy(false);
  }
});

btnPrepareCard?.addEventListener("click", async () => {
  if (!window.smartcard?.prepareNewCard) {
    if (ndefStatus) ndefStatus.textContent = "Smartcard API not available.";
    return;
  }
  const ok = await openPrepareDialog();
  if (!ok) return;
  if (ndefStatus) ndefStatus.textContent = "Preparing card...";
  setBusy(true, "Preparing new card...");
  try {
    const result = await window.smartcard.prepareNewCard();
    if (result?.ok) {
      if (ndefStatus) ndefStatus.textContent = "Card prepared.";
    } else {
      if (ndefStatus) ndefStatus.textContent = result?.error || "Prepare failed.";
    }
  } catch (err) {
    if (ndefStatus) ndefStatus.textContent = `Prepare failed: ${err}`;
  } finally {
    setBusy(false);
  }
});

btnOpenBestJson?.addEventListener("click", () => {
  const payload = ndefText ? ndefText.value : "";
  const encoded = encodeURIComponent(payload || "");
  const target = `best_json_editor.html?payload=${encoded}`;
  if (window.nav?.go && !target.includes("?")) {
    window.nav.go(target);
  } else {
    window.location.href = target;
  }
});

btnOpenBatteryReader?.addEventListener("click", () => {
  if (window.nav?.go) {
    window.nav.go("rfid_battery_tag_reader.html");
  } else {
    window.location.href = "rfid_battery_tag_reader.html";
  }
});

btnOpenFile?.addEventListener("click", () => {
  openFileInput.value = "";
  openFileInput.click();
});

openFileInput.addEventListener("change", (evt) => {
  const file = evt.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = (e) => {
    const text = String(e.target.result || "");
    if (ndefText) ndefText.value = text;
    if (ndefStatus) ndefStatus.textContent = `Loaded file: ${file.name}`;
  };
  reader.readAsText(file);
});

btnSaveFile?.addEventListener("click", () => {
  const text = ndefText ? ndefText.value : "";
  const blob = new Blob([text], { type: "text/plain" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = "ndef.json";
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
  if (ndefStatus) ndefStatus.textContent = "Saved NDEF text to ndef.json";
});

async function loadBestJsonPayload() {
  if (!window.bestjson?.getPayload) return;
  try {
    const result = await window.bestjson.getPayload();
    const payload = result?.payload || "";
    if (payload && ndefText && !ndefText.value) {
      ndefText.value = payload;
      if (ndefStatus) ndefStatus.textContent = "Loaded BEST JSON into NDEF text.";
    }
  } catch {}
}

loadBestJsonPayload();
