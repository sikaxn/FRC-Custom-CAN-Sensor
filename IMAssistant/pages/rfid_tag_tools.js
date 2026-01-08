const btnBack = document.getElementById("btnBack");
const btnReaderStatus = document.getElementById("btnReaderStatus");
const btnCopyUid = document.getElementById("btnCopyUid");
const btnReadNdef = document.getElementById("btnReadNdef");
const btnWriteNdef = document.getElementById("btnWriteNdef");
const btnPrepareCard = document.getElementById("btnPrepareCard");
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
];

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
