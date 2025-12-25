(() => {
  const isAllowedTarget = (target) => {
    if (!target || !target.closest) return false;
    if (target.closest(".terminal")) return true;
    if (target.closest("[contenteditable='true'],[contenteditable='']")) return true;
    const tag = target.tagName;
    if (!tag) return false;
    return tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT";
  };

  const blockIfNeeded = (event) => {
    if (isAllowedTarget(event.target)) return;
    event.preventDefault();
  };

  document.addEventListener("copy", blockIfNeeded);
  document.addEventListener("cut", blockIfNeeded);
  document.addEventListener("paste", blockIfNeeded);
  document.addEventListener("selectstart", blockIfNeeded);
})();
