const btnBack = document.getElementById("btnBack");

btnBack?.addEventListener("click", () => {
  if (window.nav?.back) {
    window.nav.back();
  } else {
    window.location.href = "home.html";
  }
});
