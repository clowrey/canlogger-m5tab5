// Adds a link to the CAN capture file list (served by can_logger) above the
// ESPHome web_server page.
const a = document.createElement("a");
a.href = "/files";
a.textContent = "CAN capture files";
a.style.cssText = "display:block;padding:12px 16px;font:600 16px sans-serif";
document.body.prepend(a);
