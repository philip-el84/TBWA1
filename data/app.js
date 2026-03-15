const out = document.getElementById('telemetry');
const ws = new WebSocket(`ws://${location.host}/ws`);
ws.onmessage = (ev) => {
  try {
    const obj = JSON.parse(ev.data);
    out.textContent = JSON.stringify(obj, null, 2);
  } catch {
    out.textContent = ev.data;
  }
};
