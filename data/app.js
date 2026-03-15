const state = {
  wsConnected: false,
  mode: 'auto',
  manual_mask: 0,
  ramp_speed: 20,
  pump_timeout_s: 10,
  lastPayload: null,
};

const els = {
  wsBadge: document.getElementById('wsBadge'),
  onlineBadge: document.getElementById('onlineBadge'),
  modeValue: document.getElementById('modeValue'),
  manualMaskValue: document.getElementById('manualMaskValue'),
  pumpMaskValue: document.getElementById('pumpMaskValue'),
  tankValue: document.getElementById('tankValue'),
  modeToggleBtn: document.getElementById('modeToggleBtn'),
  temp: [0, 1, 2].map((i) => document.getElementById(`temp${i}`)),
  soilBars: [0, 1, 2, 3, 4].map((i) => document.getElementById(`soilBar${i}`)),
  soilVals: [0, 1, 2, 3, 4].map((i) => document.getElementById(`soilVal${i}`)),
  manualButtons: [
    ...document.querySelectorAll('#manualButtons [data-pump]'),
  ],
  thresholds: [0, 1, 2].map((i) => document.getElementById(`threshold${i}`)),
  thresholdVals: [0, 1, 2].map((i) => document.getElementById(`thresholdVal${i}`)),
  maxPwm: [0, 1, 2].map((i) => document.getElementById(`maxPwm${i}`)),
  maxPwmVals: [0, 1, 2].map((i) => document.getElementById(`maxPwmVal${i}`)),
  rampSpeed: document.getElementById('rampSpeed'),
  rampSpeedVal: document.getElementById('rampSpeedVal'),
  pumpTimeoutS: document.getElementById('pumpTimeoutS'),
  pumpTimeoutVal: document.getElementById('pumpTimeoutVal'),
  pumpCurrent: [0, 1, 2].map((i) => document.getElementById(`pumpCurrent${i}`)),
  pumpTarget: [0, 1, 2].map((i) => document.getElementById(`pumpTarget${i}`)),
  pumpLive: [...document.querySelectorAll('.pump-live')],
};

function clamp(v, min, max) {
  return Math.min(max, Math.max(min, v));
}

function moistPercent(raw) {
  const bounded = clamp(Number(raw) || 0, 0, 4095);
  return Math.round((1 - bounded / 4095) * 100);
}

function fmtTemp(v) {
  if (typeof v !== 'number' || Number.isNaN(v)) return '--.- °C';
  return `${v.toFixed(1)} °C`;
}

function fmtTimeoutSec(v) {
  const n = Number(v) || 0;
  return n === 0 ? '0 (Unendlich)' : `${n} s`;
}

async function postConfig(payload) {
  try {
    await fetch('/api/config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
  } catch (e) {
    console.error('POST /api/config failed', e);
  }
}

function setBadge(el, text, ok) {
  el.textContent = text;
  el.classList.toggle('ok', !!ok);
  el.classList.toggle('err', !ok);
}

function applyModeUi() {
  const isAuto = state.mode === 'auto';
  els.modeToggleBtn.textContent = isAuto ? 'Auto Mode aktiv (klicken für Manual)' : 'Manual Mode aktiv (klicken für Auto)';
  els.modeToggleBtn.classList.toggle('btn-active', !isAuto);

  els.manualButtons.forEach((btn, idx) => {
    btn.disabled = isAuto;
    const isOn = (state.manual_mask & (1 << idx)) !== 0;
    btn.classList.toggle('btn-active', !isAuto && isOn);
    btn.textContent = `Pumpe ${idx + 1} ${isOn ? 'AN' : 'AUS'}`;
  });

  els.thresholds.forEach((slider) => {
    slider.disabled = !isAuto;
  });
}

function updateSliderValue(slider, value) {
  if (document.activeElement !== slider) {
    slider.value = value;
  }
}

function updateFromPayload(payload) {
  state.lastPayload = payload;
  state.mode = payload.mode || state.mode;
  state.manual_mask = Number(payload.manual_mask || 0);
  if (payload.ramp_speed !== undefined) state.ramp_speed = Number(payload.ramp_speed || 1);
  if (payload.pump_timeout_s !== undefined) state.pump_timeout_s = Number(payload.pump_timeout_s || 0);

  setBadge(els.onlineBadge, payload.online ? 'System: Online' : 'System: Offline', !!payload.online);
  els.modeValue.textContent = state.mode.toUpperCase();
  els.manualMaskValue.textContent = `0x${state.manual_mask.toString(16).padStart(2, '0')}`;
  els.pumpMaskValue.textContent = `0x${Number(payload.pump_mask || 0).toString(16).padStart(2, '0')}`;
  els.tankValue.textContent = payload.sensor?.tank_full ? 'Ja' : 'Nein';

  const t = payload.temperature_c || {};
  els.temp[0].textContent = fmtTemp(t.t0);
  els.temp[1].textContent = fmtTemp(t.t1);
  els.temp[2].textContent = fmtTemp(t.t2);

  const hw = payload.sensor?.hw390_raw || [];
  for (let i = 0; i < 5; i += 1) {
    const raw = Number(hw[i] || 0);
    const pct = moistPercent(raw);
    els.soilBars[i].style.width = `${pct}%`;
    els.soilVals[i].textContent = `${raw} (${pct}%)`;
  }

  const pumps = payload.pumps || [];
  const thresholds = [];
  const maxPwm = [];
  for (let i = 0; i < 3; i += 1) {
    const pump = pumps[i] || {};
    const current = Number(pump.current_pwm || 0);
    const target = Number(pump.target_pwm || 0);
    thresholds.push(Number(pump.threshold || 0));
    maxPwm.push(Number(pump.max_pwm_value || 0));

    els.pumpCurrent[i].textContent = `Ist: ${current}`;
    els.pumpTarget[i].textContent = `Soll: ${target}`;
    els.pumpLive[i].classList.toggle('active', current > 0 || target > 0);

    updateSliderValue(els.thresholds[i], thresholds[i]);
    els.thresholdVals[i].textContent = String(thresholds[i]);

    updateSliderValue(els.maxPwm[i], maxPwm[i]);
    els.maxPwmVals[i].textContent = String(maxPwm[i]);
  }

  updateSliderValue(els.rampSpeed, state.ramp_speed);
  els.rampSpeedVal.textContent = String(state.ramp_speed);

  updateSliderValue(els.pumpTimeoutS, state.pump_timeout_s);
  els.pumpTimeoutVal.textContent = fmtTimeoutSec(state.pump_timeout_s);

  applyModeUi();
}

function setupControls() {
  els.modeToggleBtn.addEventListener('click', () => {
    const nextMode = state.mode === 'auto' ? 'manual' : 'auto';
    state.mode = nextMode;
    if (nextMode === 'auto') {
      state.manual_mask = 0;
    }
    applyModeUi();
    postConfig({ mode: nextMode, manual_mask: state.manual_mask });
  });

  els.manualButtons.forEach((btn) => {
    btn.addEventListener('click', () => {
      if (state.mode !== 'manual') return;
      const idx = Number(btn.dataset.pump);
      state.manual_mask ^= 1 << idx;
      applyModeUi();
      postConfig({ manual_mask: state.manual_mask });
    });
  });

  els.thresholds.forEach((slider, idx) => {
    slider.addEventListener('input', () => {
      const value = Number(slider.value);
      els.thresholdVals[idx].textContent = String(value);
      const payload = {
        pump_thresholds: els.thresholds.map((s) => Number(s.value)),
      };
      postConfig(payload);
    });
  });

  els.maxPwm.forEach((slider, idx) => {
    slider.addEventListener('input', () => {
      const value = Number(slider.value);
      els.maxPwmVals[idx].textContent = String(value);
      const payload = {
        max_pwm_value: els.maxPwm.map((s) => Number(s.value)),
      };
      postConfig(payload);
    });
  });

  els.rampSpeed.addEventListener('input', () => {
    const value = Number(els.rampSpeed.value);
    state.ramp_speed = value;
    els.rampSpeedVal.textContent = String(value);
    postConfig({ ramp_speed: value });
  });

  els.pumpTimeoutS.addEventListener('input', () => {
    const value = Number(els.pumpTimeoutS.value);
    state.pump_timeout_s = value;
    els.pumpTimeoutVal.textContent = fmtTimeoutSec(value);
    postConfig({ pump_timeout_s: value });
  });

  els.rampSpeed.value = state.ramp_speed;
  els.rampSpeedVal.textContent = String(state.ramp_speed);
  els.pumpTimeoutS.value = state.pump_timeout_s;
  els.pumpTimeoutVal.textContent = fmtTimeoutSec(state.pump_timeout_s);
}

function connectWebSocket() {
  const url = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`;
  const ws = new WebSocket(url);

  ws.addEventListener('open', () => {
    state.wsConnected = true;
    setBadge(els.wsBadge, 'WS: Verbunden', true);
  });

  ws.addEventListener('message', (ev) => {
    try {
      const payload = JSON.parse(ev.data);
      updateFromPayload(payload);
    } catch (err) {
      console.error('Invalid WS payload', err);
    }
  });

  ws.addEventListener('close', () => {
    state.wsConnected = false;
    setBadge(els.wsBadge, 'WS: Getrennt', false);
    setTimeout(connectWebSocket, 1500);
  });

  ws.addEventListener('error', () => {
    setBadge(els.wsBadge, 'WS: Fehler', false);
    ws.close();
  });
}

setupControls();
connectWebSocket();
applyModeUi();
