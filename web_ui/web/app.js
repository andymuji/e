const grid = document.querySelector("#location-grid");
const connection = document.querySelector("#connection");
const statusLabel = document.querySelector("#safety-label");
const statusMessage = document.querySelector("#status-message");
const statusMark = document.querySelector("#status-mark");
const cancelButton = document.querySelector("#cancel-button");
const decisionValue = document.querySelector("#decision-value");
const speedValue = document.querySelector("#speed-value");
const distanceValue = document.querySelector("#distance-value");
const ageValue = document.querySelector("#age-value");
const mapViewport = document.querySelector("#map-viewport");
const floorMap = document.querySelector("#floor-map");
const labelForm = document.querySelector("#label-form");
const labelInput = document.querySelector("#location-label");
const labelPosition = document.querySelector("#label-position");
let mapData;
let pendingLabel;
let view = { scale: 1, x: 0, y: 0 };
const pointers = new Map();
let lastPinchDistance;
const svgNs = "http://www.w3.org/2000/svg";

async function request(path, options = {}) {
  const response = await fetch(path, options);
  const body = await response.json();
  if (!response.ok) throw new Error(body.error || "The robot could not complete that request.");
  return body;
}

function setBusy(busy) {
  document.querySelectorAll(".location-card, .map-location").forEach((button) => { button.classList.toggle("is-busy", busy); });
}

function showError(error) {
  statusMark.textContent = "!";
  statusLabel.textContent = "Please try again";
  statusMessage.textContent = error.message;
}

function renderStatus(data) {
  const goal = data.goal;
  const safety = data.safety;
  connection.classList.add("online");
  connection.lastChild.textContent = " Connected to robot";
  statusLabel.textContent = data.safety_message;
  statusMark.textContent = data.safety_state === "clear" ? "OK" : "!";
  statusMessage.textContent = safety.reason;
  cancelButton.hidden = goal.state !== "navigating";
  decisionValue.textContent = safety.state.toUpperCase();
  speedValue.textContent = `${Math.round(safety.speed_scale * 100)}%`;
  distanceValue.textContent = safety.nearest_obstacle_distance === null ? "invalid" : `${safety.nearest_obstacle_distance.toFixed(1)} m`;
  ageValue.textContent = `${safety.reading_age.toFixed(1)} s`;
  document.querySelectorAll(".scenario-card").forEach((button) => {
    button.classList.toggle("active", button.dataset.scenario === safety.scenario);
  });
}

async function refreshStatus() {
  try { renderStatus(await request("/api/status")); }
  catch (error) { connection.classList.remove("online"); showError(error); }
}

async function chooseLocation(name, button) {
  setBusy(true);
  const goLabel = button.querySelector(".go-label");
  if (goLabel) goLabel.textContent = "Sending...";
  try {
    await request("/api/goals", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ location_name: name }) });
    await refreshStatus();
  } catch (error) { showError(error); }
  finally { setBusy(false); if (goLabel) goLabel.textContent = "Go here"; }
}

async function loadLocations() {
  try {
    const data = await request("/api/locations");
    grid.replaceChildren(...data.locations.map(({ name }) => {
      const button = document.createElement("button");
      button.className = "location-card";
      button.type = "button";
      button.innerHTML = `<span class="location-name"></span><span class="go-label">Go here</span>`;
      button.querySelector(".location-name").textContent = name;
      button.addEventListener("click", () => chooseLocation(name, button));
      return button;
    }));
    await refreshStatus();
  } catch (error) { showError(error); grid.innerHTML = "<p class='loading'>Destinations are unavailable right now.</p>"; }
}

function svgElement(name, attributes = {}) {
  const element = document.createElementNS(svgNs, name);
  Object.entries(attributes).forEach(([key, value]) => element.setAttribute(key, value));
  return element;
}

function toScreen(point) {
  const bounds = mapData.bounds;
  return { x: ((point.x - bounds.min_x) / (bounds.max_x - bounds.min_x)) * 600, y: 500 - ((point.y - bounds.min_y) / (bounds.max_y - bounds.min_y)) * 500 };
}

function toMapPoint(clientX, clientY) {
  const rect = floorMap.getBoundingClientRect();
  const sx = (clientX - rect.left) * 600 / rect.width;
  const sy = (clientY - rect.top) * 500 / rect.height;
  const bounds = mapData.bounds;
  return {
    x: bounds.min_x + (sx / 600) * (bounds.max_x - bounds.min_x),
    y: bounds.min_y + ((500 - sy) / 500) * (bounds.max_y - bounds.min_y),
  };
}

function updateMapTransform() {
  floorMap.style.transform = `translate(${view.x}px, ${view.y}px) scale(${view.scale})`;
}

function renderMap() {
  floorMap.replaceChildren();
  const background = svgElement("rect", { x: 0, y: 0, width: 600, height: 500, class: "map-background" });
  floorMap.append(background);
  mapData.walls.forEach((wall) => {
    const points = wall.map(([x, y]) => { const screen = toScreen({ x, y }); return `${screen.x},${screen.y}`; }).join(" ");
    floorMap.append(svgElement("polyline", { points, class: "map-wall" }));
  });
  const robot = toScreen(mapData.robot);
  floorMap.append(svgElement("circle", { cx: robot.x, cy: robot.y, r: 13, class: "robot-marker" }));
  mapData.locations.forEach((location) => {
    const point = toScreen(location);
    const group = svgElement("g", { class: "map-location", tabindex: 0, role: "button", "aria-label": `Go to ${location.name}` });
    group.append(svgElement("circle", { cx: point.x, cy: point.y, r: 12 }));
    const label = svgElement("text", { x: point.x + 17, y: point.y + 5 });
    label.textContent = location.name;
    group.append(label);
    group.addEventListener("click", (event) => { event.stopPropagation(); chooseLocation(location.name, group); });
    group.addEventListener("keydown", (event) => { if (event.key === "Enter" || event.key === " ") { event.preventDefault(); chooseLocation(location.name, group); } });
    floorMap.append(group);
  });
  updateMapTransform();
}

async function loadMap() {
  mapData = await request("/api/map");
  renderMap();
}

function showLabelForm(point) {
  pendingLabel = point;
  labelPosition.textContent = `New location at ${point.x.toFixed(1)} m, ${point.y.toFixed(1)} m`;
  labelInput.value = "";
  labelForm.hidden = false;
  labelInput.focus();
}

floorMap.addEventListener("pointerdown", (event) => {
  floorMap.setPointerCapture(event.pointerId);
  pointers.set(event.pointerId, { x: event.clientX, y: event.clientY, startX: event.clientX, startY: event.clientY });
  if (pointers.size === 2) {
    const [first, second] = [...pointers.values()];
    lastPinchDistance = Math.hypot(first.x - second.x, first.y - second.y);
  }
});
floorMap.addEventListener("pointermove", (event) => {
  if (!pointers.has(event.pointerId)) return;
  const previous = pointers.get(event.pointerId);
  pointers.set(event.pointerId, { ...previous, x: event.clientX, y: event.clientY });
  if (pointers.size === 1) { view.x += event.clientX - previous.x; view.y += event.clientY - previous.y; updateMapTransform(); }
  if (pointers.size === 2) {
    const [first, second] = [...pointers.values()];
    const distance = Math.hypot(first.x - second.x, first.y - second.y);
    if (lastPinchDistance) { view.scale = Math.min(2.5, Math.max(0.8, view.scale * distance / lastPinchDistance)); updateMapTransform(); }
    lastPinchDistance = distance;
  }
});
floorMap.addEventListener("pointerup", (event) => {
  const start = pointers.get(event.pointerId);
  pointers.delete(event.pointerId);
  if (pointers.size < 2) lastPinchDistance = undefined;
  if (start && Math.hypot(event.clientX - start.startX, event.clientY - start.startY) < 8 && !pointers.size) showLabelForm(toMapPoint(event.clientX, event.clientY));
});
floorMap.addEventListener("pointercancel", (event) => { pointers.delete(event.pointerId); lastPinchDistance = undefined; });

document.querySelector("#zoom-in").addEventListener("click", () => { view.scale = Math.min(2.5, view.scale + 0.25); updateMapTransform(); });
document.querySelector("#zoom-out").addEventListener("click", () => { view.scale = Math.max(0.8, view.scale - 0.25); updateMapTransform(); });
document.querySelector("#reset-map").addEventListener("click", () => { view = { scale: 1, x: 0, y: 0 }; updateMapTransform(); });
document.querySelector("#discard-label").addEventListener("click", () => { labelForm.hidden = true; pendingLabel = undefined; });
labelForm.addEventListener("submit", async (event) => {
  event.preventDefault();
  if (!pendingLabel) return;
  const submit = labelForm.querySelector("button[type=submit]");
  submit.disabled = true;
  try {
    await request("/api/locations", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: labelInput.value, ...pendingLabel }) });
    labelForm.hidden = true;
    pendingLabel = undefined;
    await Promise.all([loadLocations(), loadMap()]);
  } catch (error) { showError(error); }
  finally { submit.disabled = false; }
});

cancelButton.addEventListener("click", async () => {
  cancelButton.disabled = true;
  try { await request("/api/goals/cancel", { method: "POST" }); await refreshStatus(); }
  catch (error) { showError(error); }
  finally { cancelButton.disabled = false; }
});

document.querySelectorAll(".scenario-card").forEach((button) => {
  button.addEventListener("click", async () => {
    button.disabled = true;
    try {
      await request("/api/safety/scenario", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ scenario: button.dataset.scenario }),
      });
      await refreshStatus();
    } catch (error) { showError(error); }
    finally { button.disabled = false; }
  });
});

Promise.all([loadLocations(), loadMap()]).catch(showError);
setInterval(refreshStatus, 3000);
