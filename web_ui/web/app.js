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
const commandAgeValue = document.querySelector("#command-age-value");
const latchValue = document.querySelector("#latch-value");
const voiceForm = document.querySelector("#voice-form");
const voiceInput = document.querySelector("#voice-input");
const voiceButton = voiceForm.querySelector("button");
const voiceResponse = document.querySelector("#voice-response");
const estopButton = document.querySelector("#estop-button");
const estopReset = document.querySelector("#estop-reset");
const locationForm = document.querySelector("#location-form");
const locationResponse = document.querySelector("#location-response");
const mapViewport = document.querySelector("#map-viewport");
const floorMap = document.querySelector("#floor-map");
const labelForm = document.querySelector("#label-form");
const labelInput = document.querySelector("#location-label");
const labelPosition = document.querySelector("#label-position");
const mapNotice = document.querySelector("#map-notice");
let mapData;
let pendingLabel;
let view = { scale: 1, x: 0, y: 0 };
const pointers = new Map();
let lastPinchDistance;
const svgNs = "http://www.w3.org/2000/svg";

async function request(path, options = {}) {
  const response = await fetch(path, options);
  // A reply that is not JSON at all (a proxy error page, a dropped body) used
  // to surface as a JSON parse error and lose the actual HTTP status.
  let body = null;
  try { body = await response.json(); } catch (error) { body = null; }
  if (!response.ok) {
    throw new Error((body && body.error) || `The robot could not complete that request (HTTP ${response.status}).`);
  }
  if (body === null) throw new Error("The robot sent a reply this console could not read.");
  return body;
}

function setBusy(busy) {
  document.querySelectorAll(".go-button").forEach((button) => { button.disabled = busy; });
  document.querySelectorAll(".location-card, .map-location").forEach((card) => { card.classList.toggle("is-busy", busy); });
}

function showError(error) {
  statusMark.textContent = "!";
  statusLabel.textContent = "Please try again";
  statusMessage.textContent = error.message;
}

function number(value, digits, unit) {
  return typeof value === "number" && Number.isFinite(value)
    ? `${value.toFixed(digits)}${unit}`
    : "unknown";
}

// The stop latch has three states here, not two. `null` means the console has
// not been told: before the first status read, and after any failed one.
// Showing "released" in that case is the one lie this page must never tell -
// it invites someone to approach a robot whose software stop is still latched.
// Only a status read that actually carried a boolean may say released.
function renderLatch(engaged) {
  if (engaged === null) {
    latchValue.textContent = "unknown";
    latchValue.classList.remove("engaged");
    latchValue.classList.add("unknown");
    // No "Reset stop" on an unknown latch: that control means "it is
    // engaged, release it", which is a claim we cannot make. The STOP button
    // stays available because engaging is always the safe direction. The
    // "stopped" styling is left as it was, so a latch last seen engaged does
    // not visually un-stop itself just because a poll failed.
    estopReset.hidden = true;
    estopButton.hidden = false;
    return;
  }
  latchValue.textContent = engaged ? "ENGAGED" : "released";
  latchValue.classList.toggle("engaged", engaged);
  latchValue.classList.remove("unknown");
  estopButton.hidden = engaged;
  estopReset.hidden = !engaged;
  document.body.classList.toggle("stopped", engaged);
}

function renderStatus(data) {
  const goal = data && data.goal;
  const safety = data && data.safety;
  if (!goal || !safety) throw new Error("The robot sent an unrecognised status.");

  // Latch first. Everything below it is a cosmetic field, and if one of them
  // throws on unexpected data the latch must already be correct rather than
  // left showing whatever the last render put there.
  renderLatch(typeof safety.emergency_stop === "boolean" ? safety.emergency_stop : null);

  connection.classList.add("online");
  connection.lastChild.textContent = " Connected to robot";
  statusLabel.textContent = data.safety_message || "";
  statusMark.textContent = data.safety_state === "clear" ? "OK" : "!";
  statusMessage.textContent = safety.reason || "";
  cancelButton.hidden = goal.state !== "navigating";
  decisionValue.textContent = String(safety.state || "unknown").toUpperCase();
  speedValue.textContent = typeof safety.speed_scale === "number" && Number.isFinite(safety.speed_scale)
    ? `${Math.round(safety.speed_scale * 100)}%`
    : "unknown";
  // null is the sensor reporting no usable reading, which is not the same as
  // the console not knowing.
  distanceValue.textContent = safety.nearest_obstacle_distance === null
    ? "invalid"
    : number(safety.nearest_obstacle_distance, 1, " m");
  ageValue.textContent = number(safety.reading_age, 1, " s");
  commandAgeValue.textContent = number(safety.command_age, 1, " s");
  document.querySelectorAll(".scenario-card").forEach((button) => {
    button.classList.toggle("active", button.dataset.scenario === safety.scenario);
  });
}

function showStatusUnavailable(error) {
  connection.classList.remove("online");
  connection.lastChild.textContent = " Robot unreachable";
  showError(error);
  renderLatch(null);
  decisionValue.textContent = "unknown";
  speedValue.textContent = "unknown";
  distanceValue.textContent = "unknown";
  ageValue.textContent = "unknown";
  commandAgeValue.textContent = "unknown";
}

const STATUS_POLL_MS = 3000;
const STATUS_STALE_MS = 10000;
const MAP_POLL_MS = 2000;
let polling = false;
let lastStatusAt = 0;

async function refreshStatus() {
  // A request that never settles must not queue more behind it, or a wedged
  // server turns into a backlog that keeps overwriting the panel out of order.
  if (polling) return;
  polling = true;
  try {
    renderStatus(await request("/api/status"));
    lastStatusAt = Date.now();
  } catch (error) { showStatusUnavailable(error); }
  finally { polling = false; }
}

// A fetch that hangs never rejects, so without this the panel would sit on its
// last good reading indefinitely - latch included - with nothing to say the
// robot stopped answering. Silence is not the same as "released".
function checkStatusFreshness() {
  if (Date.now() - lastStatusAt > STATUS_STALE_MS) {
    showStatusUnavailable(
      new Error("No safety status for over 10 seconds. Treat the robot's state as unknown."),
    );
  }
}

async function chooseLocation(name, button) {
  setBusy(true);
  const goLabel = button.querySelector && button.querySelector(".go-label");
  const setGoText = (text) => {
    if (goLabel) goLabel.textContent = text;
    else if (button instanceof HTMLButtonElement) button.textContent = text;
  };
  setGoText("Sending...");
  try {
    await request("/api/goals", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ location_name: name }) });
    await refreshStatus();
  } catch (error) { showError(error); }
  finally { setBusy(false); setGoText("Go here"); }
}

function renderLocations(locations) {
  if (!Array.isArray(locations) || !locations.length) {
    grid.replaceChildren(Object.assign(document.createElement("p"), {
      className: "loading",
      textContent: "No places saved yet. Add one below.",
    }));
    return;
  }
  grid.replaceChildren(...locations.map(({ name, x, y }) => {
    const card = document.createElement("div");
    card.className = "location-card";

    const label = document.createElement("span");
    label.className = "location-name";
    label.textContent = name;

    const coords = document.createElement("span");
    coords.className = "location-coords";
    coords.textContent = `${number(x, 2, "")}, ${number(y, 2, "")}`;

    const go = document.createElement("button");
    go.className = "go-button";
    go.type = "button";
    go.textContent = "Go here";
    go.addEventListener("click", () => chooseLocation(name, go));

    const remove = document.createElement("button");
    remove.className = "remove-button";
    remove.type = "button";
    remove.textContent = "Remove";
    remove.setAttribute("aria-label", `Remove ${name}`);
    remove.addEventListener("click", () => removeLocation(name));

    const actions = document.createElement("div");
    actions.className = "location-actions";
    actions.append(go, remove);

    const heading = document.createElement("div");
    heading.className = "location-heading";
    heading.append(label, coords);

    card.append(heading, actions);
    return card;
  }));
}

async function loadLocations() {
  try {
    const data = await request("/api/locations");
    renderLocations(data.locations);
  } catch (error) {
    // Reported against the destination list, not the safety panel: a failed
    // locations fetch says nothing about the state of the safety gate, and
    // overwriting the panel with it hid whether the robot was stopped.
    locationResponse.textContent = error.message;
    renderLocations([]);
    grid.firstChild.textContent = "Destinations are unavailable right now.";
  }
  // Always, so a locations failure still leaves the safety panel truthful.
  await refreshStatus();
}

async function removeLocation(name) {
  if (!window.confirm(`Remove "${name}"? The robot will no longer accept it as a destination.`)) return;
  try {
    const data = await request("/api/locations/remove", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name }),
    });
    renderLocations(data.locations);
    locationResponse.textContent = `Removed ${name}.`;
  } catch (error) { locationResponse.textContent = error.message; }
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

// Every part of the map is optional. Connected to a real robot the console
// may have an occupancy grid and no pose, a pose and no grid, or neither
// while SLAM is still starting. A missing piece is drawn as missing; none of
// them may be invented, because an operator reads this picture as where the
// robot is.
function renderMap() {
  floorMap.replaceChildren();
  const background = svgElement("rect", { x: 0, y: 0, width: 600, height: 500, class: "map-background" });
  floorMap.append(background);
  const image = mapData.image;
  if (image && image.data_url) {
    // Placed by its own corners rather than stretched over the viewport: the
    // bounds can be wider than the grid when a saved location sits off the
    // mapped area, and a grid stretched to fit would put walls where there
    // are none.
    const topLeft = toScreen({ x: image.min_x, y: image.max_y });
    const bottomRight = toScreen({ x: image.max_x, y: image.min_y });
    floorMap.append(svgElement("image", {
      x: topLeft.x, y: topLeft.y,
      width: Math.max(0, bottomRight.x - topLeft.x),
      height: Math.max(0, bottomRight.y - topLeft.y),
      href: image.data_url, class: "map-grid", preserveAspectRatio: "none",
    }));
  }
  (mapData.walls || []).forEach((wall) => {
    const points = wall.map(([x, y]) => { const screen = toScreen({ x, y }); return `${screen.x},${screen.y}`; }).join(" ");
    floorMap.append(svgElement("polyline", { points, class: "map-wall" }));
  });
  if (mapData.robot) {
    const robot = toScreen(mapData.robot);
    floorMap.append(svgElement("circle", { cx: robot.x, cy: robot.y, r: 13, class: "robot-marker" }));
  }
  mapNotice.textContent = mapData.available === false ? (mapData.message || "No map yet.") : "";
  mapNotice.hidden = !mapNotice.textContent;
  (mapData.locations || []).forEach((location) => {
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

voiceForm.addEventListener("submit", async (event) => {
  event.preventDefault();
  const transcript = voiceInput.value.trim();
  if (!transcript) return;
  voiceButton.disabled = true;
  try {
    const result = await request("/api/voice", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ transcript }),
    });
    voiceResponse.textContent = result.response;
    voiceResponse.dataset.action = result.action;
    voiceInput.value = "";
    await refreshStatus();
  } catch (error) {
    voiceResponse.textContent = error.message;
    voiceResponse.dataset.action = "refused";
  } finally { voiceButton.disabled = false; }
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

// On failure neither button assumes anything about the latch: it re-reads,
// and refreshStatus marks the latch unknown if that read fails too. A stop
// request that errored may still have been applied.
estopButton.addEventListener("click", async () => {
  estopButton.disabled = true;
  try { renderStatus(await request("/api/emergency_stop", { method: "POST" })); }
  catch (error) { showError(error); await refreshStatus(); }
  finally { estopButton.disabled = false; }
});

estopReset.addEventListener("click", async () => {
  if (!window.confirm("Release the emergency stop? Check the robot is clear first.")) return;
  estopReset.disabled = true;
  try { renderStatus(await request("/api/emergency_stop/reset", { method: "POST" })); }
  catch (error) { showError(error); await refreshStatus(); }
  finally { estopReset.disabled = false; }
});

locationForm.addEventListener("submit", async (event) => {
  event.preventDefault();
  const name = document.querySelector("#location-name").value.trim();
  if (!name) return;
  const submit = locationForm.querySelector("button[type=submit]");
  submit.disabled = true;
  try {
    const data = await request("/api/locations", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        name,
        x: Number(document.querySelector("#location-x").value),
        y: Number(document.querySelector("#location-y").value),
        yaw: Number(document.querySelector("#location-yaw").value),
      }),
    });
    renderLocations(data.locations);
    locationResponse.textContent = `Saved ${name}.`;
    document.querySelector("#location-name").value = "";
  } catch (error) {
    locationResponse.textContent = error.message;
  } finally { submit.disabled = false; }
});

lastStatusAt = Date.now();
Promise.all([loadLocations(), loadMap()]).catch(showError);
setInterval(refreshStatus, STATUS_POLL_MS);
// Without this a fetch that hangs rather than fails leaves the panel - the
// latch included - showing its last good reading for as long as the page is
// open. checkStatusFreshness was written for that and was never scheduled.
setInterval(checkStatusFreshness, STATUS_POLL_MS);
// The robot moves, so the pose on the map goes stale between reads. A failed
// map read keeps the last picture rather than blanking it; the marker is
// drawn only when the server actually sent a pose.
setInterval(() => { loadMap().catch(() => {}); }, MAP_POLL_MS);
setInterval(checkStatusFreshness, STATUS_POLL_MS);
