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
  button.textContent = "Sending...";
  try {
    await request("/api/goals", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ location_name: name }) });
    await refreshStatus();
  } catch (error) { showError(error); }
  finally { setBusy(false); button.textContent = "Go here"; }
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
loadLocations();
setInterval(refreshStatus, STATUS_POLL_MS);
setInterval(checkStatusFreshness, STATUS_POLL_MS);