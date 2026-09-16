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
  const body = await response.json();
  if (!response.ok) throw new Error(body.error || "The robot could not complete that request.");
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
  commandAgeValue.textContent = `${safety.command_age.toFixed(1)} s`;
  latchValue.textContent = safety.emergency_stop ? "ENGAGED" : "released";
  latchValue.classList.toggle("engaged", safety.emergency_stop);
  estopButton.hidden = safety.emergency_stop;
  estopReset.hidden = !safety.emergency_stop;
  document.body.classList.toggle("stopped", safety.emergency_stop);
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
  button.textContent = "Sending...";
  try {
    await request("/api/goals", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ location_name: name }) });
    await refreshStatus();
  } catch (error) { showError(error); }
  finally { setBusy(false); button.textContent = "Go here"; }
}

function renderLocations(locations) {
  if (!locations.length) {
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
    coords.textContent = `${x.toFixed(2)}, ${y.toFixed(2)}`;

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
    await refreshStatus();
  } catch (error) { showError(error); grid.innerHTML = "<p class='loading'>Destinations are unavailable right now.</p>"; }
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

estopButton.addEventListener("click", async () => {
  estopButton.disabled = true;
  try { renderStatus(await request("/api/emergency_stop", { method: "POST" })); }
  catch (error) { showError(error); }
  finally { estopButton.disabled = false; }
});

estopReset.addEventListener("click", async () => {
  if (!window.confirm("Release the emergency stop? Check the robot is clear first.")) return;
  estopReset.disabled = true;
  try { renderStatus(await request("/api/emergency_stop/reset", { method: "POST" })); }
  catch (error) { showError(error); }
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

loadLocations();
setInterval(refreshStatus, 3000);