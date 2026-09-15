const grid = document.querySelector("#location-grid");
const connection = document.querySelector("#connection");
const statusLabel = document.querySelector("#safety-label");
const statusMessage = document.querySelector("#status-message");
const statusMark = document.querySelector("#status-mark");
const cancelButton = document.querySelector("#cancel-button");

async function request(path, options = {}) {
  const response = await fetch(path, options);
  const body = await response.json();
  if (!response.ok) throw new Error(body.error || "The robot could not complete that request.");
  return body;
}

function setBusy(busy) {
  document.querySelectorAll(".location-card").forEach((button) => { button.disabled = busy; });
}

function showError(error) {
  statusMark.textContent = "!";
  statusLabel.textContent = "Please try again";
  statusMessage.textContent = error.message;
}

function renderStatus(data) {
  const goal = data.goal;
  connection.classList.add("online");
  connection.lastChild.textContent = " Connected to robot";
  statusLabel.textContent = data.safety_message;
  statusMark.textContent = data.safety_state === "clear" ? "OK" : "!";
  statusMessage.textContent = goal.message || "The robot is ready for a destination.";
  cancelButton.hidden = goal.state !== "navigating";
}

async function refreshStatus() {
  try { renderStatus(await request("/api/status")); }
  catch (error) { connection.classList.remove("online"); showError(error); }
}

async function chooseLocation(name, button) {
  setBusy(true);
  button.querySelector(".go-label").textContent = "Sending...";
  try {
    await request("/api/goals", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ location_name: name }) });
    await refreshStatus();
  } catch (error) { showError(error); }
  finally { setBusy(false); button.querySelector(".go-label").textContent = "Go here"; }
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

cancelButton.addEventListener("click", async () => {
  cancelButton.disabled = true;
  try { await request("/api/goals/cancel", { method: "POST" }); await refreshStatus(); }
  catch (error) { showError(error); }
  finally { cancelButton.disabled = false; }
});

loadLocations();
setInterval(refreshStatus, 3000);