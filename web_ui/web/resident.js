// The resident's page: pick a saved room, cancel the trip, stop the robot.
//
// It never says "Ready" until a status read has said the robot is free to
// move. A failed or silent robot is shown as "Can't reach the robot", never
// as the last good answer. STOP is not trusted until the robot confirms it.
// It cannot undo a stop: there is no such control here, and the server that
// serves this page refuses the request.

const statusLine = document.querySelector("#status");
const alertLine = document.querySelector("#alert");
const stopAlert = document.querySelector("#stop-alert");
const cancelButton = document.querySelector("#cancel-button");
const question = document.querySelector("#question");
const choose = document.querySelector("#choose");
const roomsNote = document.querySelector("#rooms-note");
const rooms = document.querySelector("#rooms");
const confirmPanel = document.querySelector("#confirm");
const confirmQuestion = document.querySelector("#confirm-question");
const yesButton = document.querySelector("#yes-button");
const noButton = document.querySelector("#no-button");
const stopButton = document.querySelector("#stop-button");

const STATUS_POLL_MS = 2000;
const STATUS_STALE_MS = 8000;
const REQUEST_TIMEOUT_MS = 5000;
const ROOMS_RETRY_MS = 3000;
const ROOMS_POLL_MS = 30000;
const REFUSAL_SHOWN_MS = 10000;
const STOP_CONFIRM_MS = 3000;
const STOP_RETRY_MS = 1000;

// What the safety gate says when the robot may be sent somewhere.
const MOVING_STATES = new Set(["clear", "caution"]);
// The gate also says "stop" whenever nothing is asking it to move, which is
// every idle moment. It checks the command stream after the sensors, the
// localisation and the battery, so these reasons mean "healthy, just idle".
const IDLE_REASONS = new Set(["motion command timed out", "no motion command received"]);

const SAY = {
  checking: "Checking the robot…",
  ready: "Ready",
  stopping: "Stopping…",
  stopped: "Stopped. Please ask your carer to reset the robot.",
  stopFailed: "The stop may not have worked. Press the red button on the robot or call your carer.",
  unreachable: "Can't reach the robot. Please ask your carer.",
  cantMove: "The robot can't move right now. Please wait, or ask your carer.",
  didNotStart: "The robot did not start. Please try again, or ask your carer.",
};

let lastStatusAt = Date.now();
let polling = false;
let failedPolls = 0;
// Replies are applied in the order their requests were sent, so a slow old
// answer cannot overwrite a newer one - a stop, say.
let sentSeq = 0;
let appliedSeq = 0;
let lastData = null;
let lastGoalKey = null;
let tripActive = false;
let canChoose = false;
let sending = false;
let refusal = null;
let pendingRoom = null;
let roomNames = null;
let roomsLoaded = false;
// While set, a STOP has been pressed and the robot has not yet confirmed it.
let stop = null;

class RequestError extends Error {
  constructor(message, status) { super(message); this.status = status; }
}

async function request(path, options = {}) {
  // Given up after a while, so a request that never answers cannot hold the
  // page on its last reading forever.
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), REQUEST_TIMEOUT_MS);
  try {
    const response = await fetch(path, { ...options, signal: controller.signal });
    let body = null;
    try { body = await response.json(); } catch (error) { body = null; }
    if (!response.ok) throw new RequestError((body && body.error) || `HTTP ${response.status}`, response.status);
    if (body === null) throw new RequestError("unreadable reply", 0);
    return body;
  } finally { clearTimeout(timer); }
}

// Routine news in the status; anything to act on in the alert. Each is
// rewritten only on a change, so a screen reader is not told it again and
// again at every poll.
function say(text, urgent = false, look = "") {
  const status = urgent ? "" : text;
  const alert = urgent ? text : "";
  if (statusLine.textContent !== status) statusLine.textContent = status;
  if (alertLine.textContent !== alert) alertLine.textContent = alert;
  statusLine.classList.toggle("is-moving", look === "moving");
}

function setCanChoose(value) {
  canChoose = value && !sending && !stop;
  rooms.querySelectorAll(".room-button").forEach((button) => { button.disabled = !canChoose; });
  yesButton.disabled = !canChoose;
  if (!canChoose && !sending) closeConfirm();
}

function movable(safety) {
  return MOVING_STATES.has(safety.state) || (safety.state === "stop" && IDLE_REASONS.has(safety.reason));
}

function showStopping() {
  say(SAY.stopping);
  cancelButton.hidden = true;
  setCanChoose(false);
}

function showUnreachable() {
  if (stop) { showStopping(); return; }
  say(SAY.unreachable, true);
  // A trip last seen running may still be running: cancelling stays possible.
  cancelButton.hidden = !tripActive;
  setCanChoose(false);
}

function applyStatus(data, seq) {
  if (seq < appliedSeq) return;
  appliedSeq = seq;
  failedPolls = 0;
  lastStatusAt = Date.now();
  const goal = data && data.goal;
  const safety = data && data.safety;
  if (stop && safety && safety.emergency_stop === true) {
    stop = null;
    stopAlert.textContent = "";
  }
  if (stop) { showStopping(); return; }
  // A reply without a definite latch says nothing about the stop, and
  // "nothing" must not be shown as ready.
  if (!goal || !safety || typeof safety.emergency_stop !== "boolean") {
    showUnreachable();
    return;
  }
  lastData = data;
  const goalKey = `${goal.goal_id}|${goal.state}`;
  if (refusal && (goalKey !== refusal.goalKey || Date.now() >= refusal.until)) refusal = null;
  lastGoalKey = goalKey;
  tripActive = goal.state === "navigating" || goal.state === "cancelling";
  const room = goal.location_name;
  cancelButton.hidden = goal.state !== "navigating" || safety.emergency_stop;

  if (safety.emergency_stop) {
    say(SAY.stopped, true);
    setCanChoose(false);
  } else if (goal.state === "navigating") {
    say(room ? `On the way to ${room}` : "On the way", false, "moving");
    setCanChoose(false);
  } else if (goal.state === "cancelling") {
    say("Stopping the trip", false, "moving");
    setCanChoose(false);
  } else if (refusal) {
    say(SAY.didNotStart, true);
    setCanChoose(movable(safety));
  } else if (!movable(safety)) {
    say(SAY.cantMove, true);
    setCanChoose(false);
  } else if (goal.state === "succeeded") {
    say(room ? `Arrived at ${room}` : "Arrived");
    setCanChoose(true);
  } else if (goal.state === "aborted" || goal.state === "rejected") {
    say(`The robot couldn't get to ${room || "the room"}. You can try again or ask your carer.`);
    setCanChoose(true);
  } else {
    say(SAY.ready);
    setCanChoose(true);
  }
}

function noteFailedPoll(seq) {
  if (seq < appliedSeq) return;
  failedPolls += 1;
  // One lost reply on home Wi-Fi is not news; two in a row are.
  if (failedPolls >= 2) showUnreachable();
}

async function refreshStatus() {
  if (polling) return;
  polling = true;
  const seq = ++sentSeq;
  try { applyStatus(await request("/api/status"), seq); }
  catch (error) { noteFailedPoll(seq); }
  finally { polling = false; }
}

// A robot that has gone quiet is noticed even if no request has failed yet.
function checkStatusFreshness() {
  if (Date.now() - lastStatusAt > STATUS_STALE_MS) showUnreachable();
}

// STOP does not wait behind the status poll, and it is not believed until a
// reply says the latch is engaged: under ROS the gate confirms after the
// reply. Until then it is sent again every second, and if it failed or is
// still unconfirmed after 3 seconds the resident is told to use the robot's
// own red button.
async function sendStop() {
  if (!stop || stop.inFlight) return;
  stop.inFlight = true;
  const seq = ++sentSeq;
  try {
    applyStatus(await request("/api/emergency_stop", { method: "POST", headers: { "Content-Type": "application/json" } }), seq);
  } catch (error) {
    if (stop) stop.failed = true;
  } finally {
    if (stop) stop.inFlight = false;
  }
  warnIfStopUnconfirmed();
}

function warnIfStopUnconfirmed() {
  if (!stop) return;
  if (stop.failed || Date.now() - stop.startedAt >= STOP_CONFIRM_MS) {
    if (stopAlert.textContent !== SAY.stopFailed) stopAlert.textContent = SAY.stopFailed;
  }
}

function pressStop() {
  if (!stop) {
    stop = { startedAt: Date.now(), failed: false, inFlight: false };
    setTimeout(warnIfStopUnconfirmed, STOP_CONFIRM_MS);
  }
  showStopping();
  sendStop();
}

function renderRooms(locations) {
  const names = Array.isArray(locations) ? locations.map((place) => String(place.name)) : [];
  if (pendingRoom !== null && !names.includes(pendingRoom)) closeConfirm();
  roomsNote.textContent = names.length ? "" : "No rooms have been saved yet. Please ask your carer.";
  if (roomNames && JSON.stringify(roomNames) === JSON.stringify(names)) return;
  roomNames = names;
  rooms.replaceChildren(...names.map((name) => {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "room-button";
    button.textContent = name;
    button.disabled = !canChoose;
    button.addEventListener("click", () => askToGo(name));
    return button;
  }));
}

async function loadRooms() {
  try {
    renderRooms((await request("/api/locations")).locations);
    roomsLoaded = true;
  } catch (error) {
    if (!roomsLoaded) roomsNote.textContent = "Can't load the rooms right now. Trying again…";
  }
  // Every 3 seconds until the rooms first arrive, then now and again.
  setTimeout(loadRooms, roomsLoaded ? ROOMS_POLL_MS : ROOMS_RETRY_MS);
}

function askToGo(name) {
  if (!canChoose) return;
  pendingRoom = name;
  confirmQuestion.textContent = `Go to ${name}?`;
  yesButton.setAttribute("aria-label", `Yes, go to ${name}`);
  choose.hidden = true;
  confirmPanel.hidden = false;
  confirmQuestion.focus();
}

function closeConfirm() {
  if (confirmPanel.hidden) return;
  const name = pendingRoom;
  const hadFocus = confirmPanel.contains(document.activeElement);
  pendingRoom = null;
  confirmPanel.hidden = true;
  choose.hidden = false;
  // Focus is moved only if it was in the question, never pulled off STOP.
  if (!hadFocus) return;
  const button = [...rooms.querySelectorAll(".room-button")].find((b) => b.textContent === name);
  if (button && !button.disabled) button.focus(); else question.focus();
}

yesButton.addEventListener("click", async () => {
  const name = pendingRoom;
  if (!name || sending || !canChoose) return;
  sending = true;
  yesButton.disabled = true;
  let failed = false;
  try {
    await request("/api/goals", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ location_name: name }),
    });
  } catch (error) {
    // Refused or unanswered, the robot is not on its way.
    failed = true;
  }
  sending = false;
  refusal = failed ? { until: Date.now() + REFUSAL_SHOWN_MS, goalKey: lastGoalKey } : null;
  closeConfirm();
  if (failed && lastData) applyStatus(lastData, appliedSeq);
  await refreshStatus();
});

noButton.addEventListener("click", closeConfirm);
confirmPanel.addEventListener("keydown", (event) => { if (event.key === "Escape") closeConfirm(); });

cancelButton.addEventListener("click", async () => {
  cancelButton.disabled = true;
  try {
    await request("/api/goals/cancel", { method: "POST", headers: { "Content-Type": "application/json" } });
  } catch (error) { /* the next status read says what happened */ }
  finally { cancelButton.disabled = false; }
  await refreshStatus();
});

// Never disabled or hidden by anything on this page: engaging is always safe.
stopButton.addEventListener("click", pressStop);

setCanChoose(false);
loadRooms();
refreshStatus();
setInterval(refreshStatus, STATUS_POLL_MS);
setInterval(checkStatusFreshness, STATUS_POLL_MS);
setInterval(() => { if (stop) { warnIfStopUnconfirmed(); sendStop(); } }, STOP_RETRY_MS);
