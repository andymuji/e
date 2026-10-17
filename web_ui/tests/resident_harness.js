// Behaviour tests for web/resident.js, run by test_app.ResidentScriptTests.
//
// No browser and no npm packages: the page's own HTML is parsed into a small
// fake DOM, fetch is answered by a fake robot, and time only moves when a
// scenario moves it. Prints one JSON object, {scenario: null | "failure"}.

const fs = require("fs");
const path = require("path");
const vm = require("vm");

const WEB = path.join(__dirname, "..", "web");
const SCRIPT = fs.readFileSync(path.join(WEB, "resident.js"), "utf8");
const HTML = fs.readFileSync(path.join(WEB, "resident.html"), "utf8");

// ---------------------------------------------------------------- fake DOM

class FakeElement {
  constructor(doc, tag, attributes = {}) {
    this.doc = doc;
    this.tagName = tag.toUpperCase();
    this.attributes = { ...attributes };
    this.id = attributes.id || "";
    this.className = attributes.class || "";
    this.hidden = "hidden" in attributes;
    this.disabled = "disabled" in attributes;
    this.type = attributes.type || "";
    this.children = [];
    this.parent = null;
    this._text = "";
    this.listeners = {};
    const element = this;
    this.classList = {
      contains: (name) => element.className.split(/\s+/).includes(name),
      add: (name) => { if (!element.classList.contains(name)) element.className = `${element.className} ${name}`.trim(); },
      remove: (name) => { element.className = element.className.split(/\s+/).filter((c) => c && c !== name).join(" "); },
      toggle: (name, on) => {
        const want = on === undefined ? !element.classList.contains(name) : Boolean(on);
        if (want) element.classList.add(name); else element.classList.remove(name);
        return want;
      },
    };
  }

  get textContent() {
    return this.children.length ? this.children.map((child) => child.textContent).join("") : this._text;
  }

  set textContent(value) { this.children = []; this._text = String(value); }

  setAttribute(name, value) { this.attributes[name] = String(value); }
  getAttribute(name) { return name in this.attributes ? this.attributes[name] : null; }
  removeAttribute(name) { delete this.attributes[name]; }
  addEventListener(type, listener) { (this.listeners[type] ||= []).push(listener); }

  dispatch(type, extra = {}) {
    let prevented = false;
    const event = { type, target: this, key: extra.key, preventDefault() { prevented = true; } };
    for (let node = this; node; node = node.parent) {
      for (const listener of node.listeners[type] || []) listener(event);
    }
    return !prevented;
  }

  click() { if (!this.disabled && !this.isHidden()) this.dispatch("click"); }
  focus() { this.doc.activeElement = this; }
  isHidden() { for (let node = this; node; node = node.parent) if (node.hidden) return true; return false; }

  append(...children) { children.forEach((child) => { child.parent = this; this.children.push(child); }); }
  replaceChildren(...children) { this.children = []; this._text = ""; this.append(...children); }

  contains(other) { for (let node = other; node; node = node.parent) if (node === this) return true; return false; }

  querySelectorAll(selector) {
    const name = selector.replace(/^\./, "");
    const found = [];
    const walk = (node) => node.children.forEach((child) => {
      if (child.classList.contains(name)) found.push(child);
      walk(child);
    });
    walk(this);
    // A NodeList, not an Array: forEach and iteration, but no find or map.
    return { length: found.length, item: (i) => found[i] || null, forEach: (fn) => found.forEach(fn), [Symbol.iterator]: () => found[Symbol.iterator]() };
  }
}

const VOID = new Set(["meta", "link", "br", "img", "input", "hr"]);

function buildDocument() {
  const doc = { activeElement: null };
  const root = new FakeElement(doc, "html");
  doc.body = root;
  doc.activeElement = root;
  const byId = {};
  const stack = [root];
  const body = HTML.replace(/<!--[\s\S]*?-->/g, "").replace(/<script[\s\S]*?<\/script>/g, "");
  const tokens = body.split(/(<[^>]+>)/);
  for (const token of tokens) {
    const tag = /^<(\/?)([a-zA-Z0-9]+)([^>]*)>$/.exec(token);
    if (!tag) {
      const text = token.trim();
      const top = stack[stack.length - 1];
      if (text && !top.children.length) top._text += text;
      continue;
    }
    const [, closing, name, rest] = tag;
    if (closing) { stack.pop(); continue; }
    if (name.toLowerCase() === "html" || token.startsWith("<!")) continue;
    const attributes = {};
    for (const match of rest.matchAll(/([a-zA-Z-]+)(?:="([^"]*)")?/g)) attributes[match[1]] = match[2] === undefined ? "" : match[2];
    const element = new FakeElement(doc, name, attributes);
    stack[stack.length - 1].append(element);
    if (attributes.id) byId[attributes.id] = element;
    if (!VOID.has(name.toLowerCase()) && !rest.endsWith("/")) stack.push(element);
  }
  doc.querySelector = (selector) => byId[selector.replace(/^#/, "")] || null;
  doc.createElement = (tag) => new FakeElement(doc, tag);
  return { doc, byId };
}

// ------------------------------------------------------------ fake robot

function fakeRobot() {
  const robot = {
    latch: false,
    state: "clear",
    reason: "path clear",
    goal: { state: "ready", location_name: null, goal_id: null },
    rooms: ["kitchen", "bedroom"],
    // How each request is answered: "ok", "fail" (HTTP 500), "down"
    // (network error), "hang" (never answers), or a delay in ms.
    mode: { status: "ok", stop: "ok", goals: "ok", cancel: "ok", locations: "ok" },
    // A stop reply under ROS: the gate has not yet echoed the latch.
    stopConfirmsAfter: 0,
    stopsSeen: 0,
    requests: [],
  };
  robot.envelope = () => {
    const confirmed = robot.latch && robot.stopsSeen > robot.stopConfirmsAfter;
    const latch = robot.latch ? (confirmed ? true : null) : false;
    return {
      goal: { ...robot.goal },
      safety_state: robot.state,
      safety: { state: latch === null ? "unknown" : robot.state, reason: robot.reason, emergency_stop: latch },
    };
  };
  robot.answer = (path, method) => {
    robot.requests.push(`${method} ${path}`);
    if (method === "GET" && path === "/api/status") return ["status", () => robot.envelope()];
    if (method === "GET" && path === "/api/locations") return ["locations", () => ({ locations: robot.rooms.map((name) => ({ name })) })];
    if (method === "POST" && path === "/api/emergency_stop") {
      return ["stop", () => { robot.latch = true; robot.stopsSeen += 1; robot.state = "stop"; robot.goal = { state: "cancelled", location_name: null, goal_id: null }; return robot.envelope(); }];
    }
    if (method === "POST" && path === "/api/goals") {
      return ["goals", (body) => {
        const name = JSON.parse(body).location_name;
        robot.goal = { state: "navigating", location_name: name, goal_id: `g${robot.requests.length}` };
        return { goal_id: robot.goal.goal_id, location_name: name };
      }];
    }
    if (method === "POST" && path === "/api/goals/cancel") {
      return ["cancel", () => { robot.goal = { state: "cancelled", location_name: null, goal_id: null }; return { message: "Trip cancelled" }; }];
    }
    return ["other", () => { throw new Error(`unexpected request ${method} ${path}`); }];
  };
  robot.count = (line) => robot.requests.filter((request) => request === line).length;
  return robot;
}

// ------------------------------------------------------------ page loader

const settle = async () => { for (let i = 0; i < 30; i += 1) await new Promise((resolve) => setImmediate(resolve)); };

function loadPage(robot) {
  const { doc, byId } = buildDocument();
  let now = 1_000_000;
  let nextTimer = 1;
  const timers = new Map();
  const schedule = (fn, ms, repeat) => {
    const id = nextTimer++;
    timers.set(id, { at: now + Math.max(0, ms || 0), fn, repeat: repeat ? Math.max(1, ms || 0) : 0 });
    return id;
  };
  const cancel = (id) => { timers.delete(id); };

  const fetch = (path, options = {}) => new Promise((resolve, reject) => {
    const method = options.method || "GET";
    const [kind, produce] = robot.answer(path, method);
    const mode = robot.mode[kind] || "ok";
    const signal = options.signal;
    const abort = () => { const error = new Error("aborted"); error.name = "AbortError"; reject(error); };
    if (signal) {
      if (signal.aborted) { abort(); return; }
      signal.addEventListener("abort", abort);
    }
    const reply = () => {
      if (mode === "down") { reject(new TypeError("network down")); return; }
      if (mode === "fail") { resolve({ ok: false, status: 500, json: async () => ({ error: "boom" }) }); return; }
      try {
        const body = produce(options.body);
        resolve({ ok: true, status: kind === "goals" ? 201 : 200, json: async () => body });
      } catch (error) { resolve({ ok: false, status: 409, json: async () => ({ error: String(error) }) }); }
    };
    if (mode === "hang") return;
    if (typeof mode === "number") schedule(reply, mode, false); else reply();
  });

  const context = {
    document: doc,
    window: {},
    fetch,
    AbortController,
    console,
    JSON,
    Math,
    Set,
    Map,
    Array,
    Object,
    String,
    Number,
    Error,
    TypeError,
    Promise,
    Date: { now: () => now },
    setTimeout: (fn, ms) => schedule(fn, ms, false),
    clearTimeout: cancel,
    setInterval: (fn, ms) => schedule(fn, ms, true),
    clearInterval: cancel,
  };
  context.window = context;
  vm.createContext(context);
  vm.runInContext(SCRIPT, context, { filename: "resident.js" });

  const page = {
    doc,
    el: (id) => byId[id],
    robot,
    async advance(ms) {
      const target = now + ms;
      for (;;) {
        await settle();
        let next = null;
        for (const [id, timer] of timers) if (timer.at <= target && (!next || timer.at < next[1].at)) next = [id, timer];
        if (!next) break;
        const [id, timer] = next;
        now = timer.at;
        if (timer.repeat) timer.at = now + timer.repeat; else timers.delete(id);
        timer.fn();
      }
      now = target;
      await settle();
    },
    rooms: () => [...byId.rooms.querySelectorAll(".room-button")],
    room: (name) => page.rooms().find((button) => button.textContent === name),
    // What a resident reads: the status line, or the alert when there is one.
    message: () => byId.alert.textContent || byId.status.textContent,
  };
  return page;
}

// -------------------------------------------------------------- scenarios

function check(condition, message) { if (!condition) throw new Error(message); }

const scenarios = {
  async "never says Ready before a status read"() {
    const robot = fakeRobot();
    robot.mode.status = "hang";
    const page = loadPage(robot);
    await page.advance(1000);
    check(page.message() !== "Ready", `said "${page.message()}"`);
    check(page.rooms().every((button) => button.disabled), "rooms were enabled");
  },

  async "says Ready and enables rooms after a clear read"() {
    const page = loadPage(fakeRobot());
    await page.advance(100);
    check(page.message() === "Ready", `said "${page.message()}"`);
    check(page.rooms().length === 2 && page.rooms().every((button) => !button.disabled), "rooms not enabled");
  },

  async "an idle gate whose only complaint is no motion command is Ready"() {
    for (const reason of ["motion command timed out", "no motion command received"]) {
      const robot = fakeRobot();
      robot.state = "stop";
      robot.reason = reason;
      const page = loadPage(robot);
      await page.advance(100);
      check(page.message() === "Ready", `${reason}: said "${page.message()}"`);
    }
  },

  async "a gate that says stop for any other reason means can't move"() {
    for (const [state, reason] of [["stop", "obstacle inside stop distance"], ["stop", "obstacle sensor timed out"], ["wobbly", "?"]]) {
      const robot = fakeRobot();
      robot.state = state;
      robot.reason = reason;
      const page = loadPage(robot);
      await page.advance(100);
      check(page.el("alert").textContent === "The robot can't move right now. Please wait, or ask your carer.", `${reason}: said "${page.message()}"`);
      check(page.rooms().every((button) => button.disabled), `${reason}: rooms enabled`);
    }
  },

  async "a room asks first, in the room's own words"() {
    const page = loadPage(fakeRobot());
    await page.advance(100);
    page.room("kitchen").click();
    check(!page.el("confirm").hidden, "no confirmation shown");
    check(page.el("confirm-question").textContent === "Go to kitchen?", `asked "${page.el("confirm-question").textContent}"`);
    check(page.el("yes-button").getAttribute("aria-label") === "Yes, go to kitchen", "Yes is not labelled with the room");
    check(page.robot.count("POST /api/goals") === 0, "a goal was sent before Yes");
  },

  async "No closes the question and gives focus back to the room"() {
    const page = loadPage(fakeRobot());
    await page.advance(100);
    page.room("kitchen").focus();
    page.room("kitchen").click();
    page.el("no-button").focus();
    page.el("no-button").click();
    check(page.el("confirm").hidden, "confirmation still shown");
    check(page.doc.activeElement === page.room("kitchen"), "focus was not returned to the room");
  },

  async "Yes sends one goal and shows the trip"() {
    const page = loadPage(fakeRobot());
    await page.advance(100);
    page.room("kitchen").click();
    page.el("yes-button").click();
    await page.advance(100);
    check(page.robot.count("POST /api/goals") === 1, "goal not sent once");
    check(page.message() === "On the way to kitchen", `said "${page.message()}"`);
    check(!page.el("cancel-button").hidden, "no Cancel trip during a trip");
  },

  async "a poll during the goal request cannot re-enable Yes"() {
    const robot = fakeRobot();
    const page = loadPage(robot);
    await page.advance(100);
    robot.mode.goals = "hang";
    page.room("kitchen").click();
    page.el("yes-button").click();
    await page.advance(2500);
    check(page.el("yes-button").disabled || page.el("confirm").hidden, "Yes was re-enabled while the goal was being sent");
    page.el("yes-button").click();
    await page.advance(100);
    check(robot.count("POST /api/goals") === 1, `sent ${robot.count("POST /api/goals")} goals`);
  },

  async "any failure to start a trip says so plainly, until the trip state changes"() {
    for (const mode of ["fail", "down"]) {
      const robot = fakeRobot();
      const page = loadPage(robot);
      await page.advance(100);
      robot.mode.goals = mode;
      page.room("kitchen").click();
      page.el("yes-button").click();
      await page.advance(100);
      check(page.el("alert").textContent === "The robot did not start. Please try again, or ask your carer.", `${mode}: said "${page.message()}"`);
      await page.advance(2100);
      check(page.el("alert").textContent.startsWith("The robot did not start"), `${mode}: cleared by an unchanged poll`);
      robot.goal = { state: "navigating", location_name: "bedroom", goal_id: "elsewhere" };
      await page.advance(2100);
      check(page.message() === "On the way to bedroom", `${mode}: still "${page.message()}" after the trip changed`);
    }
  },

  async "STOP says Stopping and keeps rooms off until the latch is confirmed"() {
    const robot = fakeRobot();
    robot.stopConfirmsAfter = 2;
    const page = loadPage(robot);
    await page.advance(100);
    page.el("stop-button").click();
    await page.advance(10);
    check(page.message() === "Stopping…", `said "${page.message()}"`);
    check(!page.message().includes("Can't reach"), "a 2xx stop reply rendered as unreachable");
    check(page.rooms().every((button) => button.disabled), "rooms enabled before the stop was confirmed");
    await page.advance(2500);
    check(page.el("alert").textContent === "Stopped. Please ask your carer to reset the robot.", `said "${page.message()}"`);
    check(page.el("stop-alert").textContent === "", "the stop warning stayed after confirmation");
  },

  async "a STOP that fails warns at once and keeps trying"() {
    const robot = fakeRobot();
    const page = loadPage(robot);
    await page.advance(100);
    robot.mode.stop = "down";
    page.el("stop-button").click();
    await page.advance(10);
    check(page.el("stop-alert").textContent === "The stop may not have worked. Press the red button on the robot or call your carer.", "no warning after a failed stop");
    await page.advance(3000);
    check(robot.count("POST /api/emergency_stop") >= 3, `only ${robot.count("POST /api/emergency_stop")} stop attempts`);
    check(page.el("stop-alert").textContent !== "", "the warning did not persist");
    robot.mode.stop = "ok";
    await page.advance(1500);
    check(page.el("stop-alert").textContent === "", "warning not cleared once the stop held");
    check(page.el("alert").textContent.startsWith("Stopped."), `said "${page.message()}"`);
  },

  async "a STOP answered but never confirmed warns after 3 seconds"() {
    const robot = fakeRobot();
    robot.stopConfirmsAfter = 1000;
    robot.mode.status = "hang";
    const page = loadPage(robot);
    await page.advance(100);
    page.el("stop-button").click();
    await page.advance(2000);
    check(page.el("stop-alert").textContent === "", "warned before 3 seconds");
    await page.advance(1500);
    check(page.el("stop-alert").textContent.startsWith("The stop may not have worked"), "no warning after 3 seconds");
  },

  async "STOP is sent even while a status poll is stuck"() {
    const robot = fakeRobot();
    const page = loadPage(robot);
    await page.advance(100);
    robot.mode.status = "hang";
    await page.advance(2100);
    page.el("stop-button").click();
    await page.advance(10);
    check(robot.count("POST /api/emergency_stop") === 1, "the stop waited behind the poll");
  },

  async "an old status reply cannot undo a newer stop on screen"() {
    const robot = fakeRobot();
    const page = loadPage(robot);
    await page.advance(100);
    // The next poll is answered late, with what the robot said before STOP.
    const stale = robot.envelope();
    robot.mode.status = 1500;
    const answer = robot.answer;
    robot.answer = (path, method) => (path === "/api/status" ? ["status", () => stale] : answer(path, method));
    await page.advance(2000);
    page.el("stop-button").click();
    await page.advance(10);
    check(page.el("alert").textContent.startsWith("Stopped."), `said "${page.message()}"`);
    await page.advance(1000);
    check(page.el("alert").textContent.startsWith("Stopped."), `the stale reply showed "${page.message()}"`);
  },

  async "a request that never answers is given up, so polling carries on"() {
    const robot = fakeRobot();
    const page = loadPage(robot);
    await page.advance(100);
    robot.mode.status = "hang";
    await page.advance(12000);
    check(robot.count("GET /api/status") >= 3, `only ${robot.count("GET /api/status")} status requests`);
  },

  async "one failed poll is not enough to say unreachable, two are"() {
    const robot = fakeRobot();
    const page = loadPage(robot);
    await page.advance(100);
    robot.mode.status = "down";
    await page.advance(2000);
    check(page.message() === "Ready", `one failure said "${page.message()}"`);
    await page.advance(2000);
    check(page.el("alert").textContent === "Can't reach the robot. Please ask your carer.", `said "${page.message()}"`);
    check(page.rooms().every((button) => button.disabled), "rooms enabled while unreachable");
  },

  async "Cancel trip stays while the robot cannot be reached mid-trip"() {
    const robot = fakeRobot();
    const page = loadPage(robot);
    await page.advance(100);
    page.room("kitchen").click();
    page.el("yes-button").click();
    await page.advance(100);
    robot.mode.status = "down";
    await page.advance(4100);
    check(page.el("alert").textContent.startsWith("Can't reach the robot"), `said "${page.message()}"`);
    check(!page.el("cancel-button").hidden, "Cancel trip hidden while the trip may still be running");
  },

  async "the alert clears when the robot answers again"() {
    const robot = fakeRobot();
    const page = loadPage(robot);
    await page.advance(100);
    robot.mode.status = "down";
    await page.advance(4100);
    robot.mode.status = "ok";
    await page.advance(2100);
    check(page.el("alert").textContent === "", `alert still "${page.el("alert").textContent}"`);
    check(page.message() === "Ready", `said "${page.message()}"`);
  },

  async "the rooms are retried every 3 seconds until they load"() {
    const robot = fakeRobot();
    robot.mode.locations = "down";
    const page = loadPage(robot);
    await page.advance(100);
    check(page.el("rooms-note").textContent === "Can't load the rooms right now. Trying again…", `note "${page.el("rooms-note").textContent}"`);
    robot.mode.locations = "ok";
    await page.advance(3100);
    check(page.rooms().length === 2, "rooms not loaded on the retry");
    check(page.el("rooms-note").textContent === "", "note not cleared");
  },

  async "the question closes if its room is removed"() {
    const robot = fakeRobot();
    const page = loadPage(robot);
    await page.advance(100);
    page.room("bedroom").click();
    robot.rooms = ["kitchen"];
    await page.advance(31000);
    check(page.el("confirm").hidden, "still asking about a room that is gone");
  },

  async "a failed trip says what to do next"() {
    const robot = fakeRobot();
    robot.goal = { state: "aborted", location_name: "kitchen", goal_id: "g" };
    const page = loadPage(robot);
    await page.advance(100);
    check(page.message() === "The robot couldn't get to kitchen. You can try again or ask your carer.", `said "${page.message()}"`);
  },
};

(async () => {
  const results = {};
  for (const [name, run] of Object.entries(scenarios)) {
    try { await run(); results[name] = null; } catch (error) { results[name] = String(error && error.stack || error); }
  }
  process.stdout.write(JSON.stringify(results));
})();
