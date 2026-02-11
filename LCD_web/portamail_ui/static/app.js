const socket = io();

const dbgMode = document.getElementById("dbgMode");
const dbgScreen = document.getElementById("dbgScreen");
const dbgRoom = document.getElementById("dbgRoom");
const dbgBits = document.getElementById("dbgBits");
const dbgEvents = document.getElementById("dbgEvents");

const screens = Array.from(document.querySelectorAll("[data-screen]"));
const homeStartButtons = Array.from(document.querySelectorAll(".home-start"));

function renderState(state) {
  dbgMode.textContent = state.mode || "-";
  dbgScreen.textContent = state.screen || "-";
  dbgRoom.textContent = state.selected_room || "-";
  dbgBits.textContent = JSON.stringify(state.bits || {}, null, 2);

  screens.forEach((el) => {
    el.classList.toggle("active", el.dataset.screen === state.screen);
  });

  const disableHome = state.screen !== "HOME";
  homeStartButtons.forEach((btn) => {
    btn.disabled = disableHome;
  });
}

function renderEvents(events) {
  dbgEvents.textContent = JSON.stringify(events || [], null, 2);
}

function bindButtons() {
  const buttons = document.querySelectorAll("button[data-bit][data-edge]");
  buttons.forEach((btn) => {
    const bit = btn.dataset.bit;
    const edge = btn.dataset.edge;

    const onDown = (e) => {
      e.preventDefault();
      socket.emit("press", { bit });
    };

    const onUp = (e, allowEdge) => {
      e.preventDefault();
      if (allowEdge) {
        socket.emit("release", { bit, edge });
      } else {
        socket.emit("release", { bit });
      }
    };

    btn.addEventListener("pointerdown", onDown);
    btn.addEventListener("pointerup", (e) => onUp(e, true));
    btn.addEventListener("pointercancel", (e) => onUp(e, false));
    btn.addEventListener("pointerleave", (e) => onUp(e, false));
  });
}

socket.on("state", (state) => {
  renderState(state || {});
});

socket.on("events", (events) => {
  renderEvents(events || []);
});

bindButtons();
