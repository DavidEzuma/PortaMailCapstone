const socket = io();

const dbgMode = document.getElementById("dbgMode");
const dbgScreen = document.getElementById("dbgScreen");
const dbgRoom = document.getElementById("dbgRoom");
const dbgBits = document.getElementById("dbgBits");
const dbgEvents = document.getElementById("dbgEvents");
const introOverlay = document.getElementById("introOverlay");
const powerConfirmOverlay = document.getElementById("powerConfirmOverlay");
const powerConfirmYes = document.getElementById("powerConfirmYes");
const powerConfirmNo = document.getElementById("powerConfirmNo");
const destinationButtons = Array.from(document.querySelectorAll(".destination-btn"));
const deliverStartBtn = document.getElementById("deliverStartBtn");

const screens = Array.from(document.querySelectorAll("[data-screen]"));
let introDismissed = false;
let currentScreen = "MODE_SELECT";
let selectedRoom = null;

// Screens that indicate the robot is actively working — auto-dismiss intro if
// the browser reconnects mid-operation.
const AUTO_DISMISS_SCREENS = new Set([
  "DELIVERING_ROOM1",
  "DELIVERING_ROOM2",
  "ARRIVED",
  "CONFIRM_SELECT",
  "CONFIRM_ACK",
  "MAPPING",
  "SAVE_MAP_SELECT",
]);

function dismissIntro() {
  if (!introOverlay || introDismissed) {
    return;
  }
  introDismissed = true;
  introOverlay.classList.add("hidden");
}

function clearDestinationSelection() {
  selectedRoom = null;
  destinationButtons.forEach((btn) => btn.classList.remove("selected"));
}

function updateDestinationControls() {
  const onHomeScreen = currentScreen === "HOME";
  destinationButtons.forEach((btn) => {
    btn.classList.toggle("selected", btn.dataset.roomTarget === selectedRoom);
    btn.disabled = !onHomeScreen;
  });
  if (deliverStartBtn) {
    deliverStartBtn.disabled = !onHomeScreen || selectedRoom === null;
  }
}

function emitTap(bit, edge) {
  socket.emit("press", { bit });
  socket.emit("release", { bit, edge });
}

function emitStartForRoom(room) {
  if (room === "ROOM1") {
    emitTap("room1_start_pressed", "start_room1");
  } else if (room === "ROOM2") {
    emitTap("room2_start_pressed", "start_room2");
  } else if (room === "ORIGIN") {
    emitTap("start_origin_pressed", "start_origin");
  }
}

function startSelectedDeliveries(e) {
  e.preventDefault();
  if (currentScreen !== "HOME" || selectedRoom === null) {
    return;
  }
  const room = selectedRoom;
  clearDestinationSelection();
  updateDestinationControls();
  emitStartForRoom(room);
}

// --- Power confirmation ---

function showPowerConfirm() {
  if (powerConfirmOverlay) {
    powerConfirmOverlay.classList.remove("hidden");
  }
}

function hidePowerConfirm() {
  if (powerConfirmOverlay) {
    powerConfirmOverlay.classList.add("hidden");
  }
}

function bindPowerConfirm() {
  if (powerConfirmYes) {
    powerConfirmYes.addEventListener("pointerup", (e) => {
      e.preventDefault();
      hidePowerConfirm();
      // Fire the actual shutdown edge only after confirmation
      emitTap("power_pressed", "power_edge");
    });
  }
  if (powerConfirmNo) {
    powerConfirmNo.addEventListener("pointerup", (e) => {
      e.preventDefault();
      hidePowerConfirm();
    });
  }
}

function renderState(state) {
  currentScreen = state.screen || currentScreen;

  // Auto-dismiss intro only when the robot is mid-operation (e.g., browser
  // reconnects while a delivery is in progress). MODE_SELECT and HOME require
  // a manual tap so the user sees the splash screen first.
  if (!introDismissed && AUTO_DISMISS_SCREENS.has(currentScreen)) {
    dismissIntro();
  }

  if (dbgMode) {
    dbgMode.textContent = state.mode || "-";
  }
  if (dbgScreen) {
    dbgScreen.textContent = state.screen || "-";
  }
  if (dbgRoom) {
    dbgRoom.textContent = state.selected_room || "-";
  }
  if (dbgBits) {
    dbgBits.textContent = JSON.stringify(state.bits || {}, null, 2);
  }

  screens.forEach((el) => {
    el.classList.toggle("active", el.dataset.screen === state.screen);
  });

  if (currentScreen !== "HOME" && selectedRoom !== null) {
    clearDestinationSelection();
  }
  updateDestinationControls();
}

function renderEvents(events) {
  if (dbgEvents) {
    dbgEvents.textContent = JSON.stringify(events || [], null, 2);
  }
}

function bindButtons() {
  const buttons = document.querySelectorAll("button[data-bit][data-edge]");
  buttons.forEach((btn) => {
    const bit = btn.dataset.bit;
    const edge = btn.dataset.edge;

    // Power buttons are intercepted — show confirmation overlay instead of
    // firing the edge directly.
    if (edge === "power_edge") {
      btn.addEventListener("pointerdown", (e) => {
        e.preventDefault();
        socket.emit("press", { bit });
      });
      btn.addEventListener("pointerup", (e) => {
        e.preventDefault();
        socket.emit("release", { bit }); // release bit only, no edge
        showPowerConfirm();
      });
      btn.addEventListener("pointercancel", (e) => {
        e.preventDefault();
        socket.emit("release", { bit });
      });
      btn.addEventListener("pointerleave", (e) => {
        e.preventDefault();
        socket.emit("release", { bit });
      });
      return;
    }

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

function bindIntro() {
  if (!introOverlay) {
    return;
  }
  introOverlay.addEventListener("pointerup", dismissIntro);
  introOverlay.addEventListener("click", dismissIntro);
  introOverlay.addEventListener("keydown", (e) => {
    if (e.key === "Enter" || e.key === " ") {
      e.preventDefault();
      dismissIntro();
    }
  });
}

function bindDestinationSelection() {
  destinationButtons.forEach((btn) => {
    btn.addEventListener("click", (e) => {
      e.preventDefault();
      if (currentScreen !== "HOME") {
        return;
      }
      const room = btn.dataset.roomTarget;
      if (!room) {
        return;
      }
      // Radio behaviour: selecting the same room again deselects it
      selectedRoom = selectedRoom === room ? null : room;
      updateDestinationControls();
    });

    btn.addEventListener("keydown", (e) => {
      if (e.key === "Enter" || e.key === " ") {
        e.preventDefault();
        btn.click();
      }
    });
  });

  if (deliverStartBtn) {
    deliverStartBtn.addEventListener("click", startSelectedDeliveries);
    deliverStartBtn.addEventListener("keydown", (e) => {
      if (e.key === "Enter" || e.key === " ") {
        startSelectedDeliveries(e);
      }
    });
  }
}

socket.on("state", (state) => {
  renderState(state || {});
});

socket.on("events", (events) => {
  renderEvents(events || []);
});

bindButtons();
bindIntro();
bindPowerConfirm();
bindDestinationSelection();
updateDestinationControls();
