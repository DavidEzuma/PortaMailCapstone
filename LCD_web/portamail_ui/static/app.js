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
const confirmBackOverlay = document.getElementById("confirmBackOverlay");
const confirmBackYes = document.getElementById("confirmBackYes");
const confirmBackNo = document.getElementById("confirmBackNo");
const mappingBackBtn = document.getElementById("mappingBackBtn");
const destinationButtons = Array.from(document.querySelectorAll(".destination-btn"));
const deliverStartBtn = document.getElementById("deliverStartBtn");

const screens = Array.from(document.querySelectorAll("[data-screen]"));
let introDismissed = false;
let currentScreen = "MODE_SELECT";
let selectedRoom = null;

// Tracks which locations have been marked this mapping session (client-side only)
const savedLocations = new Set();

// Timer for processing screen timeout warning
let processingTimeout = null;

const AUTO_DISMISS_SCREENS = new Set([
  "DELIVERING_ROOM1",
  "DELIVERING_ROOM2",
  "ARRIVED",
  "CONFIRM_SELECT",
  "CONFIRM_ACK",
  "MAPPING",
  "SAVE_MAP_SELECT",
  "SAVE_LOCATION_SELECT",
  "PROCESSING",
]);

// --- Intro ---

function dismissIntro() {
  if (!introOverlay || introDismissed) return;
  introDismissed = true;
  introOverlay.classList.add("hidden");
}

// --- Destination selection (navigation mode) ---

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
  if (currentScreen !== "HOME" || selectedRoom === null) return;
  const room = selectedRoom;
  clearDestinationSelection();
  updateDestinationControls();
  emitStartForRoom(room);
}

// --- Power confirmation ---

function showPowerConfirm() {
  if (powerConfirmOverlay) powerConfirmOverlay.classList.remove("hidden");
}

function hidePowerConfirm() {
  if (powerConfirmOverlay) powerConfirmOverlay.classList.add("hidden");
}

function bindPowerConfirm() {
  if (powerConfirmYes) {
    powerConfirmYes.addEventListener("pointerup", (e) => {
      e.preventDefault();
      hidePowerConfirm();
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

// --- Confirm back overlay (mapping mode) ---

function showConfirmBack() {
  if (confirmBackOverlay) confirmBackOverlay.classList.remove("hidden");
}

function hideConfirmBack() {
  if (confirmBackOverlay) confirmBackOverlay.classList.add("hidden");
}

function bindConfirmBack() {
  if (confirmBackYes) {
    confirmBackYes.addEventListener("pointerup", (e) => {
      e.preventDefault();
      hideConfirmBack();
      savedLocations.clear();
      updateLocationBadges();
      emitTap("back_pressed", "go_back");
    });
  }
  if (confirmBackNo) {
    confirmBackNo.addEventListener("pointerup", (e) => {
      e.preventDefault();
      hideConfirmBack();
    });
  }
}

// --- Mapping back button (special: may show confirm overlay) ---

function bindMappingBackBtn() {
  if (!mappingBackBtn) return;
  mappingBackBtn.addEventListener("pointerdown", (e) => {
    e.preventDefault();
    socket.emit("press", { bit: "back_pressed" });
  });
  mappingBackBtn.addEventListener("pointerup", (e) => {
    e.preventDefault();
    socket.emit("release", { bit: "back_pressed" });
    if (savedLocations.size > 0) {
      showConfirmBack();
    } else {
      emitTap("back_pressed", "go_back");
    }
  });
  mappingBackBtn.addEventListener("pointercancel", (e) => {
    e.preventDefault();
    socket.emit("release", { bit: "back_pressed" });
  });
  mappingBackBtn.addEventListener("pointerleave", (e) => {
    e.preventDefault();
    socket.emit("release", { bit: "back_pressed" });
  });
}

// --- Location badges (MAPPING screen) ---

const EDGE_TO_LOCATION = {
  mark_location_room1: "ROOM1",
  mark_location_room2: "ROOM2",
  mark_location_origin: "ORIGIN",
};

function updateLocationBadges() {
  document.querySelectorAll(".location-badge[data-location]").forEach((badge) => {
    badge.classList.toggle("saved", savedLocations.has(badge.dataset.location));
  });
  // Visually indicate already-saved locations in SAVE_LOCATION_SELECT
  const markRoom1 = document.getElementById("markRoom1Btn");
  const markRoom2 = document.getElementById("markRoom2Btn");
  const markOrigin = document.getElementById("markOriginBtn");
  if (markRoom1) markRoom1.classList.toggle("location-already-saved", savedLocations.has("ROOM1"));
  if (markRoom2) markRoom2.classList.toggle("location-already-saved", savedLocations.has("ROOM2"));
  if (markOrigin) markOrigin.classList.toggle("location-already-saved", savedLocations.has("ORIGIN"));
}

// --- Processing screen timeout ---

function startProcessingTimeout() {
  clearTimeout(processingTimeout);
  processingTimeout = setTimeout(() => {
    if (currentScreen === "PROCESSING") {
      const statusEl = document.querySelector("#screenProcessing .processing-status");
      if (statusEl) statusEl.textContent = "Taking longer than expected…";
    }
  }, 15000);
}

function clearProcessingTimeout() {
  clearTimeout(processingTimeout);
  processingTimeout = null;
  const statusEl = document.querySelector("#screenProcessing .processing-status");
  if (statusEl) statusEl.textContent = "Please wait. Do not power off the robot.";
}

// --- State rendering ---

function renderState(state) {
  const prevScreen = currentScreen;
  currentScreen = state.screen || currentScreen;

  if (!introDismissed && AUTO_DISMISS_SCREENS.has(currentScreen)) {
    dismissIntro();
  }

  if (dbgMode) dbgMode.textContent = state.mode || "-";
  if (dbgScreen) dbgScreen.textContent = state.screen || "-";
  if (dbgRoom) dbgRoom.textContent = state.selected_room || "-";
  if (dbgBits) dbgBits.textContent = JSON.stringify(state.bits || {}, null, 2);

  screens.forEach((el) => {
    el.classList.toggle("active", el.dataset.screen === state.screen);
  });

  // Clear saved locations when returning to mode select
  if (currentScreen === "MODE_SELECT" && prevScreen !== "MODE_SELECT") {
    savedLocations.clear();
    updateLocationBadges();
  }

  // Processing screen: start timeout on entry, clear on exit
  if (currentScreen === "PROCESSING" && prevScreen !== "PROCESSING") {
    startProcessingTimeout();
  } else if (currentScreen !== "PROCESSING" && prevScreen === "PROCESSING") {
    clearProcessingTimeout();
  }

  if (currentScreen !== "HOME" && selectedRoom !== null) {
    clearDestinationSelection();
  }
  updateDestinationControls();
  updateLocationBadges();
}

function renderEvents(events) {
  if (dbgEvents) {
    dbgEvents.textContent = JSON.stringify(events || [], null, 2);
  }
}

// --- Button binding ---

function bindButtons() {
  const buttons = document.querySelectorAll("button[data-bit][data-edge]");
  buttons.forEach((btn) => {
    const bit = btn.dataset.bit;
    const edge = btn.dataset.edge;

    // Power buttons: show confirmation overlay
    if (edge === "power_edge") {
      btn.addEventListener("pointerdown", (e) => {
        e.preventDefault();
        socket.emit("press", { bit });
      });
      btn.addEventListener("pointerup", (e) => {
        e.preventDefault();
        socket.emit("release", { bit });
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
        // Track location marks client-side for badge display
        if (edge in EDGE_TO_LOCATION) {
          savedLocations.add(EDGE_TO_LOCATION[edge]);
          updateLocationBadges();
        }
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
  if (!introOverlay) return;
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
      if (currentScreen !== "HOME") return;
      const room = btn.dataset.roomTarget;
      if (!room) return;
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
      if (e.key === "Enter" || e.key === " ") startSelectedDeliveries(e);
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
bindConfirmBack();
bindMappingBackBtn();
bindDestinationSelection();
updateDestinationControls();
updateLocationBadges();
