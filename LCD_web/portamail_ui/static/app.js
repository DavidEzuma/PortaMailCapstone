const socket = io();

const dbgMode = document.getElementById("dbgMode");
const dbgScreen = document.getElementById("dbgScreen");
const dbgRoom = document.getElementById("dbgRoom");
const dbgBits = document.getElementById("dbgBits");
const dbgEvents = document.getElementById("dbgEvents");
const introOverlay = document.getElementById("introOverlay");
const destinationButtons = Array.from(document.querySelectorAll(".destination-btn"));
const selectedRoomChips = Array.from(document.querySelectorAll("[data-selected-chip]"));
const deliverStartBtn = document.getElementById("deliverStartBtn");

const screens = Array.from(document.querySelectorAll("[data-screen]"));
let introDismissed = false;
let currentScreen = "HOME";
const selectedRooms = new Set();

function dismissIntro() {
  if (!introOverlay || introDismissed) {
    return;
  }
  introDismissed = true;
  introOverlay.classList.add("hidden");
}

function clearDestinationSelection() {
  selectedRooms.clear();
  destinationButtons.forEach((btn) => {
    btn.classList.remove("selected");
  });
}

function updateDestinationControls() {
  const onHomeScreen = currentScreen === "HOME";
  destinationButtons.forEach((btn) => {
    const room = btn.dataset.roomTarget;
    btn.classList.toggle("selected", selectedRooms.has(room));
    btn.disabled = !onHomeScreen;
  });
  selectedRoomChips.forEach((chip) => {
    const room = chip.dataset.selectedChip;
    const isSelected = selectedRooms.has(room);
    chip.classList.toggle("selected", isSelected);
    chip.classList.toggle("inactive", !isSelected);
    chip.setAttribute("aria-hidden", String(!isSelected));
  });
  if (deliverStartBtn) {
    deliverStartBtn.disabled = !onHomeScreen || selectedRooms.size === 0;
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
  }
}

function startSelectedDeliveries(e) {
  e.preventDefault();
  if (currentScreen !== "HOME" || selectedRooms.size === 0) {
    return;
  }
  const orderedRooms = ["ROOM1", "ROOM2"].filter((room) => selectedRooms.has(room));
  clearDestinationSelection();
  updateDestinationControls();
  orderedRooms.forEach((room) => emitStartForRoom(room));
}

function renderState(state) {
  currentScreen = state.screen || currentScreen;

  if (!introDismissed && currentScreen && currentScreen !== "HOME") {
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

  if (currentScreen !== "HOME" && selectedRooms.size > 0) {
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
      if (selectedRooms.has(room)) {
        selectedRooms.delete(room);
      } else {
        selectedRooms.add(room);
      }
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
bindDestinationSelection();
updateDestinationControls();
