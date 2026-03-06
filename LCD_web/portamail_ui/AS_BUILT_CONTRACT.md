# PortaMail LCD UI: As-Built Contract (Code-Locked)

This document locks behavior to the current implementation in `LCD_web/portamail_ui`.
It is intentionally implementation-first and contract-preserving.

## Scope

This Flask service is the LCD HMI/state authority. It does:
- UI state progression and screen selection.
- Bit tracking for button press/release semantics.
- Edge-event production and bounded event history.
- Delivery confirmation logging.

It does not implement robot navigation, motor/SLAM/ROS mission control.

## Stable API Endpoints

### `GET /api/state`
Returns:
- `mode`
- `screen`
- `selected_room`
- `pending_rooms`
- `active_room`
- `bits`
- `events`

### `GET /api/events`
Returns ordered event history list (oldest first), capped to 50 items.
Optional query parameters:
- `since_ts=<ISO8601>` returns only events newer than the provided timestamp.
- `limit=<N>` returns only the newest N events.

### `POST /api/mode`
Request body:
```json
{ "mode": "ARRIVED" }
```
Allowed modes:
- `DOCK_IDLE`
- `ARRIVED`

Invalid mode response:
```json
{ "ok": false, "error": "unknown_mode" }
```
with HTTP `400`.

## Bit Keys (unchanged)

- `room1_start_pressed`
- `room2_start_pressed`
- `power_pressed`
- `confirm_package_pressed`
- `confirm_room1_pressed`
- `confirm_room2_pressed`
- `package_confirmed_pressed`

## Edge Event Names (unchanged)

- `start_room1`
- `start_room2`
- `power_edge`
- `open_confirm_flow`
- `select_room1`
- `select_room2`
- `delivery_confirmed`

## Event Object Schema (backward-compatible)

Every event object includes:
- `ts`
- `name`
- `payload`
- `type`
- `data`

`name/payload` are preserved while `type/data` mirror them.
`payload`/`data` include state context fields:
- `mode`
- `screen`
- `active_room`
- `pending_rooms`

## Screen and Mode Semantics

- Mode values are currently limited to `DOCK_IDLE` and `ARRIVED`.
- Delivering state is represented by `screen`:
  - `DELIVERING_ROOM1`
  - `DELIVERING_ROOM2`
- Confirm flow screens:
  - `ARRIVED` -> `CONFIRM_SELECT` -> `CONFIRM_ACK`

## Queue Semantics (as implemented)

- Rooms are queued in `pending_rooms`.
- Queue ordering enforces `ROOM1` before `ROOM2` when both are pending.
- `active_room` is set from queue when entering a delivering state.

## Validation and Gating

- `open_confirm_flow` is accepted only when mode/screen indicates `ARRIVED`.
- `select_room1/select_room2` are accepted only on `CONFIRM_SELECT`.
- `delivery_confirmed` is accepted only on `CONFIRM_ACK` with a selected room.
- Only known, validated edges are appended to event history.

## Logging Contract

Confirmed deliveries append to:
- `logs/delivery_log.txt`

Line format:
- `<ISO8601_WITH_OFFSET>,ROOMX`

Example:
- `2026-02-11T15:42:10.123456-05:00,ROOM1`

Logging occurs only on accepted `delivery_confirmed`.
