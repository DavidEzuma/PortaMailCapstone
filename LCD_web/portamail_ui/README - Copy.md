# PortaMail LCD UI

This interface is language-agnostic and safe for C/C++ clients.

## Run
```bash
cd LCD_web/portamail_ui
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python app.py
```
Open `http://127.0.0.1:5050`.

Optional: hide debug panel:
```bash
LCD_SHOW_DEBUG_PANEL=0 python app.py
```

## External Send (commands)
Set mode:
```bash
curl -X POST http://127.0.0.1:5050/api/mode \
  -H "Content-Type: application/json" \
  -d '{"mode":"ARRIVED"}'

curl -X POST http://127.0.0.1:5050/api/mode \
  -H "Content-Type: application/json" \
  -d '{"mode":"DOCK_IDLE"}'
```

## External Receive (state + events)
- `GET /api/state` returns the full snapshot with bits and current screen.
- `GET /api/events` returns a bounded queue (max 50), ordered oldest-first.
  - Optional `since_ts=<ISO8601>` returns only newer events.
  - Optional `limit=<N>` returns only the newest `N` events.

## Socket.IO Press/Release Contract
Buttons emit two Socket.IO events:
- `press`: `{ "bit": "<bit_name>" }` (bit set to 1 while pressed)
- `release`: `{ "bit": "<bit_name>", "edge": "<edge_name>" }` (bit reset to 0, edge emitted on release)

For cancel/leave, release is sent without an edge:
`{ "bit": "<bit_name>" }`

## Event Object Schema
Events preserve the legacy keys and add canonical keys:
```json
{
  "ts": "<ISO8601>",
  "name": "<event_name>",
  "payload": { ... },
  "type": "<event_name>",
  "data": { ... }
}
```
Legacy keys (`name`/`payload`) remain unchanged for backward compatibility.

## External simulator (no ROS)
Run the stdlib-only external simulator:
```bash
python3 tools/external_sim.py
```
The simulator now uses incremental event polling (`since_ts`) to reduce duplicate reads.

## Home Screen
- Includes Room 1 and Room 2 with white-outline destination buttons.
- Tap one or both room buttons to select (selected turns green outline), then press `Deliver` to start.

## Contract + Regression Checks
- Locked contract reference: `AS_BUILT_CONTRACT.md`
- Run automated checks:
```bash
make check
```
- Run tests only:
```bash
make test
```
- Run smoke only:
```bash
make smoke
```
