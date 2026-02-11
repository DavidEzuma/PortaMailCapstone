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
